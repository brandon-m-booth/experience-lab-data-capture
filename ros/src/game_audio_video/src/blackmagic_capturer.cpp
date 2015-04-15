#include "blackmagic_capturer.h"
#include <stdlib.h>
#include <stdio.h>
#include <boost/lockfree/spsc_queue.hpp>

IDeckLinkInput* BlackMagicCapturer::deckLinkInput = NULL;
BlackMagicCapturer* BlackMagicCapturer::instance = NULL;

BlackMagicCapturer* BlackMagicCapturer::GetInstance()
{
   if(!instance)
   {
      instance = new BlackMagicCapturer();
   }

   return instance;
}

BlackMagicCapturer::BlackMagicCapturer()
{
   pthread_mutex_init(&mutex, NULL);
   deckLinkAttributes = NULL;
   deckLinkIterator = NULL;
   deckLink = NULL;
   displayModeIterator = NULL;
   displayMode = NULL;
   displayModeName = NULL;
}

BlackMagicCapturer::~BlackMagicCapturer()
{
   pthread_mutex_destroy(&mutex);

   Shutdown();
   instance = NULL;
}

bool BlackMagicCapturer::Initialize(BMDPixelFormat pixelFormat,
                                    BMDVideoInputFlags inputFlags,
                                    int audioSampleDepth,
                                    int numAudioChannels)
{
   HRESULT result;

   this->pixelFormat = pixelFormat;
   this->inputFlags = inputFlags;
   this->audioSampleDepth = audioSampleDepth;
   this->numAudioChannels = numAudioChannels;

   // Create and get the first DeckLink instance
   deckLinkIterator = CreateDeckLinkIteratorInstance();
   if (!deckLinkIterator)
   {
      Shutdown();
      errorString = "Failed to create deck link iterator instance";
      return false;
   }

   if (deckLinkIterator->Next(&deckLink) != S_OK)
   {
      Shutdown();
      errorString = "Failed to get deck link instance";
      return false;
   }

   // Get the input capture interface of the DeckLink device
   if (deckLink->QueryInterface(IID_IDeckLinkInput, (void**)&deckLinkInput) != S_OK)
   {
      Shutdown();
      errorString = "Failed to query deck link input interface";
      return false;
   }

   // Check whether the input format detection is supported
   if (deckLink->QueryInterface(IID_IDeckLinkAttributes, (void**)&deckLinkAttributes) == S_OK)
   {
      bool isFormatDetectionSupported;
      result = deckLinkAttributes->GetFlag(BMDDeckLinkSupportsInputFormatDetection, &isFormatDetectionSupported);
      if (result != S_OK || !isFormatDetectionSupported)
      {
         Shutdown();
         errorString = "Unable to get input format flag or format detection is not supported";
         return false;
      }
      this->inputFlags |= bmdVideoInputEnableFormatDetection;
   }

   // Get the first display mode
   if (deckLinkInput->GetDisplayModeIterator(&displayModeIterator) != S_OK)
   {
      Shutdown();
      errorString = "Failed to get display mode iterator";
      return false;
   }

   if (displayModeIterator->Next(&displayMode) != S_OK)
   {
      Shutdown();
      errorString = "Failed to get display mode";
      return false;
   }

   // Get the display mode name
   if (displayMode->GetName((const char**)&displayModeName) != S_OK)
   {
      displayModeName = (char*)malloc(32);
      snprintf(displayModeName, 32, "N/A");
   }

   // Check that the display mode is supported with given options
   if (deckLinkInput->DoesSupportVideoMode(displayMode->GetDisplayMode(),
                                          pixelFormat,
                                          bmdVideoInputFlagDefault,
                                          &displayModeSupport, NULL) != S_OK ||
       displayModeSupport == bmdDisplayModeNotSupported)
   {
      Shutdown();
      errorString = "Video mode not supported!";
      return false;
   }

   // Tell DeckLink to use 'this' as the callback delegate
   deckLinkInput->SetCallback(this);

   // Enable audio and video capture
   result = deckLinkInput->EnableVideoInput(displayMode->GetDisplayMode(), pixelFormat, this->inputFlags);
   result |= deckLinkInput->EnableAudioInput(bmdAudioSampleRate48kHz, audioSampleDepth, numAudioChannels);

   if (result != S_OK)
   {
      Shutdown();
      errorString = "Unable to enable video or audio input";
      return false;
   }

   // Start the capture process to begin receiving callbacks
   if (deckLinkInput->StartStreams() != S_OK)
   {
      Shutdown();
      errorString = "Unable to start streams";
      return false;
   }

   return true;
}

bool BlackMagicCapturer::Shutdown()
{
   if (displayModeName)
      free(displayModeName);

   if (deckLink)
      deckLink->Release();

   if (displayMode)
      displayMode->Release();

   if (displayModeIterator)
      displayModeIterator->Release();

   if(deckLinkInput)
   {
      deckLinkInput->StopStreams();
      deckLinkInput->DisableAudioInput();
      deckLinkInput->DisableVideoInput();
      deckLinkInput->Release();
      deckLinkInput= NULL;
   }

   if (deckLinkAttributes)
      deckLinkAttributes->Release();

   if (deckLink)
      deckLink->Release();

   if(deckLinkIterator)
      deckLinkIterator->Release();

   return true;
}

bool BlackMagicCapturer::HasFrame()
{
   return !frameQueue.empty();
}

bool BlackMagicCapturer::GetNextFrame(Frame& nextFrame)
{
   return frameQueue.pop(nextFrame);
}

ULONG BlackMagicCapturer::AddRef()
{
   pthread_mutex_lock(&mutex);
   ++referenceCount;
   pthread_mutex_unlock(&mutex);
}

ULONG BlackMagicCapturer::Release()
{
   pthread_mutex_lock(&mutex);
   --referenceCount;
   pthread_mutex_unlock(&mutex);

   if (!referenceCount)
   {
      delete this;
      return 0;
   }

   return (ULONG)referenceCount;
}

HRESULT STDMETHODCALLTYPE BlackMagicCapturer::VideoInputFormatChanged(
                                    BMDVideoInputFormatChangedEvents events,
                                    IDeckLinkDisplayMode* displayMode,
                                    BMDDetectedVideoInputFormatFlags flags)
{
   // This only gets called if bmdVideoInputEnableFormatDetection was set
   // when enabling video input
   HRESULT result;

   if (!(events & bmdVideoInputDisplayModeChanged))
      return S_OK;

   if (displayModeName)
   {
      free(displayModeName);
   }

   displayMode->GetName((const char**)&displayModeName);
   
   if (deckLinkInput)
   {
      deckLinkInput->StopStreams();
      result = deckLinkInput->EnableVideoInput(displayMode->GetDisplayMode(), pixelFormat, inputFlags);
      if (result == S_OK)
      {
         deckLinkInput->StartStreams();
      }
   }

   return S_OK;
}

HRESULT STDMETHODCALLTYPE BlackMagicCapturer::VideoInputFrameArrived(
                                    IDeckLinkVideoInputFrame* videoFrame,
                                    IDeckLinkAudioInputPacket* audioPacket)
{
   std::string error = "";
   void* videoFrameBytes = NULL;
   void* audioFrameBytes = NULL;

   // Get the video frame data
   if (videoFrame)
   {
      if (videoFrame->GetFlags() & bmdFrameHasNoInputSource)
      {
         error = "Frame received. No input signal detected.";
      }
      else
      {
         videoFrame->GetBytes(&videoFrameBytes);
      }
   }

   // Get the audio frame data
   if (audioPacket)
   {
      audioPacket->GetBytes(&audioFrameBytes);
   }

   // Store the frame
   if (videoFrameBytes || audioFrameBytes || !error.empty())
   {
      Frame frame;
      frame.videoBytes = (uint8_t*)videoFrameBytes;
      frame.audioBytes = (uint8_t*)audioFrameBytes;
      frame.videoStride = videoFrame->GetRowBytes();
      frame.videoHeight = videoFrame->GetHeight();
      frame.numAudioBytes = audioPacket->GetSampleFrameCount()*this->numAudioChannels*(audioSampleDepth/8);
      frame.error = error;

      frameQueue.push(frame);
   }
   
   return S_OK;
}
