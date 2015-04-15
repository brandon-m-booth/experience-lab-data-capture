#pragma once

#include "DeckLinkAPI.h"

#include <vector>
#include <string>
#include <pthread.h>
#include <boost/lockfree/spsc_queue.hpp>

#define FrameQueueSize (5)

struct Frame
{
   uint8_t* videoBytes;
   uint8_t* audioBytes;
   long videoStride;
   long videoHeight;
   long numAudioBytes;
   std::string error;
};

class BlackMagicCapturer : public IDeckLinkInputCallback
{
public:
   static BlackMagicCapturer* GetInstance();

   bool Initialize(BMDPixelFormat pixelFormat,
                   BMDVideoInputFlags inputFlags,
                   int audioSampleDepth,
                   int numAudioChannels);
   bool Shutdown();

   std::string GetError() { return errorString; }

   bool HasFrame();
   bool GetNextFrame(Frame& nextFrame);

   virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID* ppv) { return E_NOINTERFACE; }
   virtual ULONG STDMETHODCALLTYPE AddRef();
   virtual ULONG STDMETHODCALLTYPE Release();
   virtual HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(
                                       BMDVideoInputFormatChangedEvents events,
                                       IDeckLinkDisplayMode* displayMode,
                                       BMDDetectedVideoInputFormatFlags flags);
   virtual HRESULT STDMETHODCALLTYPE VideoInputFrameArrived(
                                       IDeckLinkVideoInputFrame* frame,
                                       IDeckLinkAudioInputPacket* packet);

protected:
   BlackMagicCapturer();
   ~BlackMagicCapturer();

   std::string errorString;

   // Deck Link members
   ULONG referenceCount;
   IDeckLinkAttributes* deckLinkAttributes;
   IDeckLinkIterator* deckLinkIterator;
   IDeckLink* deckLink;
   pthread_mutex_t mutex;

   // Deck Link display members
   IDeckLinkDisplayModeIterator* displayModeIterator;
   IDeckLinkDisplayMode* displayMode;
   BMDDisplayModeSupport displayModeSupport;
   BMDPixelFormat pixelFormat;
   BMDVideoInputFlags inputFlags;
   BMDTimecodeFormat timecodeFormat;
   char* displayModeName;

   // Deck Link audio members
   int audioSampleDepth;
   int numAudioChannels;  

   boost::lockfree::spsc_queue<Frame, boost::lockfree::capacity<FrameQueueSize> > frameQueue;

   static IDeckLinkInput* deckLinkInput;
   static BlackMagicCapturer* instance;
};
