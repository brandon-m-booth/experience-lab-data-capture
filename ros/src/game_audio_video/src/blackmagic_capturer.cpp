#include "blackmagic_capturer.h"
#include <boost/lockfree/spsc_queue.hpp>

IDeckLinkInput* BlackMagicCapturer::deckLinkInput = NULL;

BlackMagicCapturer::BlackMagicCapturer()
{
	pthread_mutex_init(&mutex, NULL);
	displayModeName = NULL;
	deckLinkIterator = NULL;
}

BlackMagicCapturer::~BlackMagicCapturer()
{
	pthread_mutex_destroy(&mutex);

	if (displayModeName)
	{
		free(displayModeName);
	}
}

bool BlackMagicCapturer::Initialize(BMDPixelFormat pixelFormat, BMDVideoInputFlags inputFlags)
{
	HRESULT result;

	deckLinkIterator = CreateDeckLinkIteratorInstance();
	if (!deckLinkIterator)
	{
		Shutdown();
		return false;
	}

	int deckLinkIndex = 0;
	while ((result = deckLinkIterator->Next(&deckLink)) == S_OK)
	{
		if (!index)
			break;

		--index;
		deckLink->Release();
	}

	if (result != S_OK || !deckLink)
	{
		Shutdown();
		return false;
	}

	// Get the input capture interface of the DeckLink device
	if (deckLink->QueryInterface(IID_IDeckLinkInput, (void**)&deckLinkInput) != S_OK)
	{
		Shutdown();
		return false;
	}

	if (deckLink->QueryInterface(IID_IDeckLinkAttributes, (void**)&deckLinkAttributes) == S_OK)
	{
	}

	this->pixelFormat = pixelFormat;
	this->inputFlags = inputFlags;
}

void Shutdown()
{
	if (displayModeName)
		free(displayModeName);

	if (deckLink)
		deckLink->Release();

	if(deckLinkIterator)
		deckLinkIterator->Release();
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

virtual HRESULT STDMETHODCALLTYPE BlackMagicCapturer::VideoInputFormatChanged(
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
		else
		{
			errors.push_back("Failed to switch video mode\n");
		}
	}

	return S_OK;
}

virtual HRESULT STDMETHODCALLTYPE BlackMagicCapturer::VideoInputFrameArrived(
												IDeckLinkVideoInputFrame* videoFrame,
												IDeckLinkAudioInputPacket* audioPacket)
{
	std::string error;
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
	if (videoFrameBytes || audioFrameBytes || !errors.empty())
	{
		Frame frame;
		frame.videoBytes = videoFrameBytes; // BB - WHAT IS THE LIFETIME?
		frame.audioBytes = audioFrameBytes;
		frame.stride = videoFrame->GetRowBytes();
		frame.height = videoFrame->GetHeight();
		frame.error = error;

		frameQueue.push(frame);
	}
	
	return S_OK;
}
