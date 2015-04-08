#pragma once

#include "DeckLinkAPI.h"

#include <vector>
#include <string>
#include <boost/lockfree/spsc_queue.hpp>

extern class pthread_mutex_t;

static const unsigned int FrameQueueSize = 5;

struct Frame
{
	uint8_t* videoBytes;
	uint8_t* audioBytes;
	long stride;
	long height;
	std::string error;
};

class BlackMagicCapturer : public IDeckLinkInputCallback
{
public:
   BlackMagicCapturer();
   ~BlackMagicCapturer();
	
	bool Initialize();
	bool Shutdown();

   virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID* ppv) { return E_NOINTERFACE; }
   virtual ULONG STDMETHODCALLTYPE AddRef();
   virtual ULONG STDMETHODCALLTYPE Release();
   virtual HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(
                                       BMDVideoInputFormatChangedEvents events,
                                       IDeckLInkDisplayMode* displayMode,
                                       BMDDetectedvideoInputFormatFlags flags);
   virtual HRESULT STDMETHODCALLTYPE VideoInputFrameArrived(
                                       IDeckLinkVideoInputFrame* frame,
                                       IDeckLinkaudioInputPacket* packet);

protected:
   ULONG referenceCount;
	IDeckLinkIterator* deckLinkIterator;
	IDeckLink* deckLink;
	BMDPixelFormat pixelFormat;
	BMDVideoInputFlags inputFlags;
	BMDTimecodeFormat timecodeFormat;
	const char* displayModeName;
   pthread_mutex_t mutex;

	boost::lockfree::spsc_queue<Frame, boost::lockfree:capacity<FrameQueueSize>> frameQueue;

	static IDeckLinkInput* deckLinkInput;
};
