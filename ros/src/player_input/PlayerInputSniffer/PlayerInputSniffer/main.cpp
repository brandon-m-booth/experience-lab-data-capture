#include <zmq.h>
#include <Windows.h>
#include <iostream>
#include <cstdlib>
#include <errno.h>
#include <boost/lockfree/spsc_queue.hpp>
#include "inputEvent.h"

static const int KeysQueueSize = 1000;
boost::lockfree::spsc_queue<Input::InputEvent, boost::lockfree::capacity<KeysQueueSize>> inputEventQueue;

__int32 GetKey(DWORD keyCode);

LRESULT CALLBACK LowLevelKeyboardProc(int nCode, WPARAM wParam, LPARAM lParam)
{
    if (nCode == HC_ACTION)
    {
		Input::InputEvent inputEvent;
		inputEvent.inputEventType = Input::KeyState;

        switch (wParam)
        {
        case WM_KEYDOWN:
        case WM_SYSKEYDOWN:
			inputEvent.inputDataItem2 = Input::DownState;
			break;
        case WM_KEYUP:
        case WM_SYSKEYUP:
			inputEvent.inputDataItem2 = Input::UpState;
            break;
		default:
			OutputDebugString("Unknown keyboard event type. FIX ME!");
			break;
        }

		PKBDLLHOOKSTRUCT keyboardHookStruct = (PKBDLLHOOKSTRUCT)lParam;
		inputEvent.inputDataItem1 = GetKey(keyboardHookStruct->vkCode);

		inputEventQueue.push(inputEvent);
    }
    return(CallNextHookEx(NULL, nCode, wParam, lParam));
}

LRESULT CALLBACK LowLevelMouseProc(int nCode, WPARAM wParam, LPARAM lParam)
{
	PMSLLHOOKSTRUCT mouseHookStruct = (PMSLLHOOKSTRUCT)lParam;

	Input::InputEvent inputEvent;
	inputEvent.inputEventType = Input::CursorPosition;
	switch(wParam)
	{
	case WM_LBUTTONDOWN:
		inputEvent.inputEventType = Input::KeyState;
		inputEvent.inputDataItem1 = Input::MouseLeft;
		inputEvent.inputDataItem2 = Input::DownState;
		break;
	case WM_LBUTTONUP:
		inputEvent.inputEventType = Input::KeyState;
		inputEvent.inputDataItem1 = Input::MouseLeft;
		inputEvent.inputDataItem2 = Input::UpState;
		break;
	case WM_MBUTTONDOWN:
		inputEvent.inputEventType = Input::KeyState;
		inputEvent.inputDataItem1 = Input::MouseWheel;
		inputEvent.inputDataItem2 = Input::DownState;
		break;
	case WM_MBUTTONUP:
		inputEvent.inputEventType = Input::KeyState;
		inputEvent.inputDataItem1 = Input::MouseWheel;
		inputEvent.inputDataItem2 = Input::UpState;
		break;
	case WM_MOUSEMOVE:
		inputEvent.inputEventType = Input::CursorPosition;
		inputEvent.inputDataItem1 = mouseHookStruct->pt.x;
		inputEvent.inputDataItem2 = mouseHookStruct->pt.y;
		break;
	case WM_MOUSEWHEEL:
		inputEvent.inputEventType = Input::MouseWheelScroll;
		inputEvent.inputDataItem1 = 0;
		inputEvent.inputDataItem2 = GET_WHEEL_DELTA_WPARAM(mouseHookStruct->mouseData)/WHEEL_DELTA;
		break;
	case WM_RBUTTONDOWN:
		inputEvent.inputEventType = Input::KeyState;
		inputEvent.inputDataItem1 = Input::MouseRight;
		inputEvent.inputDataItem2 = Input::DownState;
		break;
	case WM_RBUTTONUP:
		inputEvent.inputEventType = Input::KeyState;
		inputEvent.inputDataItem1 = Input::MouseRight;
		inputEvent.inputDataItem2 = Input::UpState;
		break;
	default:
		OutputDebugString("Unknown mouse event type. FIX ME!");
		break;
	}

	inputEventQueue.push(inputEvent);
	return(CallNextHookEx(NULL, nCode, wParam, lParam));
}

int main()
{
    // Install the low-level keyboard & mouse hooks
    HHOOK hhkLowLevelKybd = SetWindowsHookEx(WH_KEYBOARD_LL, LowLevelKeyboardProc, 0, 0);
	HHOOK hhkLowLevelMouse = SetWindowsHookEx(WH_MOUSE_LL, LowLevelMouseProc, 0, 0);

	//  Prepare our context and socket
	const char* serverAddressAndPort = "tcp://sal103xeon05:37481";
    void* zmqContext = zmq_ctx_new();
    void* zmqSocket = zmq_socket(zmqContext, ZMQ_PUB);

	// Connect the socket
    while (zmq_connect(zmqSocket, serverAddressAndPort) == EINVAL)
	{
		Sleep(10);
	}

	MSG msg;
	ZeroMemory(&msg, sizeof(msg));
	while (msg.message != WM_QUIT)
	{
		// Dispatch Windows messages
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}

		// Send queued key events to the target server
		if (!inputEventQueue.empty())
		{
			Input::InputEvent inputEvent;
			while (inputEventQueue.pop(inputEvent))
			{
				int error = zmq_send(zmqSocket, &inputEvent, sizeof(inputEvent), 0);
				if (error < 0)
				{
					char errorString[100];
					_snprintf_s(errorString, 100, "Error! Errno: %d\n", errno);
					OutputDebugString(errorString);
				}
			}
		}
		Sleep(10);
	}

	// Cleanup
	zmq_ctx_destroy(zmqContext);
    UnhookWindowsHookEx(hhkLowLevelKybd);
	UnhookWindowsHookEx(hhkLowLevelMouse);

    return(0);
}

__int32 GetKey(DWORD keyCode)
{
	__int32 key = (__int32)keyCode;

	return key;
}