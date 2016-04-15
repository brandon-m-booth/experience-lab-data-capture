#include <zmq.h>
#include <Windows.h>
#include <iostream>
#include <cstdlib>
#include <errno.h>
#include <boost/lockfree/spsc_queue.hpp>
#include "appActivityEvent.h"

static const int AppActivityQueueSize = 1000;
boost::lockfree::spsc_queue<AppActivity::AppActivityEvent, boost::lockfree::capacity<AppActivityQueueSize>> appActivityEventQueue;

BOOL QueryWindowFullProcessImageName(HWND hwnd, DWORD dwFlags, PTSTR lpExeName, DWORD dwSize)
{
	DWORD pid = 0;
	BOOL fRc = FALSE;
	if (GetWindowThreadProcessId(hwnd, &pid))
	{
		HANDLE hProcess = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid);
		if (hProcess)
		{
			fRc = QueryFullProcessImageName(hProcess, dwFlags, lpExeName, &dwSize);
			CloseHandle(hProcess);
		}
	}
	return fRc;
}

VOID CALLBACK WinEventProcCallback(HWINEVENTHOOK hWinEventHook, DWORD dwEvent, HWND hwnd, LONG idObject, LONG idChild, DWORD dwEventThread, DWORD dwmsEventTime)
{
	AppActivity::AppActivityEvent appActivityEvent;
	appActivityEvent.appActivityEventType = AppActivity::ActiveApplication;

	switch(dwEvent)
	{
	case EVENT_SYSTEM_FOREGROUND:
	{
		if (hwnd)
		{
			//if (!QueryWindowFullProcessImageName(hwnd, 0, appActivityEvent.dataItem, AppActivity::AppActivityEvent::MAX_DATA_BYTES))
			if (!GetWindowText(hwnd, appActivityEvent.dataItem, AppActivity::AppActivityEvent::MAX_DATA_BYTES))
			{
				strncpy(appActivityEvent.dataItem, "<unknown>", AppActivity::AppActivityEvent::MAX_DATA_BYTES);
			}
		}
		else
		{
			strncpy(appActivityEvent.dataItem, "<none>", AppActivity::AppActivityEvent::MAX_DATA_BYTES);
		}
		break;
	}
	default:
		break;
	}

	appActivityEventQueue.push(appActivityEvent);
}

int main(int argc, char** argv)
{
	/*
	// Handle input parameters
	bool doPrintHelp = false;
	if (argc == 2)
	{
		if (!strcmp(argv[1], "-s"))
		{
			doSafeKeyboardMode = true;
		}
		else if (!strcmp(argv[1], "-a"))
		{
			doSafeKeyboardMode = false;
		}
		else
		{
			doPrintHelp = true;
		}
	}
	if (doPrintHelp || argc > 2)
	{
		std::cout << "PlayerApplicationSniffer [args]" << std::endl;
		std::cout << "args:" << std::endl;
		std::cout << "\t--help\tPrint this help menu" << std::endl;
		std::cout << "\t-s\tRun in safe keyboard mode (default) where key codes are not sniffed" << std::endl;
		std::cout << "\t-a\tRun in normal mode" << std::endl;
		return EXIT_SUCCESS;
	}
	*/

	// Install the event hooks
	HWINEVENTHOOK hEvent = SetWinEventHook(EVENT_SYSTEM_FOREGROUND, EVENT_SYSTEM_FOREGROUND, NULL, WinEventProcCallback, 0, 0, WINEVENT_OUTOFCONTEXT | WINEVENT_SKIPOWNPROCESS);

	//  Prepare our context and socket
	const char* serverAddressAndPort = "tcp://sal103xeon05:37482";
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
		if (!appActivityEventQueue.empty())
		{
			AppActivity::AppActivityEvent appActivityEvent;
			while (appActivityEventQueue.pop(appActivityEvent))
			{
				int error = zmq_send(zmqSocket, &appActivityEvent, sizeof(appActivityEvent), 0);
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
    UnhookWinEvent(hEvent);

    return EXIT_SUCCESS;
}

__int32 GetKey(DWORD keyCode)
{
	__int32 key = (__int32)keyCode;

	return key;
}
