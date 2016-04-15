#ifdef _CONSOLE
typedef __int32 int32_t;
#endif

namespace AppActivity
{
	enum AppActivityEventType
	{
		ActiveApplication,
		//ActiveTab,
	};

	// TODO - use protocol buffers or something better than this...
	struct AppActivityEvent
	{
		static const unsigned int MAX_DATA_BYTES = 100;
		int32_t appActivityEventType; // Of type AppActivityEventType
		char dataItem[MAX_DATA_BYTES];
	};
}
