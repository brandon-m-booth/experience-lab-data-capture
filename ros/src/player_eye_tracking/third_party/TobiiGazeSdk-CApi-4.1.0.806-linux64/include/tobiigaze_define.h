//  Copyright 2013 Tobii Technology AB. All rights reserved.
#pragma once

// define TOBIIGAZE_CALL and TOBIIGAZE_API
#ifdef WIN32
    #define TOBIIGAZE_CALL __cdecl
    #ifdef TOBIIGAZE_STATIC_LIB
        #define TOBIIGAZE_API
    #else
        #ifdef TOBIIGAZE_EXPORTING
            #define TOBIIGAZE_API __declspec(dllexport)
        #else
            #define TOBIIGAZE_API __declspec(dllimport)
        #endif // TOBIIGAZE_EXPORTING
    #endif // TOBIIGAZE_STATIC_LIB
#else
    #define TOBIIGAZE_API
    #define TOBIIGAZE_CALL
#endif // WIN32

// define DEPRECATED macro
#ifdef __GNUC__
    #define DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
    #define DEPRECATED __declspec(deprecated)
#endif
