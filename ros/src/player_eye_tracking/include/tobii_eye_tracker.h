#pragma once

#include "tobii_common.h"
#include <boost/lockfree/spsc_queue.hpp>

#define GazeDataQueueSize (10)

struct GazeData
{
   uint32_t leftEyeX;
   uint32_t leftEyeY;
   uint32_t rightEyeX;
   uint32_t rightEyeY;
};

class TobiiEyeTracker
{
public:
   static TobiiEyeTracker* GetInstance();

   bool Initialize();
   bool Shutdown();

   const std::string& GetSDKVersionString() { return sdkVersionString; }
   const std::string& GetDeviceSerialNumber() { return deviceSerialNumberString; }
   const std::string& GetErrorString() { return errorString; }  

   bool HasGazeData();
   bool GetGazeData(GazeData& gazeData);

protected:
   TobiiEyeTracker();
   ~TobiiEyeTracker();

   static xthread_retval EyeTrackerEventLoop(void* eyeTracker);
   static void GazeDataCallback(const tobiigaze_gaze_data* tobiiGazeData, const tobiigaze_gaze_data_extensions* extensions, void* userData);

   tobiigaze_eye_tracker* eyeTracker;
   xthread_handle threadHandle;

   std::string sdkVersionString;
   std::string deviceSerialNumberString;
   std::string errorString;

   boost::lockfree::spsc_queue<GazeData, boost::lockfree::capacity<GazeDataQueueSize> > gazeDataQueue;

   static TobiiEyeTracker* instance;
};
