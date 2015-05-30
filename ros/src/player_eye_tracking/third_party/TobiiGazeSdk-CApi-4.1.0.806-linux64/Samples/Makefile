CC=g++
CFLAGS=-Wall -I../include -ICommon -L../lib -Wl,-rpath,'$$ORIGIN/../lib'
LDFLAGS=-ltobiigazecore -lpthread

all: tracker tracker_async calibration

tracker: MinimalTracker/MinimalTracker.c Common/Common.h
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

tracker_async: MinimalTrackerAsync/MinimalTrackerAsync.cpp Common/Common.h
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

calibration: MinimalCalibration/MinimalCalibration.cpp Common/Common.h
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)
