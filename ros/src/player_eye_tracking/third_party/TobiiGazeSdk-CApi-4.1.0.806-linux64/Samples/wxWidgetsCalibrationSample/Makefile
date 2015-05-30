# Makefile for the wxWidgetsCalibrationSample

WXCXXFLAGS=-DUNICODE -D_UNICODE `wx-config --cxxflags`
WXLIBS=`wx-config --libs`

CC=g++
CFLAGS=-c -Wall $(WXCXXFLAGS) -I../../include
LDFLAGS=-L../../lib -ltobiigazecore $(WXLIBS) -Wl,-rpath,'$$ORIGIN/../../lib'

SOURCES=CalibrationSampleApp.cpp CalibrationViewModel.cpp TestingViewModel.cpp \
CalibrationWindow.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=calibrationsample

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

