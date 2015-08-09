#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys,os
import time
import ctypes
import ntpath
from ctypes import *
import rospy
from player_eeg.msg import EEGData

#--------------------------------------------------------------------------

srcDir = os.path.realpath(__file__)
headPath, tailPath = ntpath.split(srcDir)
srcDir = headPath or ntpath.basename(headPath)
libPath = srcDir + "/../third_party/emotiv_epoc/lib/libedk.so.1.0.0"
libEDK = CDLL(libPath)

#--------------------------------------------------------------------------

ED_COUNTER = 0
ED_INTERPOLATED=1
ED_RAW_CQ=2
ED_AF3=3
ED_F7=4
ED_F3=5
ED_FC5=6
ED_T7=7
ED_P7=8
ED_O1=9
ED_O2=10
ED_P8=11
ED_T8=12
ED_FC6=13
ED_F4=14
ED_F8=15
ED_AF4=16
ED_GYROX=17
ED_GYROY=18
ED_TIMESTAMP=19
ED_ES_TIMESTAMP=20
ED_FUNC_ID=21
ED_FUNC_VALUE=22
ED_MARKER=23
ED_SYNC_SIGNAL=24

targetChannelList = [ED_COUNTER,ED_AF3, ED_F7, ED_F3, ED_FC5, ED_T7,ED_P7, ED_O1, ED_O2, ED_P8, ED_T8,ED_FC6, ED_F4, ED_F8, ED_AF4, ED_GYROX, ED_GYROY, ED_TIMESTAMP, ED_FUNC_ID, ED_FUNC_VALUE, ED_MARKER, ED_SYNC_SIGNAL]

header = ['COUNTER','AF3','F7','F3', 'FC5', 'T7', 'P7', 'O1', 'O2','P8', 'T8', 'FC6', 'F4','F8', 'AF4','GYROX', 'GYROY', 'TIMESTAMP','FUNC_ID', 'FUNC_VALUE', 'MARKER', 'SYNC_SIGNAL']

nSamples   = c_uint(0)
nSam       = c_uint(0)
nSamplesTaken  = pointer(nSamples)
secs      = c_float(1)

#---------------------------------------------------------------------------------------------------------------------------------------------------------------

write = sys.stdout.write
EE_EmoEngineEventCreate = libEDK.EE_EmoEngineEventCreate
EE_EmoEngineEventCreate.restype = c_void_p
eEvent = EE_EmoEngineEventCreate()

EE_EmoEngineEventGetEmoState = libEDK.EE_EmoEngineEventGetEmoState
EE_EmoEngineEventGetEmoState.argtypes=[c_void_p,c_void_p]
EE_EmoEngineEventGetEmoState.restype = c_int

ES_GetTimeFromStart = libEDK.ES_GetTimeFromStart
ES_GetTimeFromStart.argtypes=[ctypes.c_void_p]
ES_GetTimeFromStart.restype = c_float

EE_EmoStateCreate = libEDK.EE_EmoStateCreate
EE_EmoStateCreate.restype = c_void_p
eState=EE_EmoStateCreate()

ES_GetWirelessSignalStatus=libEDK.ES_GetWirelessSignalStatus
ES_GetWirelessSignalStatus.restype = c_int
ES_GetWirelessSignalStatus.argtypes = [c_void_p]

ES_ExpressivIsBlink=libEDK.ES_ExpressivIsBlink
ES_ExpressivIsBlink.restype = c_int
ES_ExpressivIsBlink.argtypes= [c_void_p]

ES_ExpressivIsLeftWink=libEDK.ES_ExpressivIsLeftWink
ES_ExpressivIsLeftWink.restype = c_int
ES_ExpressivIsLeftWink.argtypes= [c_void_p]

ES_ExpressivIsRightWink=libEDK.ES_ExpressivIsRightWink
ES_ExpressivIsRightWink.restype = c_int
ES_ExpressivIsRightWink.argtypes= [c_void_p]

ES_ExpressivIsLookingLeft=libEDK.ES_ExpressivIsLookingLeft
ES_ExpressivIsLookingLeft.restype = c_int
ES_ExpressivIsLookingLeft.argtypes= [c_void_p]

ES_ExpressivIsLookingRight=libEDK.ES_ExpressivIsLookingRight
ES_ExpressivIsLookingRight.restype = c_int
ES_ExpressivIsLookingRight.argtypes= [c_void_p]

ES_ExpressivGetUpperFaceAction=libEDK.ES_ExpressivGetUpperFaceAction
ES_ExpressivGetUpperFaceAction.restype = c_int
ES_ExpressivGetUpperFaceAction.argtypes= [c_void_p]

ES_ExpressivGetUpperFaceActionPower=libEDK.ES_ExpressivGetUpperFaceActionPower
ES_ExpressivGetUpperFaceActionPower.restype = c_float
ES_ExpressivGetUpperFaceActionPower.argtypes= [c_void_p]

ES_ExpressivGetLowerFaceAction=libEDK.ES_ExpressivGetLowerFaceAction
ES_ExpressivGetLowerFaceAction.restype = c_int
ES_ExpressivGetLowerFaceAction.argtypes= [c_void_p]

ES_ExpressivGetLowerFaceActionPower=libEDK.ES_ExpressivGetLowerFaceActionPower
ES_ExpressivGetLowerFaceActionPower.restype = c_float
ES_ExpressivGetLowerFaceActionPower.argtypes= [c_void_p]

ES_AffectivGetExcitementShortTermScore=libEDK.ES_AffectivGetExcitementShortTermScore
ES_AffectivGetExcitementShortTermScore.restype = c_float
ES_AffectivGetExcitementShortTermScore.argtypes= [c_void_p]

ES_AffectivGetExcitementLongTermScore=libEDK.ES_AffectivGetExcitementLongTermScore
ES_AffectivGetExcitementLongTermScore.restype = c_float
ES_AffectivGetExcitementLongTermScore.argtypes= [c_void_p]


ES_AffectivGetEngagementBoredomScore=libEDK.ES_AffectivGetEngagementBoredomScore
ES_AffectivGetEngagementBoredomScore.restype = c_float
ES_AffectivGetEngagementBoredomScore.argtypes= [c_void_p]

ES_CognitivGetCurrentAction=libEDK.ES_CognitivGetCurrentAction
ES_CognitivGetCurrentAction.restype = c_int
ES_CognitivGetCurrentAction.argtypes= [c_void_p]

ES_CognitivGetCurrentActionPower=libEDK.ES_CognitivGetCurrentActionPower
ES_CognitivGetCurrentActionPower.restype = c_float
ES_CognitivGetCurrentActionPower.argtypes= [c_void_p]
#-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
userID = c_uint(0)
user = pointer(userID)
composerPort = c_uint(1726)
timestamp = c_float(0.0)
option = c_int(0)
state =c_int(0)

#-----------------------------------------------------------------------------------------------------------------------------------------------------------

option = 1
if option == 1:
   if libEDK.EE_EngineConnect("Emotiv Systems-5") != 0:
       print "Emotiv Engine start up failed."
elif option == 2:
    if libEDK.EE_EngineRemoteConnect("127.0.0.1", composerPort) != 0:
       print "Cannot connect to EmoComposer on"
else :
    print "option = ?"
print "Start receiving EEG Data! Press any key to stop logging...\n"
#f = file('ES.csv', 'w')
#f = open('ES.csv', 'w')
#print >> f,header

hData = libEDK.EE_DataCreate()
libEDK.EE_DataSetBufferSizeInSec(secs)

fake_eeg_pub = rospy.Publisher('player_eeg', EEGData, queue_size=1)
rospy.init_node('player_eeg', anonymous=True)	
rate = rospy.Rate(15)
while not rospy.is_shutdown():
    state = libEDK.EE_EngineGetNextEvent(eEvent)
    #print "state =",state
    if state == 0:
        eventType = libEDK.EE_EmoEngineEventGetType(eEvent)
        libEDK.EE_EmoEngineEventGetUserId(eEvent, user)
        libEDK.EE_DataAcquisitionEnable(userID,True)
        #print eventType,userID
        if eventType == 64: #libEDK.EE_Event_enum.EE_EmoStateUpdated
            libEDK.EE_DataUpdateHandle(0, hData)
            libEDK.EE_DataGetNumberOfSample(hData,nSamplesTaken)
        
            
            if nSamplesTaken[0] != 0:
                nSam=nSamplesTaken[0]
                arr=(ctypes.c_double*nSamplesTaken[0])()
                ctypes.cast(arr, ctypes.POINTER(ctypes.c_double))
                
                for sampleIdx in range(nSamplesTaken[0]):
                    eegData = EEGData()
                    for i in range(22): 
                        libEDK.EE_DataGet(hData,targetChannelList[i],byref(arr), nSam)
                        # print "SampleIdx: ", sampleIdx
                        # print "Value of above:",arr[sampleIdx]
                        # print >>f,arr[sampleIdx],",",
                        if i == 1 :
                            eegData.af3 = arr[sampleIdx]
                        elif i == 2 :
                            eegData.f7 = arr[sampleIdx]
                        elif i == 3 :
                            eegData.f3 = arr[sampleIdx]
                        elif i == 4 :
                            eegData.fc5 = arr[sampleIdx]
                        elif i == 5 :
                            eegData.t7 = arr[sampleIdx]
                        elif i == 6 :
                            eegData.p7 = arr[sampleIdx]
                        elif i == 7 :
                            eegData.o1 = arr[sampleIdx]
                        elif i == 8 :
                            eegData.o2 = arr[sampleIdx]
                        elif i == 9 :
                            eegData.p8 = arr[sampleIdx]
                        elif i == 10 :
                            eegData.t8 = arr[sampleIdx]
                        elif i == 11 :
                            eegData.fc6 = arr[sampleIdx]
                        elif i == 12 :
                            eegData.f4 = arr[sampleIdx]
                        elif i == 13 :
                            eegData.f8 = arr[sampleIdx]
                        elif i == 14 :
                            eegData.af4 = arr[sampleIdx]
                        elif i == 15 :
                            eegData.gyrox = arr[sampleIdx]
                        elif i == 16 :
                            eegData.gyroy = arr[sampleIdx]
                        elif i == 17 :
                            eegData.timestamp = arr[sampleIdx]

                    #print >>f,'\n'
                    fake_eeg_pub.publish(eegData)
                    #print "Publishing EEG_DATA"
                    
            #print >>f,"DONE",'\n'
    rate.sleep()

#-------------------------------------------------------------------------------------------------------------------------------------------------------------
libEDK.EE_DataFree(hData)
libEDK.EE_EngineDisconnect()
libEDK.EE_EmoStateFree(eState)
libEDK.EE_EmoEngineEventFree(eEvent)

