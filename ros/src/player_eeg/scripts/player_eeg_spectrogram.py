#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys,os
import time
import ctypes
import ntpath
import copy
from ctypes import *
from scipy import signal
from collections import deque
import rospy
from player_eeg.msg import EEGData
from player_eeg.msg import EEGSpectralData

class EEGSpectrogram:
   """ A class to convert raw EEG data to the spectral domain """

   def __init__(self):
      rospy.init_node('player_eeg_spectrogram', anonymous=True)	

   def EEGDataCallback(self, data):

      self.samples_received = self.samples_received + 1
      cur_time = rospy.Time.now()
      # TODO - There is a big difference from the delta_time reported in the message
      # and the observed rate at which these messages are received - FIX ME!!!
      #delta_time = data.timestamp - self.last_timestamp
      #if delta_time > 0:
      #   self.sample_freq_average = (1/delta_time)*self.moving_average_coef + self.sample_freq_average*(1.0-self.moving_average_coef)
      #self.last_timestamp = data.timestamp
      self.sample_freq_average = self.samples_received/(cur_time - self.start_time).to_sec()

      self.samples.append(data)
      if len(self.samples) < self.target_sample_size:
         return
      else:
         af3 = []
         f7 = []
         f3 = []
         fc5 = []
         t7 = []
         p7 = []
         o1 = []
         o2 = []
         p8 = []
         t8 = []
         fc6 = []
         f4 = []
         f8 = []
         af4 = []
         for sample in reversed(self.samples):
            # Most recent samples should be at the front of the separate list
            af3.append(sample.af3)
            f7.append(sample.f7)
            f3.append(sample.f3)
            fc5.append(sample.fc5)
            t7.append(sample.t7)
            p7.append(sample.p7)
            o1.append(sample.o1)
            o2.append(sample.o2)
            p8.append(sample.p8)
            t8.append(sample.t8)
            fc6.append(sample.fc6)
            f4.append(sample.f4)
            f8.append(sample.f8)
            af4.append(sample.af4)

         spec_data = EEGSpectralData()
         spec_data.freqs, spec_data.af3 = signal.periodogram(af3, self.sample_freq_average, self.window)
         freqs, spec_data.f7 = signal.periodogram(f7, self.sample_freq_average, self.window)
         freqs, spec_data.f3 = signal.periodogram(f3, self.sample_freq_average, self.window)
         freqs, spec_data.fc5 = signal.periodogram(fc5, self.sample_freq_average, self.window)
         freqs, spec_data.t7 = signal.periodogram(t7, self.sample_freq_average, self.window)
         freqs, spec_data.p7 = signal.periodogram(p7, self.sample_freq_average, self.window)
         freqs, spec_data.o1 = signal.periodogram(o1, self.sample_freq_average, self.window)
         freqs, spec_data.o2 = signal.periodogram(o2, self.sample_freq_average, self.window)
         freqs, spec_data.p8 = signal.periodogram(p8, self.sample_freq_average, self.window)
         freqs, spec_data.t8 = signal.periodogram(t8, self.sample_freq_average, self.window)
         freqs, spec_data.fc6 = signal.periodogram(fc6, self.sample_freq_average, self.window)
         freqs, spec_data.f4 = signal.periodogram(f4, self.sample_freq_average, self.window)
         freqs, spec_data.f8 = signal.periodogram(f8, self.sample_freq_average, self.window)
         freqs, spec_data.af4 = signal.periodogram(af4, self.sample_freq_average, self.window)
         spec_data.timestamp = cur_time

         self.eeg_pub.publish(spec_data)

         # Clear out a percentage of the samples
         num_pop_samples = int(len(self.samples)*(1.0 - self.keep_samples_percentage))
         for i in range(num_pop_samples):
            self.samples.popleft()

   def DoConvertEEGToSpectrogram(self):
      self.eeg_sub = rospy.Subscriber('player_eeg', EEGData, self.EEGDataCallback)
      self.eeg_pub = rospy.Publisher('player_eeg_spectral', EEGSpectralData, queue_size=3)
      self.keep_samples_percentage = 0.8 # Percentage of previous samples to keep
      self.last_timestamp = rospy.Time.now()
      self.sample_freq_average = 0
      self.moving_average_coef = 0.01
      self.target_sample_size = 120
      self.window = signal.get_window('hann', self.target_sample_size)
      self.samples = deque()
      self.samples_received = 0
      self.start_time = rospy.Time.now()

      rospy.spin() # EEGDataCallback() handles it from here

if __name__ == '__main__':
   eegSpectrogram = EEGSpectrogram()
   eegSpectrogram.DoConvertEEGToSpectrogram()
