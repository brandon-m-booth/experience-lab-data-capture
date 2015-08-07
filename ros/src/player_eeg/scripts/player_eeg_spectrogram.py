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
         e1 = []
         e2 = []
         e3 = []
         e4 = []
         e5 = []
         e6 = []
         e7 = []
         e8 = []
         e9 = []
         e10 = []
         e11 = []
         e12 = []
         e13 = []
         e14 = []
         for sample in reversed(self.samples):
            # Most recent samples should be at the front of the separate list
            e1.append(sample.e1)
            e2.append(sample.e2)
            e3.append(sample.e3)
            e4.append(sample.e4)
            e5.append(sample.e5)
            e6.append(sample.e6)
            e7.append(sample.e7)
            e8.append(sample.e8)
            e9.append(sample.e9)
            e10.append(sample.e10)
            e11.append(sample.e11)
            e12.append(sample.e12)
            e13.append(sample.e13)
            e14.append(sample.e14)

         # Each wave bin here describes its frequency range and nfft window in that order
         #delta_wave = [range(0,5), ]
         #for freq_range, nfft in [[range(0, 5), ], [range(], [], [], []]:
         freqs, power_densities = signal.periodogram(e1, self.sample_freq_average, self.window)

         freq_message = EEGSpectralData()
         freq_message.freqs = [0]*(self.maximum_freq+1)
         freq_message.e1 = copy.deepcopy(freq_message.freqs)
         freq_message.e2 = copy.deepcopy(freq_message.freqs)
         freq_message.e3 = copy.deepcopy(freq_message.freqs)
         freq_message.e4 = copy.deepcopy(freq_message.freqs)
         freq_message.e5 = copy.deepcopy(freq_message.freqs)
         freq_message.e6 = copy.deepcopy(freq_message.freqs)
         freq_message.e7 = copy.deepcopy(freq_message.freqs)
         freq_message.e8 = copy.deepcopy(freq_message.freqs)
         freq_message.e9 = copy.deepcopy(freq_message.freqs)
         freq_message.e10 = copy.deepcopy(freq_message.freqs)
         freq_message.e11 = copy.deepcopy(freq_message.freqs)
         freq_message.e12 = copy.deepcopy(freq_message.freqs)
         freq_message.e13 = copy.deepcopy(freq_message.freqs)
         freq_message.e14 = copy.deepcopy(freq_message.freqs)
         for i in range(self.maximum_freq+1):
            freq_message.freqs[i] = freqs[i]
            freq_message.e1[i] = power_densities[i]
         freq_message.timestamp = rospy.Time.now()

         self.eeg_pub.publish(freq_message)

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
      self.maximum_freq = 60 # Maximum desired output frequency, Hertz
      self.target_sample_size = self.maximum_freq * 2 # Nyquist
      self.window = signal.get_window('hann', self.target_sample_size)
      self.samples = deque()
      self.samples_received = 0
      self.start_time = rospy.Time.now()

      rospy.spin() # EEGDataCallback() handles it from here

if __name__ == '__main__':
   eegSpectrogram = EEGSpectrogram()
   eegSpectrogram.DoConvertEEGToSpectrogram()
