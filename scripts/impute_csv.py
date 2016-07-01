#!/usr/bin/env python

import os
import sys
import glob
import math
import numpy as np
import csv
import pdb
from CsvFileIO import GetCsvData
from CsvFileIO import SaveCsvData
from CsvFileIO import IsNumeric

def CollapseNonUniqueTimeStamps(data):
   times = data[:,0].astype(float)
   unique_times = np.unique(times)
   collapsed_data = np.empty((len(unique_times), data.shape[1]), dtype=data.dtype)
   is_numeric = IsNumeric(data[0,:])
   for i in range(len(unique_times)):
      unique_time = unique_times[i]
      overlapping_indices = np.where(data[:,0].astype(float) == unique_time)
      if is_numeric:
         collapsed_data[i,:] = np.average(data[overlapping_indices,:][0], axis=0)
      else:
         for col_index in range(data.shape[1]):
            (unique_vals, unique_counts) = np.unique(data[overlapping_indices,col_index], return_counts=True)
            highest_count_index = np.argmax(unique_counts)
            collapsed_data[i,col_index] = unique_vals[highest_count_index]
   return collapsed_data

def ComputeWindowAverage(data, reference_time):
   is_numeric = IsNumeric(data[0,:])
   if data.shape[0] > 1:
      ref_time_dist = np.fabs(data[:,0].astype(float)-reference_time)
      weights = sum(ref_time_dist)*np.ones_like(ref_time_dist) - ref_time_dist
      weights = weights/(len(weights)-1)
   else:
      weights = None
   if is_numeric:
      data_average = np.average(data[:,1:], axis=0, weights=weights)
   else:
      data_average = []
      for col_index in range(1,data.shape[1]):
         (unique_vals, unique_counts) = np.unique(data[:,col_index], return_counts=True)
         highest_count_index = np.argmax(unique_counts)
         data_average.append(unique_vals[highest_count_index])
   return np.hstack(([reference_time],data_average))

# Assumes the first column of data is a time stamp field
def ResampleStepwise(data, t0, dt):
   resampled_rows = int(math.ceil((data[-1,0].astype(float)-t0)/dt))
   resampled_data = np.empty((resampled_rows, data.shape[1]), dtype=data.dtype)
   t = float(t0)
   last_data = data[0,1:]
   data_index = 0
   for resampled_index in range(resampled_rows):
      row_time = data[data_index,0].astype(float)
      if t < (row_time-dt/2.0):
         resampled_data[resampled_index,:] = np.hstack(([t],last_data))
      else:
         start_data_index = data_index
         data_index = data_index + 1
         while data_index < data.shape[0] and data[data_index,0] < t+(dt/2.0):
            data_index = data_index + 1
         window_data = data[start_data_index:data_index,:]
         window_data = CollapseNonUniqueTimeStamps(window_data)
         window_average = ComputeWindowAverage(window_data, t)
         resampled_data[resampled_index,:] = window_average
         data_index = min(data.shape[0]-1, data_index)
         last_data = window_average[1:]
      t = t + dt
   return resampled_data
      
def DoResampleCsv(in_folder_path, dt, out_folder_path):
   if not in_folder_path.endswith('/'):
      in_folder_path = in_folder_path+'/'
   if not out_folder_path.endswith('/'):
      out_folder_path = out_folder_path+'/'
   if not os.path.isdir(out_folder_path):
      os.mkdir(out_folder_path)

   t0 = 0.0
   for data_file in os.listdir(in_folder_path):
      print 'Processing file: %s'%(data_file)
      if data_file.endswith('.csv'):
         (header, data) = GetCsvData(in_folder_path+data_file)
         resampled_data = ResampleStepwise(data, t0, dt)
         out_file_name = os.path.basename(data_file)
         SaveCsvData(header, resampled_data, out_folder_path+out_file_name)

   return

if __name__=='__main__':
   if len(sys.argv) <= 2:
      print 'Please provide these command line arguments:\n1) Folder path containing files to resample\n2) Resampling rate in Hertz\n3) Output path for resampled data'
   else:
      DoResampleCsv(sys.argv[1], 1.0/float(sys.argv[2]), sys.argv[3])

