#!/usr/bin/env python

import os
import csv
import sys
import pdb
import glob
import math
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from CsvFileIO import GetCsvData
from CsvFileIO import IsNumeric

def PlotHist(header, data, title='Histogram', num_bins=10, save_path=''):
   if len(header) > 0:
      # Remove the first column if it holds a time stamp
      if header[0].lower().find('time') != -1:
         header = header[1:]
         data = data[:,1:]

      if data.dtype.str[1:].startswith('S'):
         print 'Histogram plots for string data types not supported'
         return

      for i in range(len(header)):
         data_name = header[i]
         num_rows = math.ceil(math.sqrt(len(header)))
         plt.subplot(num_rows, int(math.ceil(len(header)/num_rows)), i+1)
         norm_weights = np.ones_like(data[:,i]).astype(float)/data.shape[0]
         (n, bins, patches) = plt.hist(data[:,i], num_bins, color='green', weights=norm_weights)
         plt.xlabel(data_name)
         plt.ylabel('Probability')
         if i == 0:
            plt.title(title)

      if save_path:
         plt.savefig(save_path)
      else:
         plt.show()
      plt.clf()

def DoGenerateHistograms(subfolder, save_path):
   if not save_path.endswith('/'):
      save_path = save_path+'/'
   save_path = save_path+subfolder+'/'
   if not os.path.isdir(save_path):
      os.mkdir(save_path)

   subfolder_path = os.path.dirname(os.path.realpath(__file__))+'/../data/'+subfolder+'/'
   # Normal case: plot each file separately
   if subfolder.find('annotation') == -1:
      for data_file in os.listdir(subfolder_path):
         if data_file.endswith('.csv'):
            (header, data) = GetCsvData(subfolder_path+data_file)
         else:
            print 'Unknown file extension for file %s.  FIX ME!!'%(data_file)

         save_file_path = save_path+data_file[:-4]+'.png'
         PlotHist(header, data, title='Histogram from %s'%(data_file), save_path=save_file_path)
   else: # Special case: aggregate annotations from the same subject and task
      plot_groups = {}
      for data_file in os.listdir(subfolder_path):
         if data_file.endswith('.csv'):
            data_file_split = data_file.split('_')
            subject_id = data_file_split[0]
            task = data_file_split[3]
            stream_name = data_file_split[-1][:-4]
            plot_group_str = 'Subject %s, Task: %s, Stream: %s'%(subject_id, task, stream_name)
            if plot_group_str in plot_groups.keys():
               plot_groups[plot_group_str].append(subfolder_path+data_file)
            else:
               plot_groups[plot_group_str] = [subfolder_path+data_file]
         else:
            print 'Unknown file extension for file %s.  FIX ME!!'%(data_file)

      for plot_group_name in plot_groups.keys():
         group_data = np.array([])
         for file_path in plot_groups[plot_group_name]:
            (header, data) = GetCsvData(file_path)
            if len(group_data) == 0:
               group_data = np.array(data)
            else:
               group_data = np.vstack((group_data, data))
         save_file_path = save_path+os.path.basename(file_path)[:-4]+'.png'
         PlotHist(header, group_data, title=plot_group_name, save_path=save_file_path)

if __name__ == '__main__':
   if len(sys.argv) <= 2:
      print 'Please provide two arguments: \n1)Subfolder name, like "annotation" for example\n2) Save path for plots'
   else:
      DoGenerateHistograms(sys.argv[1], sys.argv[2])
