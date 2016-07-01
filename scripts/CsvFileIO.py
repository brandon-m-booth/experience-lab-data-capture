import csv
import pdb
import numpy as np

def IsNumeric(obj):
   if hasattr(obj, '__iter__'):
      ret_val = True
      for item in obj:
         try:
            float(item)
            ret_val = ret_val and True
         except ValueError:
            ret_val = False
   else:
      try:
         float(obj)
         ret_val = True
      except ValueError:
         ret_val = False
   return ret_val

def GetCsvData(file_path):
   with open(file_path, 'rb') as csvfile:
      csvreader = csv.reader(csvfile, delimiter=',')
      # Get row and column counts and data type
      row_count = sum(1 for row in csvreader)
      csvfile.seek(0) # Reset file iterator
      csvreader.next() # Skip header row
      first_data_row = csvreader.next()
      col_count = len(first_data_row)
      is_numeric = IsNumeric(first_data_row)
      if is_numeric:
         data = np.zeros((row_count-1, len(first_data_row)))
      else:
         data = np.empty((row_count-1, len(first_data_row)), dtype='S12')

      csvfile.seek(0) # Reset file iterator
      row_num = 0
      header = None
      for row in csvreader:
         if header is None:
            header = np.array(row)
         else:
            if is_numeric:
               data[row_num, :] = np.array(row).astype(float)
            else:
               data[row_num, :] = np.array(row)

            row_num = row_num + 1
               
   return (header, data)

def SaveCsvData(header, data, file_path):
   with open(file_path, 'wb') as csvfile:
      csvwriter = csv.writer(csvfile, delimiter=',')
      csvwriter.writerow(header)
      for row in data:
         csvwriter.writerow(row)
