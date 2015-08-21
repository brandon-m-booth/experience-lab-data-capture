# Usage:
# 1) rostopic echo /player_eeg_spectral > eeg_spectral.txt
# 2) python eeg_spectral_to_csv.py -i eeg_spectral.txt -o output.csv

import csv
import sys
import getopt

def doConvert(eeg_spectral_file, output_csv):
   infile = open(eeg_spectral_file, 'r')
   outfile = open(output_csv, 'wb')
   outfileWriter = csv.writer(outfile)
   datum = []
   outputHeader = []
   for line in infile:
      strippedLine = line.rstrip();
      if strippedLine == "---":
         splitLine = datum[0].split(':', 1)
         header = splitLine[0]
         data = splitLine[-1]
         if header == "freqs":
            data = data.split('[', 1)[-1]
            data = data.rstrip(']');
            freqs = data.split(', ')

         headers = []
         outLines = []
         for sensorData in datum[1:-3]:
            splitLine = sensorData.split(':', 1)
            header = splitLine[0]
            data = splitLine[-1]
            data = data.split('[', 1)[-1]
            data = data.rstrip(']')

            if len(outputHeader) == 0:
               freqCounter = 0
               for freq in freqs:
                  headers.append(header+"_"+str(freqCounter)+"Hz")
                  freqCounter = freqCounter + 1
                  #headers.append(header+"_"+str(int(round(float(freq))))+"Hz")

            outLines.extend(data.split(', '))

         headers.append("timestamp")
         seconds = float(datum[-2].split(':', 1)[-1])
         nanoseconds = float(datum[-1].split(':', 1)[-1])
         seconds = seconds + nanoseconds/1000000000
         outLines.append(seconds)
         
         if len(outputHeader) == 0:
            outputHeader = headers
            outfileWriter.writerow(outputHeader)

         outfileWriter.writerow(outLines)
         datum = []
      else:
         datum.append(strippedLine)

if __name__ == "__main__":
   inputfile = ''
   outputfile = ''
   try:
      opts, args = getopt.getopt(sys.argv[1:],"hi:o:",["ifile=","ofile="])
   except getopt.GetoptError:
      print 'eeg_spectral_to_csv.py -i <inputfile> -o <outputfile>'
      sys.exit(2)
   for opt, arg in opts:
      if opt == '-h':
         print 'eeg_spectral_to_csv.py -i <inputfile> -o <outputfile>'
         sys.exit()
      elif opt in ("-i", "--ifile"):
         inputfile = arg
      elif opt in ("-o", "--ofile"):
         outputfile = arg

   doConvert(inputfile, outputfile)
