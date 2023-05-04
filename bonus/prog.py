import serial
import csv
import time

ser1 = Serial("/dev/ttyAMA1", 115200) # Open channel 1

with open('Bonus_ML_Data/Data1.csv') as csv_file1:  # Open the first csv file. Will close the csv file when done running
    with open('Bonus_ML_Data/Data2.csv') as csv_file2:  # Open the second csv file. Will close the csv file when done running
        reader1 = csv.reader(csv_file1, delimiter=',')  #  Creates a reader object for the first file that holds a list of lists of all of the data
        reader2 = csv.reader(csv_file2, delimiter=',')  #  Creates a reader object for the second file that holds a list of lists of all of the data
        for (row1, row2) in zip(reader1, reader2): # Iterates through the two reader objects at the same time 
            command_str = ' '.join(row1) + ' ' + ' '.join(row2) + '\n' # Joins each item in the line with a space and puts it into a command string
            ser1.write(command_str.encode())  # Write the command string to the Pi in terms of bytes, because write() requires bytes to send info
            time.sleep(int(row1[2]) + 1) 

ser1.close() # Closes channel 1