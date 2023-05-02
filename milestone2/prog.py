import serial
import csv
import time

ser1 = Serial("/dev/ttyAMA1", 115200)

with open('milestone2/milestone2.csv') as csv_file:  # Open the csv file. Will close the csv file when done running
    reader = csv.reader(csv_file, delimiter=',')  #  Creates a reader object that holds a list of lists of all of the data
    for row in reader:  # Iterate through each row in the entire reader file
        command_str = ' '.join(row) + '\n'
        print(command_str.encode())
        ser1.write(command_str.encode())  # Write the command string to the Pi in terms of bytes, because write() requires bytes to send info
        time.sleep(int(row[2]) + 1) # time.sleep(5)

ser1.close()
