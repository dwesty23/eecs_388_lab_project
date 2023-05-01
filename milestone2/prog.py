import serial
import csv
import time


with open('milestone2/milestone2.csv') as csv_file:  # Open the csv file. Will close the csv file when done running
    with serial.Serial('/dev/ttyAMA1' , 115200 , timeout = 1) as ser1:  # Creates a serial chanel over /dev/ttyAMA1 with a baudrate of 115200. Sets it to ser1 so it's easy to call functions on
        reader = csv.reader(csv_file, delimiter=',')  #  Creates a reader object that holds a list of lists of all of the data
        for row in reader:  # Iterate through each row in the entire reader file
            angle = str(row[0])  # Assign the angle to the first item
            speed = str(row[1])  # Assign the speed to the second item
            duration = str(row[2])  # Assign the duration to the third item
            command_str = "angle: " + angle + ", speed: " + speed + ", duration: " + duration  # Set the command string to be sent to the Pi
            print(command_str.encode())
            ser1.write(command_str.encode())  # Write the command string to the Pi in terms of bytes, because write() requires bytes to send info
            time.sleep(int(row[2]) + 1) # time.sleep(5)
