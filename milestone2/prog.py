import serial
import csv

with open('milestone2.csv') as csv_file:  # Open the csv file. Will close the csv file when done running
    with serial.Serial('/dev/ttyAMA1' , 115200 , timeout = 1) as ser1:  # Creates a serial chanel over /dev/ttyAMA1 with a baudrate of 115200. Sets it to ser1 so it's easy to call functions on
        reader = csv.reader(csv_file, delimiter=',')  #  
        for row in reader:
            angle = str(row[0])  # 
            speed = str(row[1])
            duration = str(row[2])
            command_str = "angle: " + angle + ", speed: " + speed + ", duration: " + duration  # Set the command string to be sent to the Pi
            print(command_str)
            # ser1.write(bytes(command_str))  # Write the command string to the Pi in terms of bytes, because write() requires bytes to send info

