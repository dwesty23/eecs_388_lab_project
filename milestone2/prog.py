import serial
import csv

with open('milestone2.csv') as csv_file:
    with serial.Serial('/dev/ttyAMA1' , 115200 , timeout = 1) as ser1:
        reader = csv.reader(csv_file, delimiter=',', quotechar='|')
        for row in reader:
            angle = str(row[0])
            speed = str(row[1])
            duration = str(row[2])
            command_str = "angle: " + angle + " speed: " + speed + " duration: " + duration
            print(command_str)
            ser1.write(bytes(command_str))

