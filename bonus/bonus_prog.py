import serial
import csv

def validate(file1, file2):
    with open(file1) as csv_file1:  # Open the csv file. Will close the csv file when done running
        with open(file2) as csv_file2:  # Open the csv file. Will close the csv file when done running
                reader1 = csv.reader(csv_file1, delimiter=',')  #  Creates a reader object that holds a list of lists of all of the data
                reader2 = csv.reader(csv_file2, delimiter=',')  #  Creates a reader object that holds a list of lists of all of the data
                for (row1, row2) in zip(reader1, reader2):  # Iterate through each row in the entire reader file
                    if (row1 != row2):
                        return False
    return True

def main():
    if validate('bonus/bonus1.csv', 'bonus/bonus2.csv'):
        with open('bonus/bonus1.csv') as csv_file:  # Open the csv file. Will close the csv file when done running
            with serial.Serial('/dev/ttyAMA1' , 115200 , timeout = 1) as ser1:  # Creates a serial chanel over /dev/ttyAMA1 with a baudrate of 115200. Sets it to ser1 so it's easy to call functions on
                reader = csv.reader(csv_file, delimiter=',')  #  Creates a reader object that holds a list of lists of all of the data
                for row in reader:  # Iterate through each row in the entire reader file
                    angle = str(row[0])  # Assign the angle to the first item
                    speed = str(row[1])  # Assign the speed to the second item
                    duration = str(row[2])  # Assign the duration to the third item
                    command_str = "angle: " + angle + ", speed: " + speed + ", duration: " + duration  # Set the command string to be sent to the Pi
                    ser1.write(bytes(command_str))  # Write the command string to the Pi in terms of bytes, because write() requires bytes to send info

    else:
        print("files are not equal, stopping car")
        with serial.Serial('/dev/ttyAMA1' , 115200 , timeout = 1) as ser1:
            angle = 0
            speed = 0
            duration = 0
            command_str = "angle: " + angle + ", speed: " + speed + ", duration: " + duration  # Set the command string to be sent to the Pi
            ser1.write(bytes(command_str))  # Write the command string to the Pi in terms of bytes, because write() requires bytes to send info

main()