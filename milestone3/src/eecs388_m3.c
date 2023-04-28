#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "eecs388_lib.h"
#include "metal/i2c.h"

struct metal_i2c *i2c;
uint8_t bufWrite[5];
uint8_t bufRead[1];

//The entire setup sequence
void set_up_I2C()
{
    uint8_t oldMode;
    uint8_t newMode;
    _Bool success;

    bufWrite[0] = PCA9685_MODE1;
    bufWrite[1] = MODE1_RESTART;
    printf("%d\n",bufWrite[0]);
    
    i2c = metal_i2c_get_device(0);

    if(i2c == NULL){
        printf("Connection Unsuccessful\n");
    }
    else{
        printf("Connection Successful\n");
    }
    
    //Setup Sequence
    metal_i2c_init(i2c,I2C_BAUDRATE,METAL_I2C_MASTER);
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//reset
    delay(100);
    printf("resetting PCA9685 control 1\n");

    //Initial Read of control 1
    bufWrite[0] = PCA9685_MODE1;//Address
    success = metal_i2c_transfer(i2c,PCA9685_I2C_ADDRESS,bufWrite,1,bufRead,1);//initial read
    printf("Read success: %d and control value is: %d\n", success, bufWrite[0]);
    
    //Configuring Control 1
    oldMode = bufRead[0];
    newMode = (oldMode & ~MODE1_RESTART) | MODE1_SLEEP;
    printf("sleep setting is %d\n", newMode);
    bufWrite[0] = PCA9685_MODE1;//address
    bufWrite[1] = newMode;//writing to register
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//sleep
    bufWrite[0] = PCA9685_PRESCALE;//Setting PWM prescale
    bufWrite[1] = 0x79;
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//sets prescale
    bufWrite[0] = PCA9685_MODE1;
    bufWrite[1] = 0x01 | MODE1_AI | MODE1_RESTART;
    printf("on setting is %d\n", bufWrite[1]);
    success = metal_i2c_write(i2c,PCA9685_I2C_ADDRESS,2,bufWrite,METAL_I2C_STOP_DISABLE);//awake
    delay(100);
    printf("Setting the control register\n");
    bufWrite[0] = PCA9685_MODE1;
    success = metal_i2c_transfer(i2c,PCA9685_I2C_ADDRESS,bufWrite,1,bufRead,1);//initial read
    printf("Set register is %d\n",bufRead[0]);
} 

void breakup(int bigNum, uint8_t* low, uint8_t* high)
{
    *low = (uint8_t)bigNum & 0xFF; // takes bigNum and ANDs it with 11111111 so that the low 8 bits are taken
    *high = bigNum >> 8; // takes bigNum and shifts right by 8 bits so that the high 4 bits are taken with 0000 leading them
}

void steering(int angle)
{
    uint16_t valToBreak = getServoCycle(angle); // passes in the angle to getServoCycle and stores the value in valToBreak
    uint8_t low, high; // creates two 8 bit numbers to pass into breakup
    breakup(valToBreak, &low, &high); // makes 2 8 bit numbers from 1 12bit number
    printf("STEERING - %d\n" , angle);

    //talk to motor
    bufWrite[0] = PCA9685_LED0_ON_L; //this is a memory address for servo
    bufWrite[1] = 0x00;
    bufWrite[2] = 0x00;
    bufWrite[3] = low; // store low bits
    bufWrite[4] = high; // store high bits

    metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1); // send info to the car
    // calls transfer to transfer the bufWrite and bufRead arrays to the i2c to control the car
}

void stopMotor()
{
    uint8_t low, high; // creates two 8 bit numbers to pass into breakup
  
    breakup(280, &low, &high); // make low and high bit numbers for the stopping speed
    printf("STOPPING\n");
    
    //talk to motors
    bufWrite[0] = PCA9685_LED1_ON_L; //this is a memory address for motor direction control
    bufWrite[1] = 0x00;
    bufWrite[2] = 0x00; 
    bufWrite[3] = low; // store low bits
    bufWrite[4] = high; // store high bits

    metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1); // send all the info to car
    // calls transfer to transfer the bufWrite and bufRead arrays to the i2c to control the car
}

void driveForward(uint8_t speedFlag)
{
    int speed = 0; // default speed
    printf("DRIVE FORWARD\n");

    switch(speedFlag){ // switch case for different speeds depending on passed in speedFlag value
        case 1:
            speed = 313; // low speed value in positive direction
            break;
        case 2:
            speed = 315; // medium speed value in positive direction
            break;
        case 3:
            speed = 317; // high speed value in positive direction
            break;
    }

    uint8_t low, high;
    breakup(speed, &low, &high); // break into 2 8 bits
    
    //talk to motors
    bufWrite[0] = PCA9685_LED1_ON_L; //this is a memory address for motor direction control
    bufWrite[1] = 0x00;
    bufWrite[2] = 0x00; 
    bufWrite[3] = low; // store low bits
    bufWrite[4] = high; // store high bits
    
    metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1); // send all the info to car
    // calls transfer to transfer the bufWrite and bufRead arrays to the i2c to control the car
}

void driveReverse(uint8_t speedFlag)
{
    int speed = 0; // default speed
    printf("DRIVE BACKWARD\n");
    switch(speedFlag){ // switch case for different speeds depending on passed in speedFlag value
        case 1:
            speed = 267; // low speed value in negative direction
            break;
        case 2:
            speed = 265; // medium speed value in negative direction
            break;
        case 3:
            speed = 263; // high speed value in negative direction
            break;
    }
    uint8_t low, high;
    breakup(speed, &low, &high);
    
    //talk to motors
    bufWrite[0] = PCA9685_LED1_ON_L; //this is a memory address for motor direction control
    bufWrite[1] = 0x00;
    bufWrite[2] = 0x00;
    bufWrite[3] = low; // store low bits
    bufWrite[4] = high; // store high bits
    
    metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1); // send all the info to car
    // calls transfer to transfer the bufWrite and bufRead arrays to the i2c to control the car
}

void raspberrypi_int_handler(int devid, int * angle, int * speed, int * duration)
{
    char * str = malloc(20 * sizeof(char)); // you can use this to store the received string
                
    //char str[10];                                        // it is the same as char str[20]
    /*printf("before readline\n");
    int read = ser_readline(devid,20, &str);
    printf("stopped reading at read: %d\n", read);
    //str = ser_read(1);
    printf("line was read\n");

    printf(&str);
    printf("\n");
    
    printf("before sscanf\n");
    sscanf(str, "a:%d s:%d d:%d", &angle,&speed,&duration);
    //sscanf(str+7,"%d",&angle);
    printf("after sscanf\n");

    printf(angle);
    printf(speed);
    printf(duration);
    



   // Extract the values of angle, speed and duration inside this function
   // And place them into the correct variables that are passed in

    free(str);
    */
}


int main()
{
    // Initialize I2C
    set_up_I2C();
    delay(2000);

    // Calibrate Motor
    printf("Calibrate Motor.\n");
    stopMotor();
    delay(2000);

    // initialize UART channels
    //ser_setup(0); // uart0 (receive from raspberry pi)
    //ser_setup(1);
    
    printf("Setup completed.\n");
    printf("Begin the main loop.\n");
    

    //int angle, speed, duration;
    // Drive loop
    while (1) {
        // The following pseudo-code is a rough guide on how to write your code
        // It is NOT actual C code
         /*  
          if (is UART0 ready?)
          {
              call raspberrypi_int_handler() to get values of angle, speed and duration

              call steering(), driveForward/Reverse() and delay with the extracted values
          }
        */
       ser_setup(0); // uart0 (receive from raspberry pi)
        ser_setup(1);
       if (ser_isready(1)){
            printf("READY\n");
            //raspberrypi_int_handler(1,&angle,&speed,&duration);

            //char * str = malloc(20 * sizeof(char)); // you can use this to store the received string
        
            char str[20];                                        // it is the same as char str[20]
            printf("before readline\n");
            int read = ser_readline(1,20, str);
            printf("stopped reading at read: %d\n", read);
            //str = ser_read(1);
            printf("line was read\n");

            printf("%s\n",str);
            printf("\n");
            
            printf("before sscanf\n");
            int angle, speed, duration;
            sscanf(str, "a:%d s:%d d:%d", &angle,&speed,&duration);
            //sscanf(str+7,"%d",&angle);
            printf("after sscanf\n");

            printf("angle: %d\n",angle);
            printf("speed: %d\n", speed);
            printf("duration: %d\n", duration);
            



        // Extract the values of angle, speed and duration inside this function
        // And place them into the correct variables that are passed in

            free(str);
            
            steering(angle);

            if (speed < 0) {
                driveReverse(abs(speed));
            } else if (speed == 0) {
                stopMotor();
            } else {
                driveForward(speed);
            }
            
            delay(duration * 1000);



        }
    }
}
