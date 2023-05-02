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
// your code from Milestone 1
    *low = bigNum & 0xff;
    *high = (bigNum>>8);
}

void steering(int angle)
{
// your code from Milestone 1
    int cycle = getServoCycle(angle);
    bufWrite[0] = PCA9685_LED1_ON_L; //Either LED 1 or 0
    bufWrite[1] = 0x00;
    bufWrite[2] = 0x00;
    breakup(cycle, &bufWrite[3], &bufWrite[4]);
    metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
}

void stopMotor()
{
// your code from Milestone 1
    uint8_t var1, var2;
    breakup(280, &var1, &var2);
    bufWrite[0] = PCA9685_LED0_ON_L;
    bufWrite[1] = 0x00;
    bufWrite[2] = 0x00;
    bufWrite[3] = var1;
    bufWrite[4] = var2;
    metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
}

void driveForward(uint8_t speedFlag)
{
// your code from Milestone 1
    uint8_t variable1;
    uint8_t variable2;
    printf("Starting to drive");
    if (speedFlag == 1) {
        breakup(313, &variable1, &variable2);
        bufWrite[0] = PCA9685_LED0_ON_L;
        bufWrite[1] = 0x00;
        bufWrite[2] = 0x00;
        bufWrite[3] = variable1;
        bufWrite[4] = variable2;
        metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
        // int drive_low = metal_i2c_write(i2c, PCA9685_LED0_OFF_L, 1, variable1, METAL_I2C_STOP_DISABLE);
        // int drive_high = metal_i2c_write(i2c, PCA9685_LED0_OFF_H, 1, variable2, METAL_I2C_STOP_ENABLE);
    } else if (speedFlag == 2) {
        breakup(315, &variable1, &variable2);
        bufWrite[0] = PCA9685_LED0_ON_L;
        bufWrite[1] = 0x00;
        bufWrite[2] = 0x00;
        bufWrite[3] = variable1;
        bufWrite[4] = variable2;
        metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
        // int drive_low = metal_i2c_write(i2c, PCA9685_LED0_OFF_L, 1, variable1, METAL_I2C_STOP_DISABLE);
        // int drive_high = metal_i2c_write(i2c, PCA9685_LED0_OFF_H, 1, variable2, METAL_I2C_STOP_ENABLE);
    } else if (speedFlag == 3) {
        breakup(317, &variable1, &variable2);
        bufWrite[0] = PCA9685_LED0_ON_L;
        bufWrite[1] = 0x00;
        bufWrite[2] = 0x00;
        bufWrite[3] = variable1;
        bufWrite[4] = variable2;
        metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
        // int drive_low = metal_i2c_write(i2c, PCA9685_LED0_OFF_L, 1, variable1, METAL_I2C_STOP_DISABLE);
        // int drive_high = metal_i2c_write(i2c, PCA9685_LED0_OFF_H, 1, variable2, METAL_I2C_STOP_ENABLE);
    }
    printf("Finished Driving");
}

void driveReverse(uint8_t speedFlag)
{
// your code from Milestone 1
    uint8_t variable1;
    uint8_t variable2;
    if (speedFlag == 1) {
        breakup(267, &variable1, &variable2);
        bufWrite[0] = PCA9685_LED0_ON_L;
        bufWrite[1] = 0x00;
        bufWrite[2] = 0x00;
        bufWrite[3] = variable1;
        bufWrite[4] = variable2;
        metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
    } else if (speedFlag == 2) {
        breakup(265, &variable1, &variable2);
        bufWrite[0] = PCA9685_LED0_ON_L;
        bufWrite[1] = 0x00;
        bufWrite[2] = 0x00;
        bufWrite[3] = variable1;
        bufWrite[4] = variable2;
        metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
    } else if (speedFlag == 3) {
        breakup(263, &variable1, &variable2);
        bufWrite[0] = PCA9685_LED0_ON_L;
        bufWrite[1] = 0x00;
        bufWrite[2] = 0x00;
        bufWrite[3] = variable1;
        bufWrite[4] = variable2;
        metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
    }
}

void raspberrypi_int_handler(int devid, int * angle, int * speed, int * duration)
{
    char * str = malloc(20 * sizeof(char)); // you can use this to store the received string
                
    //char str[20];                                        // it is the same as char str[20]
    printf("before readline\n");
    int read = ser_readline(devid,20, str);
    printf("stopped reading at read: %d\n", read);
    //str = ser_read(1);
    printf("line was read\n");

    //printf(&str);
    //printf("\n");


    printf("before sscanf\n");
    sscanf(str, "%d %d %d\n", angle,speed,duration);
    printf("after sscanf\n");

/*
    printf(&angle);
    printf(&speed);
    printf(&duration);
*/
    



   // Extract the values of angle, speed and duration inside this function
   // And place them into the correct variables that are passed in
    free(str);
    return;
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
            int angle, speed, duration;
            raspberrypi_int_handler(1,&angle,&speed,&duration);
            printf("EXITED RASP INT\n");
            
            printf("Angle: %d, Speed: %d, Duration: %d\n", angle, speed, duration);


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
    return 0;
}
