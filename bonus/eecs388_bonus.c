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

void breakup(int bigNum, uint8_t* low, uint8_t* high){
    *low = (uint8_t)bigNum & 0xFF; // takes bigNum and ANDs it with 11111111 so that the low 8 bits are taken
    *high = bigNum >> 8; // takes bigNum and shifts right by 8 bits so that the high 4 bits are taken with 0000 leading them
    printf("inside breakup call low: %d\n", low);
    printf("inside breakup call high: %d\n", high);
}

void steering(int angle){
    printf("STEERING - %d\n" , angle);
    uint16_t valToBreak = getServoCycle(angle); // passes in the angle to getServoCycle and stores the value in valToBreak
    printf("ValToBreak = %d", valToBreak);
    uint8_t low, high; // creates two 8 bit numbers to pass into breakup
    breakup(valToBreak, &low, &high); // makes 2 8 bit numbers from 1 12bit number
    printf("inside function call low: %d\n", low);
    printf("inside function call high: %d\n", high);

    //talk to motor
    bufWrite[0] = PCA9685_LED1_ON_L; //this is a memory address for servo
    bufWrite[1] = 0x00;
    bufWrite[2] = 0x00;
    bufWrite[3] = low; // store low bits
    bufWrite[4] = high; // store high bits

    metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1); // send info to the car
    // calls transfer to transfer the bufWrite and bufRead arrays to the i2c to control the car
    
}

void stopMotor(){
   uint8_t low, high; // creates two 8 bit numbers to pass into breakup
  
    breakup(280, &low, &high); // make low and high bit numbers for the stopping speed
    
    printf("STOPPING\n");
    printf("inside function call low: %d\n", low);
    printf("inside function call high: %d\n", high);
    
    //talk to motors
    bufWrite[0] = PCA9685_LED0_ON_L; //this is a memory address for motor direction control
    bufWrite[1] = 0x00;
    bufWrite[2] = 0x00; 
    bufWrite[3] = low; // store low bits
    bufWrite[4] = high; // store high bits

    metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1); // send all the info to car
    // calls transfer to transfer the bufWrite and bufRead arrays to the i2c to control the car
}

void driveForward(uint8_t speedFlag){
   int speed = 0; // default speed
   printf("DRIVE FORWARD - %d\n", speedFlag);

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
   bufWrite[0] = PCA9685_LED0_ON_L; //this is a memory address for motor direction control
   bufWrite[1] = 0x00;
   bufWrite[2] = 0x00; 
   bufWrite[3] = low; // store low bits
   bufWrite[4] = high; // store high bits
   
   metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1); // send all the info to car
   // calls transfer to transfer the bufWrite and bufRead arrays to the i2c to control the car

}

void driveReverse(uint8_t speedFlag){
   int speed = 0; // default speed
   printf("DRIVE BACKWARD - %d\n", speedFlag);
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
   bufWrite[0] = PCA9685_LED0_ON_L; //this is a memory address for motor direction control
   bufWrite[1] = 0x00;
   bufWrite[2] = 0x00;
   bufWrite[3] = low; // store low bits
   bufWrite[4] = high; // store high bits
   
   metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1); // send all the info to car
   // calls transfer to transfer the bufWrite and bufRead arrays to the i2c to control the car
}

void raspberrypi_int_handler(int devid, int * index1, int * angle1, int * speed1, int * duration1, int * index2, int * angle2, int * speed2, int * duration2)
{
    char * str = malloc(30 * sizeof(char)); // you can use this to store the received string
                
    //char str[20];                                        // it is the same as char str[20]
    printf("before readline\n");
    int read = ser_readline(devid,30, str);
    printf("stopped reading at read: %d\n", read);
    //str = ser_read(1);
    printf("line was read\n");

    //printf(&str);
    //printf("\n");


    printf("before sscanf\n");
    sscanf(str, "%d %d %d %d %d %d %d %d\n", index1, angle1, speed1, duration1, index2, angle2, speed2, duration2);
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

int validate(int i1, int a1, int s1, int d1, int i2, int a2, int s2, int d2) { // verifies that the values are the same
    if(i1 == i2 && a1 == a2 && s1 == s2 && d1 == d2) { // runs if all the values match 
        update_car(a1, s1, d1); // updates the car 
        return 1; // returns 1 to keep the flag loop running
    } 

    return 0; // returns 0 if the values don't match to trigger the flag
}

void update_car(int angle, int speed, int duration) { // runs the car with the passed in values
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
    

    
    int flag = 1;
    // Drive loop
    while (flag != 0) {
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
       if (ser_isready(1) && ser_isready(2)){
            printf("READY\n");
            int index1, angle1, speed1, duration1, index2, angle2, speed2, duration2; // defines variables for each UART read
            raspberrypi_int_handler(1,&index1,&angle1,&speed1,&duration1, &index2,&angle2,&speed2,&duration2); // reads from devid 1
            printf("EXITED RASP INT\n");
            printf("Angle1: %d, Speed1: %d, Duration1: %d\n", angle1, speed1, duration1);
            printf("Angle2: %d, Speed2: %d, Duration2: %d\n", angle2, speed2, duration2);
            flag = validate(index1, angle1, speed1, duration1, index2, angle2, speed2, duration2); // validates the values and runs if the same or returns 0 for emergency flag
        }
    }
    if (flag == 0) { // runs if a line doesn't match so the emergency route is called
        update_car(0, 0, 4); // index 1
        update_car(-40, 1, 3); // index 2
        update_car(0, 0, 4); // index 3
        update_car(40, -1, 4); // index 4
        update_car(0, 0, 4); // index 5
        /*
        1,0,0,4
        2,-40,1,3
        3,0,0,4
        4,40,-1,4
        5,0,0,4
        */
    }
    return 0;
}
