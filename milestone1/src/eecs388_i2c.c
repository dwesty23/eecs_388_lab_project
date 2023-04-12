#include <stdio.h>
#include <stdint.h>
#include "eecs388_lib.h"
#include "metal/i2c.h"


struct metal_i2c *i2c;
uint8_t bufWrite[9];
uint8_t bufRead[1];


//The entire setup sequence
void set_up_I2C(){
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
    *high = bigNum >> 8; // takes bigNum and shifts right by 8 bits so that the high 8 bits are taken
    // the shift is 8 because no overlap
}

void steering(int angle){
   printf("STEERING...%d\n", angle);
    uint16_t valToBreak = getServoCycle(angle); // passes in the angle to getServoCycle and stores the value in valToBreak
    uint8_t low, high; // creates two 8 bit numbers to pass into breakup
    breakup(valToBreak, &low, &high); // makes 2 8bit from 1 12bit number

    //talk to motor
    bufWrite[0] = PCA9685_LED0_ON_L; //this is a memory addy for servo
    bufWrite[1] = 0x00; // no idea
    bufWrite[2] = 0x00; // no idea
    bufWrite[3] = low; // give low bits
    bufWrite[4] = high; //give high bits
    
    //print steering values
    printf("Steering values: %d %d %d\n", valToBreak, bufWrite[3], bufWrite[4]); 

    // send info to the car
    metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
    
}

void stopMotor(){

   printf("STOPPING...\n"); //print stopping message
   uint8_t low, high; // creates two 8 bit numbers to pass into breakup
  
    breakup(280, &low, &high); // make low and high bit numbers for the stopping speed
    
    //talk to motors
    bufWrite[0] = PCA9685_LED1_ON_L; //this is a memory addy for motor direction control
    bufWrite[1] = 0x00; // no idea
    bufWrite[2] = 0x00; // no idea
    bufWrite[3] = low; // send low bits
    bufWrite[4] = high; // send high bits

    // send all the info to car
    metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
}

void driveForward(uint8_t speedFlag){
    
   printf("DRIVING FORWARD...\n");
   int speed = 0; // default speed
   switch(speedFlag){ // switch case for different speeds
    case 1:
        speed = 313;
        break;
    case 2:
        speed = 315;
        break;
    case 3:
        speed = 317;
        break;
   }
   uint8_t low, high;
   breakup(speed, &low, &high); // break into 2 8 bits

    // talk to board
   bufWrite[0] = PCA9685_LED1_ON_L; //addy
   bufWrite[1] = 0x00; //idk
   bufWrite[2] = 0x00; //idk
   bufWrite[3] = low; //send low bits
   bufWrite[4] = high; //send high bits

    // send to car
   metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);

}

void driveReverse(uint8_t speedFlag){

   printf("DRIVING BACKWARD...\n");
   int speed = 0;
   switch(speedFlag){
    case 1:
        speed = 267;
        break;
    case 2:
        speed = 265;
        break;
    case 3:
        speed = 263;
        break;
   }
   uint8_t low, high;
   breakup(speed, &low, &high);

   bufWrite[0] = PCA9685_LED1_ON_L;
   bufWrite[1] = 0x00;
   bufWrite[2] = 0x00;
   bufWrite[3] = low;
   bufWrite[4] = high;

   metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
}

int main()
{
    set_up_I2C();
    steering(0);
    delay(2000);
    driveForward(1);
    delay(2000);
    steering(20);
    delay(2000);
    stopMotor();
    delay(2000);
    driveReverse(1);
    delay(2000);
    steering(0);
    delay(2000);
    stopMotor();




    
    //Defining the breakup function
    /*
        Task 1: breaking 12 bit into two 8-bit
        Define the function created that recieves a 12 bit number,
        0 to 4096 and breaks it up into two 8 bit numbers.

        Assign these values to a referenced value handed into
        the function. 

        ex: 
        uint8_t variable1;
        uint8_t variable2;
        breakup(2000,&variable1,&variable2);
        variable1 -> low 8 bits of 2000
        variable2 -> high 8 bits of 2000


    */    
    
    //Changing Steering Heading
    /*
        Task 2: using getServoCycle(), bufWrite, bufRead, 
        breakup(), and and metal_i2c_transfer(), implement 
        the function defined above to control the servo
        by sending it an angle ranging from -45 to 45.

        Use the getServoCycle function to get the value to 
        breakup.

        ex: 
        int valToBreak = getServoCycle(45);
        >>>sets valToBreak to 355
        
        note: the motor's speed controller is either 
        LED0 or LED1 depending on where its plugged into 
        the board. If its LED1, simply add 4 to the LED0
        address

        ex: steering(0); -> driving angle forward
    */
    
    
    //Motor config/stop. This will cause a second beep upon completion
    /*
        -Task 3: using bufWrite, bufRead, breakup(), and
        and metal_i2c_transfer(), implement the funcion made
        above. This function Configure the motor by 
        writing a value of 280 to the motor.

        -include a 2 second delay after calling this function
        in order to calibrate

        -Note: the motor's speed controller is either 
        LED0 or LED1 depending on where its plugged into 
        the board. If its LED1, simply add 4 to the LED0
        address

        ex: stopMotor();
    */


    /*
    ############################################################
        ATTENTION: The following section will cause the        
        wheels to move. Confirm that the robot is              
        Propped up to avoid it driving away, as well as         
        that nothing is touching the wheels and can get 
        caught in them

        If anything goes wrong, unplug the hifive board from
        the computer to stop the motors from moving 
        
        Avoid sticking your hand inside the 
        car while its wheels are spinning
    #############################################################
    */
    

    //Motor Forward
    /*
        -Task 4: using bufWrite, bufRead, breakup(), and
        and metal_i2c_transfer(), implement the function
        made above to Drive the motor forward. The given
        speedFlag will alter the motor speed as follows:
        
        speedFlag = 1 -> value to breakup = 303 
        speedFlag = 2 -> value to breakup = 305(Optional)
        speedFlag = 3 -> value to breakup = 307(Optional)

        -note 1: the motor's speed controller is either 
        LED0 or LED1 depending on where its plugged into 
        the board. If its LED1, simply add 4 to the LED0
        address

        ex: driveForward(1);
    */
    
    //Motor Reverse
    /*
        -Task 5: using bufWrite, bufRead, breakup(), and
        and metal_i2c_transfer(), implement the function
        made above to Drive the motor backward. The given
        speedFlag will alter the motor speed as follows:
        
        speedFlag = 1 -> value to breakup = 257 
        speedFlag = 2 -> value to breakup = 255(Optional)
        speedFlag = 3 -> value to breakup = 253(Optional)

        -note 1: the motor's speed controller is either 
        LED0 or LED1 depending on where its plugged into 
        the board. If its LED1, simply add 4 to the LED0
        address

        ex: driveReverse(1);
    */
    
    
    //Fully Controlling the PCA9685
    /*
        -Task 6: using previously defined functions, 
        perform the following sequence of actions
        
        -Configure the motors (wait for 2 seconds)
        -Set the steering heading to 0 degrees 
        -Drive forward (wait for 2 seconds)
        -Change the steering heading to 20 degrees (wait for 2 seconds)
        -Stop the motor (wait for 2 seconds)
        -Drive forward (wait for 2 seconds)
        -Set steering heading to 0 degrees (wait for 2 seconds)
        -Stop the motor
    */

}
