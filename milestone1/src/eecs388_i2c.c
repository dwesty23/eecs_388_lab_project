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
    /*
        Write Task 1 code here
    */
    low = bigNum & 0xFF; // takes bigNum and ANDs it with 11111111 so that the low 8 bits are taken
    high = bigNum >> 8; // takes bigNum and shifts right by 4 bits so that the high 8 bits are taken
    // the shift is 8 because no overlap, i figured out why!
}

void steering(int angle){
    /*
        Write Task 2 code here
    */
    int valToBreak = getServoCycle(angle); // passes in the angle to getServoCycle and stores the value in valToBreak
    //uint8_t low, high; // creates two 8 bit numbers to pass into breakup

    //print the steering values (cycle, low and high)

    //my friend's TA fed them this...
    bufWrite[0] = PCA9685_LED0_ON_L; //this is a memory addy for steering
    bufWrite[1] = 0; // no idea
    bufWrite[2] = 0; // no idea
    //bufWrite[3] = low; // i think bufWrite is just an array of values that gets fed 
                        //into the metal_i2c_transfer function along with a bunch of other stuff
                        // to communicate with motors
    //bufWrite[4] = high;

    breakup(valToBreak, &bufWrite[3], &bufWrite[4]); // passes in the 12 bit valToBreak and the low and high 8 bit numbers that the broken up number will store into
    printf("Steering values: %d %d %d\n", valToBreak, bufWrite[3], bufWrite[4]); 

    metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
    // this block is pretty much how you do everything as far as communicating with motors -> I HAVE NO IDEA WHY...
    // -david
}

void stopMotor(){
    /*
        Write Task 3 code here
    */
   int valToBreak = getServoCycle(280); // NOT SUPPOSED TO BE 'ANGLE' we need to find the random number here that gets passed iN that means stop
    //my friend's TA fed them this...
    bufWrite[0] = PCA9685_LED1_ON_L; //this is a memory addy for motor direction control
    bufWrite[1] = 0; // no idea
    bufWrite[2] = 0; // no idea

    breakup(valToBreak, &bufWrite[3], &bufWrite[4]); // passes in the 12 bit valToBreak and the low and high 8 bit numbers that the broken up number will store into

    metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
}

void driveForward(uint8_t speedFlag){
    /*
        Write Task 4 code here
    */
   if (speedFlag == 1) {
        int valToBreak = getServoCycle(313); // NOT SUPPOSED TO BE 'ANGLE' we need to find the random number here that gets passed iN that means stop
        //my friend's TA fed them this...
        bufWrite[0] = PCA9685_LED1_ON_L; //this is a memory addy for motor direction control
        bufWrite[1] = 0; // no idea
        bufWrite[2] = 0; // no idea

        breakup(valToBreak, &bufWrite[3], &bufWrite[4]); // passes in the 12 bit valToBreak and the low and high 8 bit numbers that the broken up number will store into

        metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);

   } else if (speedFlag == 2) {
        int valToBreak = getServoCycle(315); // NOT SUPPOSED TO BE 'ANGLE' we need to find the random number here that gets passed iN that means stop
        //my friend's TA fed them this...
        bufWrite[0] = PCA9685_LED1_ON_L; //this is a memory addy for motor direction control
        bufWrite[1] = 0; // no idea
        bufWrite[2] = 0; // no idea

        breakup(valToBreak, &bufWrite[3], &bufWrite[4]); // passes in the 12 bit valToBreak and the low and high 8 bit numbers that the broken up number will store into

        metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
   } else if (speedFlag == 3) {
        int valToBreak = getServoCycle(317); // NOT SUPPOSED TO BE 'ANGLE' we need to find the random number here that gets passed iN that means stop
        //my friend's TA fed them this...
        bufWrite[0] = PCA9685_LED1_ON_L; //this is a memory addy for motor direction control
        bufWrite[1] = 0; // no idea
        bufWrite[2] = 0; // no idea

        breakup(valToBreak, &bufWrite[3], &bufWrite[4]); // passes in the 12 bit valToBreak and the low and high 8 bit numbers that the broken up number will store into

        metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
   }

}

void driveReverse(uint8_t speedFlag){
    /*
        Write task 5 code here
    */
   if (speedFlag == 1) {
        int valToBreak = getServoCycle(267); // NOT SUPPOSED TO BE 'ANGLE' we need to find the random number here that gets passed iN that means stop
        //my friend's TA fed them this...
        bufWrite[0] = PCA9685_LED0_Off_L; //this is a memory addy for something :/
        bufWrite[1] = 0; // no idea
        bufWrite[2] = 0; // no idea

        breakup(valToBreak, &bufWrite[3], &bufWrite[4]); // passes in the 12 bit valToBreak and the low and high 8 bit numbers that the broken up number will store into

        metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);

   } else if (speedFlag == 2) {
        int valToBreak = getServoCycle(265); // NOT SUPPOSED TO BE 'ANGLE' we need to find the random number here that gets passed iN that means stop
        //my friend's TA fed them this...
        bufWrite[0] = PCA9685_LED0_Off_L; //this is a memory addy for something :/
        bufWrite[1] = 0; // no idea
        bufWrite[2] = 0; // no idea

        breakup(valToBreak, &bufWrite[3], &bufWrite[4]); // passes in the 12 bit valToBreak and the low and high 8 bit numbers that the broken up number will store into

        metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
   } else if (speedFlag == 3) {
        int valToBreak = getServoCycle(263); // NOT SUPPOSED TO BE 'ANGLE' we need to find the random number here that gets passed iN that means stop
        //my friend's TA fed them this...
        bufWrite[0] = PCA9685_LED0_Off_L; //this is a memory addy for something :/
        bufWrite[1] = 0; // no idea
        bufWrite[2] = 0; // no idea

        breakup(valToBreak, &bufWrite[3], &bufWrite[4]); // passes in the 12 bit valToBreak and the low and high 8 bit numbers that the broken up number will store into

        metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 5, bufRead, 1);
   }
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
