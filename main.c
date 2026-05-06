#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#pragma config OSC = HS        //High speed resonator
#pragma config WDT = OFF       //Watchdog timer off
#pragma config LVP = OFF       //Low voltage programming disabled
#pragma config PWRT = ON       //Power up timer on
#define Leftmotor1A LATAbits.LA4    //Direction bits Left motor
#define Leftmotor2A LATAbits.LA5
#define Rightmotor3A LATBbits.LB0   //Direction bits Right motor
#define Rightmotor4A LATBbits.LB1

//already in the file
void configPWM(void);       //Configure PWM
void goforward(void);       //Turn both motors forward
int markspaceL;    //Mark space ratio for Left motor
int markspaceR;    //Mark space ratio for Right motor
int baseSpeed = 280;
int stage = 0; //Global variable to keep track of which stage robot is running in
    

    //This is the correct "staging"

    //Stage 0 : Robot starts on outer loop, follows line until bar and changes lane.
    //Stage 1: Robot now on inner loop, follows line until it hits the bar. Stops at bar.
    //Stage 2: Robot rotate 180, then follow inner loop until next bar and change lane.
    //Stage 3: Robot follows outer loop until hit bar, then halts.
    //Stage 4: Robot flashes LEDs indefinitely


void I2C_Initialise(void);          //Initialise I2C
void I2C_checkbus_free(void);   //Wait until I2C bus is free
void I2C_Start(void);               //Generate I2C start condition
void I2C_RepeatedStart(void);       //Generate I2C Repeat start condition
void I2C_Stop(void);                //Generate I2C stop condition
void I2C_Write(unsigned char write);    //Generate I2C write condition
unsigned char I2C_Read(void);       //Generate I2C read condition
void halt(void); //thou shalt not pass


//created by us
int AngleFromI2C(unsigned char inData);

//-------------------------------
// MAIN
//-------------------------------
void main(void){
    
    //Please sanity check this logic.
    if (stage == 0 || stage == 2 || stage == 3) {
        //Normal stuff
        
    } else if (stage == 2){
        //Spin 180, then swap to stage 1 behaviour
    } else if (stage == 4){
        //Halt robot
        //Flash LEDs
        
    }

  //from I2C file
  int angle = 0, last_angle = 0;
  int angle_error, u, lambda = 2;
  int angle_kp = 7;
  int angle_kd = 12;
  int phaseCount = 0;
  
  int barCounter=0;
  int rotateDone=0;
  int flashState=0;
  
  

  unsigned char linesensor;         //Store raw data from sensor array
  TRISC = 0xFF;                     //Set PORTC as inputs
  TRISB = 0x00;                     //Set PORTB as outputs
  LATB = 0x00;                      //Turn All LEDs off
  I2C_Initialise();                 //Initialise I2C Master

  //from pwm file
  ADCON1 = 0b00001101;    //AN0,AN1 are analogue inputs,RA2 -RA5 are digital
  TRISA = 0b11001111;    //Set PORTA pins
  TRISB = 0;                   //Set all PORTB pins to outputs
  TRISC = 0b00111001;    //Set PORTC pins
  LATB = 0;                   //Turn LEDs off
  configPWM();                //Configure PWM

  markspaceL=200;   //Left initial motor speed
  markspaceR=200;   //Right initial motor speed

  while(1){

        I2C_Start();                    //Send Start condition to slave
        I2C_Write(0x7C);                //Send 7 bit address + Write to slave
        I2C_Write(0x11);                //Write data, select RegdataA and send to slave
        I2C_RepeatedStart();            //Send repeat start condition
        I2C_Write(0x7D);                //Send 7 bit address + Read
        linesensor=I2C_Read();          //Read  the IR sensors
        LATB=linesensor;                //Output to LEDs
        I2C_Stop();                     //Send Stop condition

        //Proportional control for angle
        last_angle = angle;

        angle  = AngleFromI2C(linesensor);
        //angle = 0;

        if(angle == 69){                //set angle to last angle if robot has gone off line (as last angle should be -12 or 12) to correct
            angle = 0;
            //
        }
        angle_error = 0 - angle;
        
        u = angle_kp*angle_error + angle_kd * (last_angle-angle);
        markspaceL = baseSpeed - lambda*u;
        markspaceR = baseSpeed + lambda*u;

        if(markspaceL > 1023) markspaceL = 1023;       //potential clamping for speed DO WE NEED THIS AS ANGLE LIMITED ANYWAY???
        if(markspaceR > 1023) markspaceR = 1023;
        if(markspaceL < 0) markspaceL = 0;
        if(markspaceR < 0) markspaceR = 0;
        goforward();          //Turn both motor forward
   }
}

// White line on black
int AngleFromI2C(unsigned char inData){
    int magicNum = 10;
    static int count;
      
    
    
    if (count > 0){
        
        if (count<310){
            count = count+1  ;
            
            }
        else{
            count = 0;
        }
        
        return magicNum;
    }
    count=0;
    
    switch(inData){
        case 0b10000001:  //Whole white bar is fullish
            
            count++;
            return magicNum; //Return 67 for WHITE BAR
            
        case 0b11000011:  //Whole white bar is fullish
            
            count++;
            return magicNum; //Return 67 for WHITE BAR
            
        case 0b00000000:  //Whole white bar is fullish
            
            count++;
            return magicNum; //Return 67 for WHITE BAR
            
        case 0b11111110:  //12 degrees
            return 12;
        case 0b11111100:  //10 degrees
            return 10;
        case 0b11111101:  //9 degrees
            return 9;
        case 0b11111001:  //7 degrees
            return 7;
        case 0b11111011:  //5 degrees
            return 5;
        case 0b11110011:  //3 degrees
            return 3;
        case 0b11110111:  //2 degrees
            return 2;
        case 0b11100111:  //0 degrees
            return 0;
        case 0b11101111:  //-2 degrees
            return -2;
        case 0b11001111:  //-3 degrees
            return -3;
        case 0b11011111:  //-5 degrees
            return -5;
        case 0b10011111:  //-7 degrees
            return -7;
        case 0b10111111:  //-9 degrees
            return -9;
        case 0b00111111:  //-10 degrees
            return -10;
        case 0b01111111:  //-12 degrees
            return -12;
        default:
            return 69; //Default value (no white detected)
    }
}

//functions from I2C file

void I2C_Initialise(void)      //Initialise I2C
{
  SSPCON1 = 0b00101000;     //set to master mode, enable SDA and SCL pins
  SSPCON2 = 0;                  //reset control register 2
  SSPADD = 0x63;                //set baud rate to 100KHz
  SSPSTAT = 0;                  //reset status register
  }
void I2C_checkbus_free(void)        //Wait until I2C bus is free
{
  while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));    //wait until I2C bus is free
}

void I2C_Start(void)        //Generate I2C start condition
{
  I2C_checkbus_free();      //Test to see I2C bus is free
  SEN = 1;                  //Generate start condition,SSPCON2 bit 0 = 1
}

void I2C_RepeatedStart(void)     //Generate I2C Repeat start condition
{
  I2C_checkbus_free();          //Test to see I2C bus is free
  RSEN = 1;                     //Generate repeat start, SSPCON2 bit1 = 1
}

void I2C_Stop(void)         //Generate I2C stop condition
{
  I2C_checkbus_free();          //Test to see I2C bus is free
  PEN = 1;                      // Generate stop condition,SSPCON2 bit2 = 1
}

void I2C_Write(unsigned char write)     //Write to slave
{
  I2C_checkbus_free();          //check I2C bus is free
  SSPBUF = write;               //Send data to transmit buffer
}

unsigned char I2C_Read(void)    //Read from slave
{
  unsigned char temp;
  I2C_checkbus_free();      //Test to see I2C bus is free
  RCEN = 1;                 //enable receiver,SSPCON2 bit3 = 1
  I2C_checkbus_free();      //Test to see I2C bus is free
  temp = SSPBUF;            //Read slave
  I2C_checkbus_free();      //Test to see I2C bus is free
  ACKEN = 1;                //Acknowledge
  return temp;              //return sensor array data
}

//functions from pwm file

void configPWM(void){   //Configures PWM
PR2 = 0b11111111 ;     //set period of PWM,610Hz
T2CON = 0b00000111 ;   //Timer 2(TMR2)on, prescaler = 16
CCP1CON = 0b00001100;   //enable CCP1 PWM
CCP2CON = 0b00001100;   //enable CCP2 PWM
CCPR1L = 0;             //turn left motor off
CCPR2L = 0;             //turn Right motor off
return;
}

void goforward(void){
Leftmotor1A = 0;    //Left motor forward;
Leftmotor2A = 1;
Rightmotor3A = 0;   //Right motor forward;
Rightmotor4A = 1;


CCP1CON = (unsigned char)((0x0c) | ((markspaceR & 0x03) << 4));
CCPR1L  = (unsigned char)(markspaceR >> 2);

CCP2CON = (unsigned char)((0x0c) | ((markspaceL & 0x03) << 4));
CCPR2L  = (unsigned char)(markspaceL >> 2);
return;
}


void halt(void){
    //thou shalt not pass
    Leftmotor1A = 1;    //Left motor forward;
    Leftmotor2A = 1;
    Rightmotor3A = 1;   //Right motor forward;
    Rightmotor4A = 1;
}

// Encoder functions from encoders-working.c

void configEncoders() {
    // Configure RC0 and RC5 as inputs for encoder reading
    TRISCbits.TRISC0 = 1;
    TRISCbits.TRISC5 = 1;

    //read states to previous states
    prevLeftState = PORTCbits.RC0;
    prevRightState = PORTCbits.RC5;
}

void encoderTurn(int angle){
    //Turns using encoders in deg
    //Positive angle -> right turn
    //Negative angle -> left turn
    int steps = angleToEncoderSteps(angle);

    if (angle > 0){
        SpinRight();
        int count = 0;
        int state = PORTCbits.RC0; // initialize to actual state
        int prevState = state;
        while (count < steps){
            state = PORTCbits.RC0; //read state of encoder
            if (state != prevState){
                count = count+1;
            }
            prevState = state; // update after the check
        }
        Stop();
    }
    else {
        SpinLeft();
        int count = 0;
        int state = PORTCbits.RC0; // initialize to actual state
        int prevState = state;
        while (count < steps){
            state = PORTCbits.RC0; //read state of encoder
            if (state != prevState){
                count = count+1;
            }
            prevState = state; // update after the check
        }
        Stop();
    }
}

int angleToEncoderSteps(int angle){
    //Converts angle in deg to encoder steps
    //Returns number of encoder steps for given angle
    //Divide angle by 360 and multiply by number of steps per full rotation
    int stepsRotMagicNumber = 800;

    // Fixed logic: must multiply before dividing to avoid integer truncation to 0
    return (stepsRotMagicNumber * angle) / 360;
}

void SpinRight(void){        //set H bridge to make robot spin to the right
    Leftmotor1A = 0;
    Leftmotor2A = 1;
    Rightmotor3A = 1;
    Rightmotor4A = 0;

    // Added PWM to move the motors
    CCP1CON = (0x0c)|((markspaceL&0x03)<<4);
    CCPR1L = markspaceL>>2;
    CCP2CON = (0x0c)|((markspaceR&0x03)<<4);
    CCPR2L = markspaceR>>2;
}

void SpinLeft(void){          //set H bridge to make robot spin to the left
    Leftmotor1A = 1;
    Leftmotor2A = 0;
    Rightmotor3A = 0;
    Rightmotor4A = 1;

    // Added PWM to move the motors
    CCP1CON = (0x0c)|((markspaceL&0x03)<<4);
    CCPR1L = markspaceL>>2;
    CCP2CON = (0x0c)|((markspaceR&0x03)<<4);
    CCPR2L = markspaceR>>2;
}

void Stop(void){               //set H bridge to make robot stop
    Leftmotor1A = 1;
    Leftmotor2A = 1;
    Rightmotor3A = 1;
    Rightmotor4A = 1;

    // Drop PWM to 0
    CCPR1L = 0;
    CCPR2L = 0;
}
