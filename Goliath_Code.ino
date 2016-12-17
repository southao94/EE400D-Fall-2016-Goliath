//Include the Timer1 and Timer 3 Libraries for Interrupt
//#include <TimerThree.h>
//#include <TimerOne.h>
#include <Configure.h>
//math.h file does trig identity
#include <math.h>
//declare the sensor's I2C address
#define leftSensorAddress 0x72
#define rightSensorAddress 0x71
//Use the Adafruit Library to control PWM of LEDs
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwmSensors = Adafruit_PWMServoDriver();
//pitches are the frequencies used for the speaker
#include "pitches.h"

//initialize the Integral terms for the PID Controllers
float verticalIntegral, horizontalIntegral;
float speedLeft,speedRight;
//The sensors return 2 bytes, the higher byte is discarded
int LeftSensorDistance,discard1, RightSensorDistance,discard2;
//initialize the range command to start ranging the sensors
byte rangeCommand = 0x51;

const int leftNumReadings = 6;      //initializes the size of the moving averager to 6
const int rightNumReadings = 6;
int leftReadings[leftNumReadings] = {0};  //initializes array of 6 data points to all be 0
int rightReadings[leftNumReadings] = {0}; 

//initializes all data to create moving averager to be 0
int readLeftIndex = 0;
int readRightIndex = 0;
int leftTotal = 0;
int rightTotal = 0;
int leftSensorAverage = 0;
int rightSensorAverage = 0;

//initialize the brightness variable to set the intensity of the LEDs
double brightness = 0;

//thisNote is used as a coutner to play each note
int thisNote;

//the notes for the tank noise
int melody[] = {
  NOTE_E3, NOTE_A2, NOTE_B2, NOTE_F1,
  NOTE_CS1, NOTE_E1, NOTE_D1, NOTE_AS3,
  NOTE_E3, NOTE_E2, NOTE_B2, NOTE_E3,
  NOTE_F2, NOTE_AS1, NOTE_D3, NOTE_B3,
  NOTE_CS2, NOTE_E3
};

 
// include C:\Program Files (x86)\Arduino 1_6_5\hardware\arduino\avr\libraries
#include <Robot3DoTBoard.h>     // instantiated as Robot3DoT at end of class header
#include <EEPROM.h>
#include <Wire.h>               // I2C support

Robot3DoTBoard Robot3DoT;       // instantiated as Robot3DoT at end of class header

#define MODE       0x40
const uint8_t CMD_LIST_SIZE = 2;   // we are adding 3 commands (MOVE, BLINK, SERVO) 
int mode = 0; //starts initially in RC MODE
int x;   //Right Motor PWM in RC
int y;   //Left Motor PWM in RC
Motor motorA;   //Right Motor
Motor motorB;   //Left Motor

void modeHandler (uint8_t cmd, uint8_t param[], uint8_t n);
void moveHandler (uint8_t cmd, uint8_t param[], uint8_t n);

Robot3DoTBoard::cmdFunc_t onCommand[CMD_LIST_SIZE] = {{MOVE,moveHandler}, {MODE,modeHandler}}; 

Packet battery3dot(BATTERY_ID);  // initialize the packet properties to default values

void setup()
{
  Serial.begin(9600);              
  Robot3DoT.begin();
  Robot3DoT.setOnCommand(onCommand, CMD_LIST_SIZE);    
  motorA.begin(5,10,9); //begin(control_pin1,control_pin2,pwmPin) --> set up  motor control pins
  motorB.begin(19,20,6); 
  
  pwmSensors.begin();       //begin using PWM for LED control
  pwmSensors.setPWMFreq(500); //Set the frequency so there is no blinking from LEDs
 // Timer3.initialize(8000); //Initializes when the timer will interupt in microseconds

  battery3dot.setAccuracy(1);          // change sensor accuracy from +/-2 DN to +/-1 DN    (-- this line is optional --)
  battery3dot.setSamplePeriod(500);    // change sample period from 1 second to 0.5 seconds (-- this line is optional --)
  
  //Play Tank Noise in the Setup
  play();
  play();
  play();
  play();
  play();
  play();
  play();
  play();
  play();
  play();
  play();
}

void loop()
{
  Robot3DoT.loop();

    uint16_t battery_reading = (uint16_t) 35;  // read 8-bit Output Compare Register Timer 4D and cast to 16-bit signed word
    battery3dot.sendSensor(battery_reading);

  if (mode == 1)  //mode selections is autonomous
  {
   NewControlAlgorithm();
  }
}

/*///////////////////////////////////////////////////////
 * ////////////////  Mode Handler  //////////////////////
 *///////////////////////////////////////////////////////
void modeHandler (uint8_t cmd, uint8_t param[], uint8_t n)
{
  mode = param[0];
}  // moveHandler

/*///////////////////////////////////////////////////////
 * ////////////////  Move Handler  //////////////////////
 *///////////////////////////////////////////////////////
void moveHandler (uint8_t cmd, uint8_t param[], uint8_t n)
{
  if (mode == 0)
  {
    Serial.write(cmd);             // move command = 0x01
    Serial.write(n);               // number of param = 4
    for (int i=0;i<n;i++)          // param = 01 80 01 80
    {
      Serial.write (param[i]);
    }  

      x = 150+param[1]/2.42857;   //param[1] speed for motor 1
      y = 150+param[3]/2.42857;   //param[3] speed for motor 2
      motorA.go(param[0],x);    //param[0] direction of motor 1
      motorB.go(param[2],y);    //param[2] directrion of motor 2

    //Make all of the LEDs turn on to
    //indicate we are in RC Mode
    //LED1 Furthest Right LED
    pwmSensors.setPWM(12,0,500);
    pwmSensors.setPWM(13,0,500);
    pwmSensors.setPWM(11,0,500);
    //LED2
    pwmSensors.setPWM(9,0,500);
    pwmSensors.setPWM(10,0,500);
    pwmSensors.setPWM(8,0,500);
    //LED3
    pwmSensors.setPWM(6,0,500);
    pwmSensors.setPWM(7,0,500);
    pwmSensors.setPWM(5,0,500);
    //LED4
    pwmSensors.setPWM(3,0,500);
    pwmSensors.setPWM(4,0,500);
    pwmSensors.setPWM(2,0,500);
    //LED5 Furthest Left LED
    pwmSensors.setPWM(0,0,500);
    pwmSensors.setPWM(1,0,500);
    pwmSensors.setPWM(14,0,500);
    }
    
}  



