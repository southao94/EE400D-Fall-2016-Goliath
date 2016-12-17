void NewControlAlgorithm(){
  //Timer3.attachInterrupt(play);

//Take out the oldest data from the left running averager
leftTotal = leftTotal-leftReadings[readLeftIndex];

//Begin I2C Communication using Wire.h Library for Left Sensor
Wire.beginTransmission(leftSensorAddress);
Wire.write(rangeCommand);
Wire.requestFrom(leftSensorAddress,2);
while(!Wire.available()){}
//discard the MSB that the sensors sends back
discard1=Wire.read();
//store the distance into the array
leftReadings[readLeftIndex] = Wire.read();
Wire.endTransmission();

//Add the new distance data to the sum of the 10 point averager
leftTotal = leftTotal+leftReadings[readLeftIndex];
//Find the new average for the running 10 point averager
leftSensorAverage = leftTotal/leftNumReadings;
//advance to the next position in the array
readLeftIndex = readLeftIndex+1;

//if we're at the end of the array start over at the beginning again
if (readLeftIndex >= leftNumReadings) {
  readLeftIndex = 0;
}

//There must be a 100ms delay between readings for the sensors
delay(100);

//Take out the oldest data from the right running averager
rightTotal = rightTotal-rightReadings[readRightIndex];

//Begin I2C Communication using Wire.h Library for Right Sensor
Wire.beginTransmission(rightSensorAddress);
Wire.write(rangeCommand);
Wire.requestFrom(rightSensorAddress,2);
while(!Wire.available()){}
//discard the MSB that the sensors sends back
discard2=Wire.read();
//store the distance into the array
rightReadings[readRightIndex] = Wire.read();
Wire.endTransmission();

//Add the new distance data to the sum of the 10 point averager
rightTotal = rightTotal+rightReadings[readRightIndex];
//Find the new average for the running 10 point averager
rightSensorAverage = rightTotal/rightNumReadings;
//advance to the next position in the array
readRightIndex = readRightIndex+1;

//if we're at the end of the array start over at the beginning again
if (readRightIndex >= rightNumReadings) {
  readRightIndex = 0;
}

//store the sensors values into variables a and b
double a = leftSensorAverage;
double b = rightSensorAverage;
//c represents the half the distance between both sensors
double c = 9.375;

//Fix the Distances if There is an Error in Creating a Triangle
if (a>b) {
  b = adjust(a,b);}
  else {
  a = adjust(b,a);}

//determining the direction the object is at
double horizontalDirection = a-b;

//using triangulation to find distance d and theta angle
//please refer to Control Algorithm Using Triangulation
//on Goliath Blog Posts for More Information
double angleD = acos((sq(a)-sq(b)+(sq(2*c)))/(4*a*c));
double d = sqrt(sq(a)+sq(c)-(2*a*c*cos(angleD)));
double theta = asin(a/d*sin(angleD));

//Initialize the set point of the vertical distance to be 45cm
int dT = 45;
//Find the Vertical Error
int verticalError = dT - d;

//Set an initial speed where motors are barely going forward
int minVerticalSpeed = 140;

/*///////////////////////////////////////////////////////
 */////////PI Contrller For Verical Control////////////// 
 *//////////////////////////////////////////////////////
//Setting Up the P Controller
int kpVertical = 0.7;
float PVertical = abs(kpVertical*verticalError);

//Setting Up the I Controller
int kiVertical = 1;
//If the vertcal distance d is closer to the set point make
//integral term equal to 0 otherwise increase the integral term
if (abs(verticalError)> 5){
  verticalIntegral = verticalIntegral+abs(verticalError);
}
else {
  verticalIntegral = 0;
}
int IVertical = abs(verticalIntegral*kiVertical);
//Set the Vertical Speed to the sum of the output from the PI Controller
float verticalSpeed = minVerticalSpeed+PVertical+IVertical;


//Make the horizontal set point equal to 1.5708
double horizontalError = 1.5708-theta;
//Set an initial speed to differentiate between the motors speed
int minHorizontalSpeed = 20;

/*//////////////////////////////////////////////////////
 *////////PI Contrller For Horizontal Control///////////
 *//////////////////////////////////////////////////////
//Setting Up the P Controller
int kpHorizontal = 50;
float PHorizontal = abs(kpHorizontal*horizontalError);

//Setting Up the I Controller
int kiHorizontal = 10;
//If the horizontal angle is closer to the set point make
//integral term equal to 0 otherwise increase the integral term
if (abs(horizontalError)> .4){
  horizontalIntegral = horizontalIntegral+abs(horizontalError);
}
else {
  horizontalIntegral = 0;
}
int IHorizontal = abs(horizontalIntegral*kiHorizontal);
//Set the Horizontal Speed to the sum of the output from the PI Controller
float horizontalSpeed = minHorizontalSpeed+PHorizontal+IHorizontal;

//If the Object is further than the set point and it's located
//to the right make the left motor run faster than the right motor
//to make a left turn going forward
if ((verticalError<0)&&(horizontalDirection>0)){
   speedLeft = verticalSpeed + horizontalSpeed;
   speedRight = verticalSpeed - horizontalSpeed;
  }
  //If the object is further than the set point and it's located
  //to the left make the right motor run faster than the left motor
  //to make a right turn going forward
  else if ((verticalError<0)&&(horizontalDirection<0)) {
   speedLeft = verticalSpeed - horizontalSpeed;
   speedRight = verticalSpeed + horizontalSpeed;
  }  
  //If the object is further than the set point and it's located
  //in the middle make both motors run faster going forward
  else if ((verticalError<0)&&(horizontalDirection==0)) {
   speedLeft = verticalSpeed+horizontalSpeed;
   speedRight = verticalSpeed+horizontalSpeed;
  }
  //If the object is closer than the set point and it's located
  //to the right make the left motor slower than the right motor
  //to make a right turn going backwards
  else if ((verticalError>0)&&(horizontalDirection>0)) {
   speedLeft = verticalSpeed - horizontalSpeed;
   speedRight = verticalSpeed + horizontalSpeed;
  }
  //if the object is closer than the set point and it's located
  //to the left make the right motor slower than the left motor
  //to make a left turn going backwards
  else if ((verticalError>0)&&(horizontalDirection<0)) {
   speedLeft = verticalSpeed + horizontalSpeed;
   speedRight = verticalSpeed - horizontalSpeed;
  }
  //if the object is closer than the set point and it's located
  //in the middle make both sensors run faster going backwards
  else if ((verticalError>0)&&(horizontalDirection==0)) {
    speedLeft = verticalSpeed + horizontalSpeed;
    speedRight = verticalSpeed + horizontalSpeed;
  }
  //if the object reaches the set point, don't move the motors
  else if (verticalError==0) {
    speedLeft = 0;
    speedRight = 0;
  }

//control the speed of the PWM to be within 0 to 255
if (speedLeft >255) { 
    speedLeft = 255;
}
else if (speedLeft < 0) {
    speedLeft = 0;
}

if (speedRight >255) { 
    speedRight = 255;
}
else if (speedRight < 0) {
    speedRight = 0;
}

//If the object is on the right and it is below
//the range of the set point adjust the brightness
//of the two right red LEDs
if ((a>b)&&(d<42)) {
  brightness = map(d,20,42,10,500);
   //LED1 Furthest Right LED
  pwmSensors.setPWM(12,0,brightness);
  pwmSensors.setPWM(13,0,0);
  pwmSensors.setPWM(11,0,0);
  //LED2
  pwmSensors.setPWM(9,0,brightness);
  pwmSensors.setPWM(10,0,0);
  pwmSensors.setPWM(8,0,0);
  //LED3
  pwmSensors.setPWM(6,0,0);
  pwmSensors.setPWM(7,0,0);
  pwmSensors.setPWM(5,0,0);
  //LED4
  pwmSensors.setPWM(3,0,0);
  pwmSensors.setPWM(4,0,0);
  pwmSensors.setPWM(2,0,0);
  //LED5 Furthest Left LED
  pwmSensors.setPWM(0,0,0);
  pwmSensors.setPWM(1,0,0);
  pwmSensors.setPWM(14,0,0);
}
//If the object is on the right and it is within
//the range of the set point adjust the brightness
//of the two right green LEDs
else if ((a>b)&&((d>=42) && (d<=48))) {
brightness = map(d,42,48,10,500);
   //LED1 Furthest Right LED
  pwmSensors.setPWM(12,0,0);
  pwmSensors.setPWM(13,0,brightness);
  pwmSensors.setPWM(11,0,0);
  //LED2
  pwmSensors.setPWM(9,0,0);
  pwmSensors.setPWM(10,0,brightness);
  pwmSensors.setPWM(8,0,0);
  //LED3
  pwmSensors.setPWM(6,0,0);
  pwmSensors.setPWM(7,0,0);
  pwmSensors.setPWM(5,0,0);
  //LED4
  pwmSensors.setPWM(3,0,0);
  pwmSensors.setPWM(4,0,0);
  pwmSensors.setPWM(2,0,0);
  //LED5 Furthest Left LED
  pwmSensors.setPWM(0,0,0);
  pwmSensors.setPWM(1,0,0);
  pwmSensors.setPWM(14,0,0);
}
//If the object is on the right and it is above the
//range of the set point adjust the brightness of
//the two right blue LEDs
else if ((a>b)&&(d>48)) {
  brightness = map(d,48,255,10,4095);
   //LED1 Furthest Right LED
  pwmSensors.setPWM(12,0,0);
  pwmSensors.setPWM(13,0,0);
  pwmSensors.setPWM(11,0,brightness);
  //LED2
  pwmSensors.setPWM(9,0,0);
  pwmSensors.setPWM(10,0,0);
  pwmSensors.setPWM(8,0,brightness);
  //LED3
  pwmSensors.setPWM(6,0,0);
  pwmSensors.setPWM(7,0,0);
  pwmSensors.setPWM(5,0,0);
  //LED4
  pwmSensors.setPWM(3,0,0);
  pwmSensors.setPWM(4,0,0);
  pwmSensors.setPWM(2,0,0);
  //LED5 Furthest Left LED
  pwmSensors.setPWM(0,0,0);
  pwmSensors.setPWM(1,0,0);
  pwmSensors.setPWM(14,0,0);
}
//If the object is on the left and it is below
//the range of the set point adjust the brightness
//of the two left red LEDs
else if ((a<b)&&(d<42)) {
brightness = map(d,20,42,10,500);
   //LED1 Furthest Right LED
  pwmSensors.setPWM(12,0,0);
  pwmSensors.setPWM(13,0,0);
  pwmSensors.setPWM(11,0,0);
  //LED2
  pwmSensors.setPWM(9,0,0);
  pwmSensors.setPWM(10,0,0);
  pwmSensors.setPWM(8,0,0);
  //LED3
  pwmSensors.setPWM(6,0,0);
  pwmSensors.setPWM(7,0,0);
  pwmSensors.setPWM(5,0,0);
  //LED4
  pwmSensors.setPWM(3,0,brightness);
  pwmSensors.setPWM(4,0,0);
  pwmSensors.setPWM(2,0,0);
  //LED5 Furthest Left LED
  pwmSensors.setPWM(0,0,brightness);
  pwmSensors.setPWM(1,0,0);
  pwmSensors.setPWM(14,0,0);
}
//If the object is on the left and it is within
//the range of the set point adjust the brightness
//of the two left green LEDs
else if ((a<b)&&((d>=42) && (d<=48))){
brightness = map(d,42,48,10,500);
   //LED1 Furthest Right LED
  pwmSensors.setPWM(12,0,0);
  pwmSensors.setPWM(13,0,0);
  pwmSensors.setPWM(11,0,0);
  //LED2
  pwmSensors.setPWM(9,0,0);
  pwmSensors.setPWM(10,0,0);
  pwmSensors.setPWM(8,0,0);
  //LED3
  pwmSensors.setPWM(6,0,0);
  pwmSensors.setPWM(7,0,0);
  pwmSensors.setPWM(5,0,0);
  //LED4
  pwmSensors.setPWM(3,0,0);
  pwmSensors.setPWM(4,0,brightness);
  pwmSensors.setPWM(2,0,0);
  //LED5 Furthest Left LED
  pwmSensors.setPWM(0,0,0);
  pwmSensors.setPWM(1,0,brightness);
  pwmSensors.setPWM(14,0,0);
}
//If the object is on the left and it is above the
//range of the set point adjust the brightness of
//the two left blue LEDs
else if ((a<b)&&(d>48)){
brightness = map(d,48,255,10,4095);
   //LED1 Furthest Right LED
  pwmSensors.setPWM(12,0,0);
  pwmSensors.setPWM(13,0,0);
  pwmSensors.setPWM(11,0,0);
  //LED2
  pwmSensors.setPWM(9,0,0);
  pwmSensors.setPWM(10,0,0);
  pwmSensors.setPWM(8,0,0);
  //LED3
  pwmSensors.setPWM(6,0,0);
  pwmSensors.setPWM(7,0,0);
  pwmSensors.setPWM(5,0,0);
  //LED4
  pwmSensors.setPWM(3,0,0);
  pwmSensors.setPWM(4,0,0);
  pwmSensors.setPWM(2,0,brightness);
  //LED5 Furthest Left LED
  pwmSensors.setPWM(0,0,0);
  pwmSensors.setPWM(1,0,0);
  pwmSensors.setPWM(14,0,brightness);
}
//If the object is in the middle and it is below the
//range of the set point adjust the brightness of
//the three middle red LEDs
else if ((a==b)&&(d<42)) {
brightness = map(d,20,42,10,500);
   //LED1 Furthest Right LED
  pwmSensors.setPWM(12,0,0);
  pwmSensors.setPWM(13,0,0);
  pwmSensors.setPWM(11,0,0);
  //LED2
  pwmSensors.setPWM(9,0,brightness);
  pwmSensors.setPWM(10,0,0);
  pwmSensors.setPWM(8,0,0);
  //LED3
  pwmSensors.setPWM(6,0,brightness);
  pwmSensors.setPWM(7,0,0);
  pwmSensors.setPWM(5,0,0);
  //LED4
  pwmSensors.setPWM(3,0,brightness);
  pwmSensors.setPWM(4,0,0);
  pwmSensors.setPWM(2,0,0);
  //LED5 Furthest Left LED
  pwmSensors.setPWM(0,0,0);
  pwmSensors.setPWM(1,0,0);
  pwmSensors.setPWM(14,0,0);
}
//If the object is in the middle and it is within the
//range of the set point adjust the brightness of
//the three middle green LEDs
else if ((a==b)&&((d>=42) && (d<=48))){
brightness = map(d,42,48,10,500);
   //LED1 Furthest Right LED
  pwmSensors.setPWM(12,0,0);
  pwmSensors.setPWM(13,0,0);
  pwmSensors.setPWM(11,0,0);
  //LED2
  pwmSensors.setPWM(9,0,0);
  pwmSensors.setPWM(10,0,brightness);
  pwmSensors.setPWM(8,0,0);
  //LED3
  pwmSensors.setPWM(6,0,0);
  pwmSensors.setPWM(7,0,brightness);
  pwmSensors.setPWM(5,0,0);
  //LED4
  pwmSensors.setPWM(3,0,0);
  pwmSensors.setPWM(4,0,brightness);
  pwmSensors.setPWM(2,0,0);
  //LED5 Furthest Left LED
  pwmSensors.setPWM(0,0,0);
  pwmSensors.setPWM(1,0,0);
  pwmSensors.setPWM(14,0,0);
}
//If the object is in the middle and it is above the
//range of the set point adjust the brightness of
//the three middle blue LEDs
else if ((a==b)&&(d>48)){
brightness = map(d,48,255,10,4095);
   //LED1 Furthest Right LED
  pwmSensors.setPWM(12,0,0);
  pwmSensors.setPWM(13,0,0);
  pwmSensors.setPWM(11,0,0);
  //LED2
  pwmSensors.setPWM(9,0,0);
  pwmSensors.setPWM(10,0,0);
  pwmSensors.setPWM(8,0,brightness);
  //LED3
  pwmSensors.setPWM(6,0,0);
  pwmSensors.setPWM(7,0,0);
  pwmSensors.setPWM(5,0,brightness);
  //LED4
  pwmSensors.setPWM(3,0,0);
  pwmSensors.setPWM(4,0,0);
  pwmSensors.setPWM(2,0,brightness);
  //LED5 Furthest Left LED
  pwmSensors.setPWM(0,0,0);
  pwmSensors.setPWM(1,0,0);
  pwmSensors.setPWM(14,0,0);
}
//If the difference between both LEDs is 
//greater than 15, we are lost so turn on
//all red LEDs
if (abs(a-b)>15){
   //LED1 Furthest Right LED
  pwmSensors.setPWM(12,0,500);
  pwmSensors.setPWM(13,0,0);
  pwmSensors.setPWM(11,0,0);
  //LED2
  pwmSensors.setPWM(9,0,500);
  pwmSensors.setPWM(10,0,0);
  pwmSensors.setPWM(8,0,0);
  //LED3
  pwmSensors.setPWM(6,0,500);
  pwmSensors.setPWM(7,0,0);
  pwmSensors.setPWM(5,0,0);
  //LED4
  pwmSensors.setPWM(3,0,500);
  pwmSensors.setPWM(4,0,0);
  pwmSensors.setPWM(2,0,0);
  //LED5 Furthest Left LED
  pwmSensors.setPWM(0,0,500);
  pwmSensors.setPWM(1,0,0);
  pwmSensors.setPWM(14,0,0);
}

//If the object is too close
//from set point go backwards
if (verticalError>0){
  motorA.go(2,speedLeft);
  motorB.go(2,speedRight);
}
//If the object is too far 
//from set point go forwards
else if (verticalError<0) {
  motorA.go(1,speedLeft);
  motorB.go(1,speedRight);
}
//if the object is within range
//of set point stop moving
else if (verticalError==0) {
  motorA.go(1,0);
  motorB.go(1,0);
}

//There must be a 100ms delay between 
//ranging readings from each sensor
delay(50);

//Use the Serial.print to troubleshoot
//any errors caused by the control algorithm
//Serial.print("Left Distance: ");
//Serial.print(leftSensorAverage);
//Serial.print("      ");
//Serial.print("Right Distance: ");
//Serial.println(rightSensorAverage);
//Serial.print("a=");
//Serial.print(a);
//Serial.print("    ");
//Serial.print("b=");
//Serial.println(b);
//Serial.print("Middle Distance: ");
//Serial.print(d);
//Serial. print("     ");
//Serial.print("Theta Angle: ");
//Serial.println(theta);
//Serial.print("Vertical Speed: ");
//Serial.print(verticalSpeed);
//Serial.print("     ");
//Serial.print("Horizontal Speed: ");
//Serial.println(horizontalSpeed);
//Serial.print("Vertical Error is: ");
//Serial.print(verticalError);
//Serial.print("      ");
//Serial.print("Horizontal Error is: ");
//Serial.println(horizontalError);
//Serial.print("Left Speed: ");
//Serial.print(speedLeft);
//Serial.print("      ");
//Serial.print("Right Speed: ");
//Serial.println(speedRight);
//Serial.println("    ");
}

