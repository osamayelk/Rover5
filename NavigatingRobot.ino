#include <PID_v1.h>
#include <PidCompass.h>
#include <QueueList.h>
#include <EnableInterrupt.h>

QueueList <String> distanceQueue;
QueueList <String> angleQueue;

boolean debug = true;
boolean matlab = false;
float x=0;
float y =0;
float theta = 0;
int i;
const int pwmLeft = 5;     // CH3
const int pwmLeft2 = 11;   // CH1

const int pwmRight = 9;    // CH4
const int pwmRight2 = 10;  // CH2


const int dirLeft = 4;     // CH3
const int dirLeft2 = 7;    // CH1

const int dirRight = 12;   // CH4
const int dirRight2 = 13;  // CH2

boolean newAngle = false;
boolean newDistance = false;

long refTime = 0;
long currentTime = 0;

//PID STUFF
double compassReading;
double refReading = 0;
//the value added or subtracted to the motor values
double Output;
//right motor pwm referance value
int rightMotor;
//left motor pwm referance value
int leftMotor;
double kp =  20;
double ki = 40;
double kd = 1.5;
//Creats PID controller PID(&Input, &Output, &Setpoint, kp, ki, kd, Direction) where "input" is the output we are trying to control, "output" the variable that will be adjusted by pid
PID_COMPASS myPID(&compassReading, &Output, &refReading, kp, ki, kd, DIRECT);


//Encoders
double refDistance = 0;
double currentDistance = 0;
double pidDistance;
PID distPID(&currentDistance, &pidDistance, &refDistance, 80, 250, 1.5,DIRECT);
//kd = 1.5

// Mapping Encoders interrupt pins to arduino analog pins.
const byte encoder1 = A0;  // Right  Front  Motor  "J5"
const byte encoder2 = A1;  // Right  Back   Motor  "J3"
const byte encoder3 = 2;   // Left   Front  Motor  "J6"
const byte encoder4 = 3;   // Left   Back   Motor  "J4"

// Variable +ve and -ve edges in the encoders interrupt signals are stored which gives 4x resolution
volatile long ticksR = 0;
volatile long ticksL = 0;
boolean forwardR ;
boolean forwardL ;
// Note that variables that are used in interrupt service routine must be defined as volatile.

//Move backwards
void backwardMotors() {
  digitalWrite(dirLeft, HIGH);
  digitalWrite(dirRight, HIGH);
  digitalWrite(dirLeft2, LOW);
  digitalWrite(dirRight2, LOW);
  forwardR = false;
  forwardL = false;
}
//Move forward
void forwardMotors() {
    digitalWrite(dirLeft, LOW);
    digitalWrite(dirRight, LOW);
    digitalWrite(dirLeft2, HIGH);
    digitalWrite(dirRight2, HIGH);
    forwardR = true;
    forwardL = true;
}

void rightMotorForward() {
  digitalWrite(dirRight, LOW);
  digitalWrite(dirRight2, HIGH);
  forwardR = true;
}

void leftMotorForward() {
  digitalWrite(dirLeft, LOW);
  digitalWrite(dirLeft2, HIGH);
  forwardL = true;
}

void rightMotorReverse() {
  digitalWrite(dirRight, HIGH);
  digitalWrite(dirRight2, LOW);
  forwardR = false;
}

void leftMotorReverse() {
  digitalWrite(dirLeft, HIGH);
  digitalWrite(dirLeft2, LOW);
  forwardL = false;
}

//Brake
void brake() {
    analogWrite(pwmLeft, 0);
    analogWrite(pwmRight, 0);
    analogWrite(pwmLeft2, 0);
    analogWrite(pwmRight2, 0);
}

void rightMotorToggle() {
  if (forwardR) { rightMotorReverse(); forwardR = false;}
  else { rightMotorForward(); forwardR = true;}
}


void rightMotorSpeed(int rightMotor) {
  analogWrite(pwmRight, rightMotor);
  analogWrite(pwmRight2, rightMotor);
}

void leftMotorSpeed(int leftMotor) {
  analogWrite(pwmLeft, leftMotor);
  analogWrite(pwmLeft2, leftMotor);
}


//Encoder functions
// Interrupt Service Routine executed whenever +ve or -ve edge's occures on interrupt pins
void isrR () {
  if(forwardR == true)
    ticksR++;
  else
    ticksR--;
}

void isrL () {
  if(forwardL == true)
    ticksL++;
  else
    ticksL--;
}

unsigned short diameter = 6;
float cpr = 1000 / 3;
double diameterR = 10.3306;
double diameterL = 1.6694;
const double calibrationR = 0.0974; //(PI * diameter) / cpr;
const double calibrationL = 0.0157; //(PI * diameter) / cpr; // in case we wanted to calibrate each motor separatly but right now its irrelevant
//float baseLine = 16.8;
float baseLine = 36.6707;

double displacementR () {  return calibrationR * (ticksR / 2) ; }
double displacementL () { return calibrationL * (ticksL / 2) ; }
double displacementT () { return (displacementR() + displacementL()) / 2 ; }
//double relativeAngle () { return (displacementL() - displacementR()) / baseLine ; }
double relativeAngleR() { return ((displacementR()/(baseLine))*180/PI); }
double relativeAngleL() { return ((displacementL()/(baseLine))*180/PI); }
//double relativeAngleT() { return ((abs(relativeAngleR()) + abs(relativeAngleL())) / 2) ; }
double relativeAngleT() { return ((relativeAngleR() - relativeAngleL())) ; }
void updateXY() {
  theta = theta + relativeAngleT();
  x = x + (displacementT()*cos(relativeAngleT()*(PI/180)));
  y = y + (displacementT()*sin(relativeAngleT()*(PI/180)));
}

void resetTicksR() {
  ticksR = 0;
}

void resetTicksL() {
  ticksL = 0;
}

void resetTicksT(){
  ticksR = 0;
  ticksL = 0;
}

// returns the compass degree
double getDegree() {
  double heading = relativeAngleT();
  currentDistance = displacementT();
  if (debug) {
    // Output
  Serial.print(currentDistance);
  Serial.print(":");
  Serial.print(refDistance);
  //Serial.print(":");
  //Serial.print(norm.ZAxis);
  Serial.print(":");
  Serial.print(heading);
  Serial.print(":");
  Serial.print(refReading);
  //Serial.print(fixedHeadingDegrees);
  //Serial.print(":");
  //Serial.print(smoothHeadingDegrees);
  Serial.println();

  // One loop: ~5ms @ 115200 serial.
  // We need delay ~28ms for allow data rate 30Hz (~33ms)
  //delay(30);
  }

return heading;
}

float getCm(long pulses) {
  float Distance = ((6*M_PI*pulses)/(1000/3));
  return Distance;
}


int count2 = 0;
long t2 = millis();

boolean fixAngle() {
  double err = compassReading - refReading;
//  if (err > 180) err = err - 360;
//  else if (err < -180) err = err + 360;
  if (matlab) {
  currentTime = millis() - refTime;
  Serial.print("1:");
  Serial.print(compassReading);
  Serial.print(":");
  Serial.print(refReading);
  Serial.print(":");
  Serial.print(currentTime);
  Serial.println();

  }
     if (Output > 0) { //&& !(err < 2 && err > -2)) {
      rightMotor = (int)Output;
      leftMotor = (int)Output;
      rightMotorForward();
      leftMotorReverse();
      rightMotorSpeed(rightMotor);
      leftMotorSpeed(leftMotor);
    } else if (Output < 0) { //&& !(err < 2 && err > -2)) {
      //Output = Output - 35;
      rightMotor = -(int)Output;
      leftMotor = -(int)Output;
      rightMotorReverse();
      leftMotorForward();
      rightMotorSpeed(rightMotor);
      leftMotorSpeed(leftMotor);
    } else {
      newAngle = false;
      resetTicksT();
      if (refDistance != 0) {
      newDistance = true;
      }
      brake();
      return true;
    }
    if (abs(err) <= 0.12) {
    count2 = count2 + 1;
    if (count2 == 30) {
      count2 = 0;
      resetTicksT();
      brake();
      if (refDistance != 0) {
      newDistance = true;
      }
      newAngle = false;
      refTime = millis();
      return true;
    }
  } else {
    count2 = 0;
  }
}

int count = 0;
long t1 = millis();
boolean fixDistance() {
  double errDist = refDistance - currentDistance;
    if (matlab) {
  currentTime = millis() - refTime;
  Serial.print("2:");
  Serial.print(currentDistance);
  Serial.print(":");
  Serial.print(refDistance);
  Serial.print(":");
  Serial.print(currentTime);
  Serial.println();

  }
  if (pidDistance > 0) { //&& !(errDist < 3 && errDist > -3)) {
    //pidDistance += 35;
    leftMotorForward();
    rightMotorForward();
    rightMotorSpeed((int)pidDistance);
    leftMotorSpeed((int)pidDistance);
  } else if (pidDistance < 0) { //&& !(errDist < 3 && errDist > -3)) {
    //pidDistance -= 35;
    pidDistance = -pidDistance;
    leftMotorReverse();
    rightMotorReverse();
    rightMotorSpeed((int)pidDistance);
    leftMotorSpeed((int)pidDistance);
  } else {
    brake();
    newDistance = false;
  }
  if (abs(errDist) <= 0.04){// && millis() - t1 > 13) {
    count = count + 1;
    t1 = millis();
    if (count == 10) {
      count = 0;
      brake();
      newDistance = false;
      newAngle = false;
      resetTicksT();
    }
  } else {
    count = 0;
  }

}

void messageParser(){
  while (Serial.available()) {
  if(Serial.available()){
    angleQueue.push(Serial.readStringUntil(','));
    distanceQueue.push(Serial.readStringUntil(':'));
  }
  }
}

void setup(){
  //initiate UART serial communication for PC
  Serial.begin(9600);
  //Specifies whether the PID should be on (Automatic) or off (Manual.) The PID defaults to the off position when created.
  myPID.SetMode(AUTOMATIC);
  //Determines how often the PID algorithm evaluates the value is in mSeconds
  myPID.SetSampleTime(13);
  //The PID controller is designed to vary its output within a given range. By default this range is 0-255: the arduino PWM range.
  myPID.SetOutputLimits(-255,255);

  distPID.SetMode(AUTOMATIC);
  distPID.SetSampleTime(13);
  distPID.SetOutputLimits(-225,225);

  // enabling the pullup resistors of analog pins to be used as pin change interrupt, note! this step isn't needed for external interrupt pins
  pinMode(encoder1, INPUT_PULLUP); // See http://arduino.cc/en/Tutorial/DigitalPins
  pinMode(encoder2, INPUT_PULLUP);
  //pinMode(encoder3, INPUT_PULLUP);
  //pinMode(encoder4, INPUT_PULLUP);
  // enableInterrupt: A library function which takes the pin to enable interrupt on, interrupt service routin to be executed, when interrupt will occure i.e.(falling, rising edges or both"change")
  enableInterrupt (encoder1, isrR, CHANGE);
  enableInterrupt (encoder2, isrR, CHANGE);
  enableInterrupt (encoder3, isrL, CHANGE);
  enableInterrupt (encoder4, isrL, CHANGE);
  // Motors pin setup
  pinMode(dirLeft, OUTPUT);
  pinMode(dirRight, OUTPUT);
  pinMode(dirLeft2, OUTPUT);
  pinMode(dirRight2, OUTPUT);
}

void loop() {
  messageParser();
  if (!distanceQueue.isEmpty() && !angleQueue.isEmpty() && !newAngle && !newDistance) {
    brake();
    refReading = angleQueue.pop().toInt();
    refDistance = distanceQueue.pop().toInt();
    Serial.print("You sent ");
    Serial.print(refReading);
    Serial.print(" and ");
    Serial.println(refDistance);
    if (refReading != 0) {
    newAngle = true;
    newDistance = false;
    } else if (refDistance != 0) {
      newAngle = false;
      newDistance = true;
    } else {
      newAngle = false;
      newDistance = false;
    }
    resetTicksT();
    refTime = millis();
  }
  compassReading = getDegree();
  if (newAngle && myPID.Compute()) {
    fixAngle();
  } else if(newDistance && distPID.Compute()) {
    fixDistance();
  }
}
