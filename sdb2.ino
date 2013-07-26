/*
  Developed for JIZZED IN MY PANTS @ 3331
  
  Using Arduino MEGA ADK.
  Rotate washer head could be painting whole space.
*/


#include <Servo.h>

// pin asign ///////////////////////////////
#define RND A0
#define MOTOR_DRIVE 3
#define SW_SWING_TOP 2
#define SW_SWING_BTM 4
#define SRV_TRIGGER_BTM 5
#define SRV_TRIGGER_TOP 6
#define MOTOR_GEAR1 7
#define MOTOR_GEAR2 8
#define MOTOR_GEAR_PWM 9
#define SW_GEAR1 10
#define SW_GEAR2 11
#define SENSOR_DISTANCE_L 22
#define SENSOR_DISTANCE_R 24
#define RELAY 26

// servo ///////////////////////////////
Servo triggerBottom; //bottom
Servo triggerTop; //top

boolean triggerStateTop = false;
byte switchStateBottom = 0;
byte switchStateTop = 0;
byte lastSwitchStateBottom = 0;
byte lastSwitchStateTop = 0;
byte initialTrigger;

// drive motor ///////////////////////////////
int speedLeft = 55;
int speedRight = 19;
int speedInit = 21; //Initial movement speed (slowly)
int speedStop = 36;

// gun & direction ///////////////////////////////
/* dickDir
 0   = front
 90  = left side
 180 = back
 270 = right side
 */
unsigned int dickDir = 0;
unsigned int distSensorL;
unsigned int distSensorR;
int pulse_w; //for getDistance
unsigned int phaseDelay = 5000;
const unsigned int minDist = 100; //1m


// conditions ///////////////////////////////
unsigned int rndDly = 0;
unsigned int rndDlyMotor = 0;
const int rndDlyRange = 700;
const int rndDlyMotorRange = 1600;

const int kickbackDelay = 1000;

// timings ///////////////////////////////
unsigned int runningTime = 3000;
unsigned int waitingTime = 3000;
unsigned int bootStartTime = 0;
unsigned int drawingStartTime = 0;
unsigned int timeStampTrigger = 0;
unsigned int timeStampMotor = 0;

boolean serial = false;
byte phase = 0;
/*
0 = stop
 1 = initial movement phase
 2 = drawing phase
 */

// for test///////////////////////
boolean test = false;
byte testCount = 0;
//////////////////////////////////


void setup(){
  Serial.begin(57600);

  pinMode(SW_SWING_TOP, INPUT);
  pinMode(SW_SWING_BTM, INPUT);
  pinMode(MOTOR_DRIVE, OUTPUT);
  pinMode(SRV_TRIGGER_BTM, OUTPUT);
  pinMode(SRV_TRIGGER_TOP, OUTPUT);
  pinMode(MOTOR_GEAR1, OUTPUT);
  pinMode(MOTOR_GEAR2, OUTPUT);
  pinMode(MOTOR_GEAR_PWM, OUTPUT);
  pinMode(SW_GEAR1, INPUT);
  pinMode(SW_GEAR2, INPUT);
  pinMode(SENSOR_DISTANCE_L, INPUT);
  pinMode(SENSOR_DISTANCE_R, INPUT);
  pinMode(RELAY, OUTPUT);

  randomSeed(analogRead(RND));

  triggerBottom.attach(SRV_TRIGGER_BTM);
  triggerTop.attach(SRV_TRIGGER_TOP);
  push("off");

  sleep();
  //bootStartTime = time;
  phase = 1;
}

void loop(){

  //always watching distance L & R
  distSensorL = getDistance(SENSOR_DISTANCE_L);
  distSensorR = getDistance(SENSOR_DISTANCE_R);

  if(!test){
    //movement flow
    if(phase == 0) sleep();
    else if(phase == 1) wakeup();
    else if(phase == 2) jizz();
  }
  else{
    //test shoot for gun
    rotate(1);
    if(testCount < 5){
      delay(1100);
      push("top");
      delay(1100);
      push("bottom");
      testCount++;
    }
    else{
      rotate(0);
      push("off");
    }
  }
}


//BASE FUNCTIONS//////////////////////////////////////////////////////

void wakeup(){
  digitalWrite(RELAY, HIGH);

  //start initial running
  if(millis() < bootStartTime + runningTime) motorDrive(10);

  //stop and waiting for draw
  else if(bootStartTime + runningTime < millis() &&
          millis() < bootStartTime + runningTime + waitingTime) motorDrive(0);

  else if(bootStartTime + runningTime + waitingTime < millis()) {
    initialTrigger = random(2);
    if(initialTrigger) triggerStateTop = true;
    else triggerStateTop = false;

    timeStampMotor = millis();
    timeStampTrigger = millis();
    rndDly = random(rndDlyRange) + 400;
    rndDlyMotor = random(rndDlyRange) + 400;
    motorDrive(2);
    phase = 2;
  }
  Serial.println("boot");
}


void jizz(){

  if(dickDir == 0 || dickDir == 180){
    rotate(0);

    if(dickDir == 0){
      if(distSensorL < minDist){
        motorDrive(1);
        delay(kickbackDelay);
        pause();
        delay(phaseDelay);
        dickDir = 90;
      }
      else {
        motorDrive(2);
        jizzing();
      }
    }
    else if(dickDir == 180){
      if(distSensorR < minDist){
        motorDrive(2);
        delay(kickbackDelay);
        pause();
        delay(phaseDelay);
        dickDir = 270;
      }
      else {
        motorDrive(1);
        jizzing();
      }
    }
  }

  else if(dickDir == 90 || dickDir == 270){
    motorDrive(0);

    if(dickDir == 90){
      if(digitalRead(SW_GEAR2) == 1){
        pause();
        delay(phaseDelay);
        dickDir = 180;
      }
      else {
        rotate(1);
        jizzing();
      }
    }
    else if(dickDir == 270){
      if(digitalRead(SW_GEAR1) == 1){
        pause();
        delay(phaseDelay);
        dickDir = 360;
      }
      else {
        rotate(1);
        jizzing();
      }
    }
  }
  else if(dickDir == 360) phase = 0;
}



void sleep(){
  push("off");
  motorDrive(0);
  digitalWrite(RELAY, LOW);
}

////////////////////////////////////////////////////////////////////////

void jizzing(){
  switchStateBottom = digitalRead(SW_SWING_BTM);
  switchStateTop = digitalRead(SW_SWING_TOP);

  if(switchStateBottom != lastSwitchStateBottom ||
      switchStateTop != lastSwitchStateTop ||
      timeStampTrigger + rndDly < millis()){

    triggerStateTop = !triggerStateTop;
    timeStampTrigger = millis();
    rndDly = random(rndDlyRange) + 500;
  }
  if(triggerStateTop) push("top");
  else push("bottom");

  lastSwitchStateBottom = switchStateBottom;
  lastSwitchStateTop = switchStateTop;
}

void pause(){
  push("off");
  rotate(0);
  motorDrive(0);
}

void push(String state){
  int n = 0;
  if(state == "off") n = 0;
  else if(state == "bottom") n = 1;
  else if(state == "top") n = 2;
  else if(state == "both") n = 3;
  else n = 0;

  switch(n){
  case 0:
    triggerBottom.write(getPushVal(0, false));
    triggerTop.write(getPushVal(1, false));
    break;
  case 1:
    triggerBottom.write(getPushVal(0, true));
    triggerTop.write(getPushVal(1, false));
    break;
  case 2:
    triggerBottom.write(getPushVal(0, false));
    triggerTop.write(getPushVal(1, true));
    break;
  case 3:
    triggerBottom.write(getPushVal(0, true));
    triggerTop.write(getPushVal(1, true));
    break;
  default:
    triggerBottom.write(getPushVal(0, false));
    triggerTop.write(getPushVal(1, false));
  }
}

void motorDrive(int n){
  /*
  0 = stop;
   1 = move left
   2 = move right
   10 = move right slowly
   */
  switch(n){
  case 0:
    analogWrite(MOTOR_DRIVE, speedStop);
    break;
  case 1:
    analogWrite(MOTOR_DRIVE, speedLeft);
    break;
  case 2:
    analogWrite(MOTOR_DRIVE, speedRight);
    break;
  case 10:
    analogWrite(MOTOR_DRIVE, speedInit);
    break;
  default:
    analogWrite(MOTOR_DRIVE, speedStop);
  }
}

void rotate(byte state){
  if(state == 1){
    analogWrite(MOTOR_GEAR_PWM, 255);  
    digitalWrite(MOTOR_GEAR1, HIGH);
    digitalWrite(MOTOR_GEAR2, LOW);
  }
  else if(state == 2){
    analogWrite(MOTOR_GEAR_PWM, 255);  
    digitalWrite(MOTOR_GEAR1, LOW);
    digitalWrite(MOTOR_GEAR2, HIGH);
  }
  else if(state == 0){
    analogWrite(MOTOR_GEAR_PWM, 0);  
    digitalWrite(MOTOR_GEAR1, LOW);
    digitalWrite(MOTOR_GEAR2, LOW);
  }
}

///////////////////////////////////////////////////////////

int getPushVal(int n, boolean state){
  //bottom: n = 0
  //top: n = 1
  int n1 = 150;
  int n2 = 30;

  if(n == 0){
    if(state) return 30;
    else return 150;
  }
  else if(n == 1){
    if(state) return 150;
    else return 30;
  }
}

int getDistance(byte sensor_pin){
  pinMode(sensor_pin, OUTPUT);
  digitalWrite(sensor_pin, LOW);

  digitalWrite(sensor_pin, HIGH);
  delayMicroseconds(2);
  digitalWrite(sensor_pin, LOW);

  delayMicroseconds(5);
  pinMode(sensor_pin, INPUT);

  pulse_w = pulseIn(sensor_pin, HIGH);
  int distance = pulse_w*0.0342/2;
  return distance;
}









