/*
  Using Arduino MEGA ADK
  
  Dvelopedn for TRANS ARTS TOKYO
  Going one way and running comletely random.
*/

#include <Servo.h>
Servo triggerBottom; //bottom
Servo triggerTop; //top

#define POT_PIN A0
#define Z_PIN A1
#define RND_PIN A2
#define BOTTOM_SWITCH_PIN 3
#define TOP_SWITCH_PIN 2
#define SERVO_BOTTOM_PIN 4
#define SERVO_TOP_PIN 6
#define RELAY_PIN 8
#define MOTOR_PIN 9

boolean serial = false;

//for servo
boolean triggerStateTop = false;
byte switchStateBottom = 0;
byte switchStateTop = 0;
byte lastSwitchStateBottom = 0;
byte lastSwitchStateTop = 0;
byte initialTrigger;

//for motor
int speedLeft = 140;
int speedRight = 14;
int speedInit = 24;
int speedStop = 36;
byte movDir = 0;
byte lastMovDir = 0;
/*
0 = stop;
 1 = move left
 2 = move right
 10 = move right slowly (speedInit)
 */

//Z accelaration
const int BUFFER_LENGTH = 20;
int buffer[BUFFER_LENGTH];
int index = 0;
int smoothedZ = 0;

//conditions
byte count = 0;
byte countMax = 18;
unsigned int dly = 800;
unsigned int rndDly = 0;
unsigned int rndDlyMotor = 0;
const int rndDlyRange = 1500;
const int rndDlyMotorRange = 1600;

//timings
unsigned long time = 0;
unsigned int runningTime = 3000;
unsigned int waitingTime = 3000;
unsigned int drawingStartTime = 0;
unsigned int bootStartTime = 0;
unsigned int timeStampTrigger = 0;
unsigned int timeStampMotor = 0;

byte phase = 0;
/*
0 = stop
 1 = initial movement phase
 2 = drawing phase
 */

void setup(){
  Serial.begin(57600);
  pinMode(BOTTOM_SWITCH_PIN, INPUT);
  pinMode(TOP_SWITCH_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(SERVO_BOTTOM_PIN, OUTPUT);
  pinMode(SERVO_TOP_PIN, OUTPUT);
  randomSeed(analogRead(RND_PIN));

  triggerBottom.attach(SERVO_BOTTOM_PIN);
  triggerTop.attach(SERVO_TOP_PIN);
  push("off");

  initialTrigger = random(2);
  sleep();
  time = millis();
  //bootStartTime = time;
  phase = 0;
}

void loop(){

  time = millis();

  //smoothing
  int raw = analogRead(Z_PIN);
  buffer[index] = raw;
  index = (index + 1) % BUFFER_LENGTH;
  smoothedZ = smoothByMeanFilter(buffer);

  if (Serial.available() > 0) {
    Serial.println(smoothedZ);
    if(!serial){
      bootStartTime = time;
      phase = 1;
      serial = true;
    }
  }

  //movement flow
  if(phase == 0) sleep();
  else if(phase == 1) boot();
  else if(phase == 2) drawing();

}


//BASE FUNCTIONS//////////////////////////////////////////////////////

void boot(){
  digitalWrite(RELAY_PIN, HIGH);

  if(time < bootStartTime + runningTime) motorDrive(10);
  else if(bootStartTime + runningTime < time && time < bootStartTime + runningTime + waitingTime) motorDrive(0);
  else if(bootStartTime + runningTime + waitingTime < time) {
    if(initialTrigger) triggerStateTop = true;
    else triggerStateTop = false;

    drawingStartTime = time;
    timeStampMotor = time;
    rndDly = random(rndDlyRange) + 400;
    rndDlyMotor = random(rndDlyRange) + 400;
    motorDrive(0);
    phase = 2;
  }
  Serial.println("boot");
}

void drawing(){

  if(count < countMax){
    //trigger
    if(count < 3){
      if(drawingStartTime+dly*(count+1) < time){
        triggerStateTop = !triggerStateTop;
        timeStampTrigger = time;
        count++;
      }
      if(triggerStateTop) push("top");
      else push("bottom");
    }
    else if(2 < count && count < countMax){
      switchStateBottom = digitalRead(BOTTOM_SWITCH_PIN);
      switchStateTop = digitalRead(TOP_SWITCH_PIN);

      if(switchStateBottom - lastSwitchStateBottom == 1 ||
        switchStateTop - lastSwitchStateTop == 1 ||
        timeStampTrigger + rndDly < time){
        triggerStateTop = !triggerStateTop;
        count++;
        timeStampTrigger = time;
        rndDly = random(rndDlyRange) + 500;

        movDir = random(4);
        if(1 < movDir) movDir = 2;
        motorDrive(movDir);
      }

      if(triggerStateTop) push("top");
      else push("bottom");

      lastSwitchStateBottom = switchStateBottom;
      lastSwitchStateTop = switchStateTop;
    }

    //motor
    if(timeStampMotor + rndDlyMotor < time){
      motorDrive(getMovDir());
      timeStampMotor = time;
      rndDlyMotor = random(rndDlyMotorRange) + 500;
      lastMovDir = movDir;
      Serial.print("movDir = ");
      Serial.print(movDir);
      Serial.print(" ");
      Serial.print("rndDlyMotor = ");
      Serial.println(rndDlyMotor);
    }
  }
  else{
    phase = 0;
  }

}

void sleep(){
  push("off");
  motorDrive(0);
  digitalWrite(RELAY_PIN, LOW);
}

////////////////////////////////////////////////////////////////////////

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
    analogWrite(MOTOR_PIN, speedStop);
    break;
  case 1:
    analogWrite(MOTOR_PIN, speedLeft);
    break;
  case 2:
    analogWrite(MOTOR_PIN, speedRight);
    break;
  case 10:
    analogWrite(MOTOR_PIN, speedInit);
    break;
  default:
    analogWrite(MOTOR_PIN, speedStop);
  }
}


////////////////////////////////////////////////////////////////////////

int smoothByMeanFilter(int b[]) {
  // バッファの値の合計を集計するための変数
  long sum = 0;

  // バッファの値の合計を集計
  for (int i = 0; i < BUFFER_LENGTH; i++) {
    sum += b[i];
  }

  // 平均をフィルタの出力結果として返す
  return (int)(sum / BUFFER_LENGTH);
}

int getPushVal(int n, boolean state){
  //bottom: n = 0
  //top: n = 1
  int n1 = 150;
  int n2 = 30;

  if(n == 0){
    if(state) return n2;
    else return n1;
  }
  else if(n == 1){
    if(state) return n1;
    else return n2;
  }
}


int getMovDir(){
  movDir = random(17);
  if(movDir < 3) movDir = 0;
  else if(2 < movDir && movDir < 7) movDir = 1;
  else if(6 < movDir) movDir = 2;

  //  if(movDir != 2 && lastMovDir == movDir){
  //    if(movDir == 0) movDir = 1;
  //    else movDir = 0;
  //  }

  return movDir;
}







