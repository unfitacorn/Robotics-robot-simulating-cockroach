#include <Servo.h>
#include <NewPing.h>
#include <SharpIR.h>

Servo leftServo;
Servo rightServo;
Servo turret;

NewPing BLeftPing(8, 8, 20);
NewPing FLeftPing(9, 9, 20);
NewPing FRightPing(10, 10, 20);
NewPing BRightPing(11, 11, 20);

SharpIR IRTop(A3, 25, 93, 20150);

#define leftStopValue 91
#define rightStopValue 92
#define angleRotationTime 15
#define topServoStraight 90
#define topServoRight 4
#define topServoLeft 180


int leftStandardSpeed = leftStopValue + 200;
int rightStandardSpeed = rightStopValue - 200;
//for bottom LDR's. when value is higher than dark, it is off the line.
int dark = 0;
//top LDR when it is below dark then it is in shade
int topDark = 0;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  rightServo.attach(5);
  leftServo.attach(6);
  turret.attach(13);
  stopServos();

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A4, INPUT);
  
  calibrateLDR();
  //findClosestObjectTurret();

}

void loop() {
  // put your main code here, to run repeatedly:
  //int dist = FRightPing.ping_cm();
}

void followLine() {
  if (analogRead(A1) > dark) {
    turnUntilDark();
  }
  else {
    moveStraight(5);
  }
}

void moveStraight(unsigned int distance) {

  rightServo.write(rightStandardSpeed);
  leftServo.write(leftStandardSpeed);
  delay(distance+50);
  stopServos();
}

void moveBack(unsigned int distance){
  rightServo.write(-rightStandardSpeed);
  leftServo.write(-leftStandardSpeed);
  delay(distance+50);

  stopServos();
}

void turnAngle(float angle) {


  if (angle > 0) {
    rightServo.write(rightStandardSpeed);
    leftServo.write(-leftStandardSpeed);


  }
  else if (angle < 0) {
    leftServo.write(leftStandardSpeed);
    rightServo.write(-rightStandardSpeed);
    angle = angle * -1;

  }
  // angle * 44 is roughly the time it takes at standard speed to turn that angle.
  angle = angle * angleRotationTime;

  delay(angle);
  stopServos();

}

void turnUntilDark() {
  int i = 1;

  while (analogRead(A1) > dark) {
    turnAngle(i);
    i++;
    if (analogRead(A1) < dark) {
      break;
    }
    turnAngle(-i);
    i++;
  }

}

void stopServos() {
  rightServo.write(rightStopValue);
  leftServo.write(leftStopValue);
}

void calibrateLDR() {

  int average = (analogRead(A0) + analogRead(A2)) / 2;
  dark = (average + analogRead(A1)) / 2;
  int total = 0;
  
  for (int i=  0; i< 25; i++){
    Serial.println(i);
    total=total+analogRead(A4);
    randomMove();
    
  }
  Serial.println(total);
  topDark = total/50;
  Serial.println(topDark);

}

void turnTopServo(int angle) {

  angle = angle + 90;
  if (angle < topServoRight) {
    angle = topServoRight;
  }
  turret.write(angle);

}

void findClosestObjectTurret() {
  digitalWrite(12, LOW);
  int closestAngle = 0;
  int closestDistance = 32000;
  turnTopServo(-90);
  delay(1000);
  for (int i = -90 ; i <= 90 ; i++) {
    turnTopServo(i);
    delay(20);
    int dist_cm = IRTop.distance();


    if (dist_cm < closestDistance) {
      closestDistance = dist_cm;
      closestAngle = i;
    }

  }
  digitalWrite(12, HIGH);


}

void randomMove(){
  int direction = random(0,2);

if (direction == 1){
  moveStraight(10);
}else{
  moveBack(10);
  
}

}



