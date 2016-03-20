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
#define angleRotationTime 17
#define topServoStraight 90
#define topServoRight 4
#define topServoLeft 180


int leftStandardSpeed = leftStopValue + 200;
int rightStandardSpeed = rightStopValue - 200;
//for bottom LDR's. when value is higher than dark, it is off the line.
int dark = 0;
//top LDR when it is below dark then it is in shade
int topLight = 0;


int closestAngle = 0;
int closestDistance = 32000;



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
  //    turnAngle(90);
  //    turnAngle(-90);






  //  while (1) {
  //    delay(2000);
  //    followWall(200);
  //Serial.println(BLeftPing.ping_median());
  //Serial.println(BLeftPing.ping_cm());
  //  }

}

void loop() {

  while (analogRead(A4) > topLight / 2) {
    if ((FRightPing.ping_cm() < 10 && FRightPing.ping_cm() != 0) || (FLeftPing.ping_cm() < 10 && FLeftPing.ping_cm() != 0)) {
      objectOnFront();
      Serial.println("front");
    } else {
      if ((BRightPing.ping_cm() < 10 && BRightPing.ping_cm() != 0) || (BLeftPing.ping_cm() < 10 && BLeftPing.ping_cm() != 0)) {
        Serial.println("follow");
        followWall(200);
      } else {
        moveStraight(200);
      }
    }
  }

  stopServos();
  Serial.println("Stop");
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
  delay(distance + 50);
  stopServos();
}

void moveBack(unsigned int distance) {
  rightServo.write(-rightStandardSpeed);
  leftServo.write(-leftStandardSpeed);
  delay(distance + 50);

  stopServos();
}

void turnAngle(float angle) {


  if (angle > 0) {
    rightServo.write(rightStandardSpeed);
    leftServo.write(-leftStandardSpeed);


  } else if (angle < 0) {
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
  Serial.println("Calibrating.....");
  int average = (analogRead(A0) + analogRead(A2)) / 2;
  dark = (average + analogRead(A1)) / 2;
  int total = 0;

  for (int i =  0; i < 25; i++) {
    Serial.println(i);
    total = total + analogRead(A4);
    randomMove(10);

  }

  topLight = (total / 25) - 80;

  Serial.println("finished calibrating.....");

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

  turnTopServo(-90);
  delay(1000);
  for (int i = -90 ; i <= 90 ; i++) {
    turnTopServo(i);
    delay(10);
    int dist_cm = IRTop.distance();


    if (dist_cm < closestDistance) {
      closestDistance = dist_cm;
      closestAngle = i;
    }

  }
  digitalWrite(12, HIGH);

}

void objectOnSide() {
  if (closestAngle < 0) {
    turnAngle(-closestAngle);

  } else if (closestAngle == 0) {
    turnAngle(90);
  } else if (closestAngle > 0) {
    turnAngle(-closestAngle);

  }
}

void randomMove(int distance) {

  int direction = random(0, 2);

  if (direction == 0) {
    moveStraight(distance);
  } else if (direction == 1) {
    moveBack(distance);
  }
  stopServos();

}

void followWall(unsigned int distance) {
    int distLeft = BLeftPing.ping_cm();
    int distRight = BRightPing.ping_cm();

  
  moveStraight(distance);

  if ((distLeft < distRight) || (distRight == 0 && distLeft != 0)) {
    if (distLeft < 10) {
      turnAngle(-1);
    } else {
      turnAngle(1);
    }
  } else if (distRight < distLeft || (distLeft == 0 && distRight != 0)) {
    if (distRight < 10) {
      turnAngle(1);
    } else {
      turnAngle(-1);
    }

  }
}

void objectOnFront() {

  int distLeft = BLeftPing.ping_cm();
  int distRight = BRightPing.ping_cm();


  if ((distLeft < distRight) || (distRight == 0 && distLeft != 0)) {
    //    Serial.println("turn 22 right");
    //    Serial.println(distLeft);
    //    Serial.println(distRight);
    turnAngle(22);
  } else if ((distRight < distLeft) || (distLeft == 0 && distRight != 0)) {
    //    Serial.println("turn 22 left");
    //    Serial.println(distLeft);
    //    Serial.println(distRight);

    turnAngle(-22);

  } else if (distRight == distLeft) {
    int direction = random(0, 2);
    //    Serial.println("random 90");
    //    Serial.println(distLeft);
    //    Serial.println(distRight);

    if (direction == 0) {
      turnAngle(90);
    } else if (direction == 1) {
      turnAngle(-90);

    }
  } else {
    //    Serial.println("turn 180");
    //    Serial.println(distLeft);
    //    Serial.println(distRight);

    turnAngle(180);
  }
}



