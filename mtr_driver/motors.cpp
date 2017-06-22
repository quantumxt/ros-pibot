#include "Arduino.h"
#include "motors.h"

MotorControl::MotorControl(int enA, int enB, int in1, int in2, int in3, int in4) {
  this->enA = enA;
  this->enB = enB;
  this->in1 = in1;
  this->in2 = in2;
  this->in3 = in3;
  this->in4 = in4;
}

void MotorControl::initMotors() {
  // Declare motor control pins to be in output
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

/*  Move function
    Dir (Boolean) { true: Forward,
          false: Backward }
    Spd (Int) { 0 <-> maxSpd }
    bias (Int) { Move more to left or right: -ve is left, +ve is right [Range: -turnFactor <-> turnFactor ] }
*/

void MotorControl::moveBot(float spd, float bias) {
  float sL = spd;
  float sR = spd;
  float b = bias;
  bool dir = (spd > 0) ? 1 : 0;

  if (spd == 0) {
    dir = (bias > 0) ? 1 : 0;
    rotateBot(dir, bias / turnFactor);
  } else {
    sL = constrain(sL, -moveFactor * maxSpd, moveFactor * maxSpd);
    sR = constrain(sR, -moveFactor * maxSpd, moveFactor * maxSpd);
    if (sL < 0) {
      sL *= -1;
    }
    if (sR < 0) {
      sR *= -1;
    }
    // Motor A
    digitalWrite(in1, dir);
    digitalWrite(in2, !dir);  //The '!' symbol inverts the boolean value. So for example, if dir is true, !dir is false.
    // Motor B
    digitalWrite(in3, dir);
    digitalWrite(in4, !dir);
    // Set motor speed for turning
    if (b != 0) {
      if (b < 0) {
        b *= -1;
        //Go left
        sL *= turnFactor - b;
        sR *= b;
      } else if (b > 0) {
        //Go right
        sL *= b;
        sR *= turnFactor - b;
      }
      sL /=  turnFactor;
      sR /=  turnFactor;
    } else {

    }
    analogWrite(enA, (int) sL);
    analogWrite(enB, (int) sR);
  }
  delay(1);
}

void MotorControl::rotateBot(bool dir, float spd) {
  int s = spd * maxSpd;
  if (s < 0) {
    s *= -1;
  }
  // Motor A
  digitalWrite(in1, dir);
  digitalWrite(in2, !dir);  //The '!' symbol inverts the boolean value. So for example, if dir is true, !dir is false.
  // Motor B
  digitalWrite(in3, !dir);
  digitalWrite(in4, dir);
  // Set motor speed to spd
  analogWrite(enA, s);
  analogWrite(enB, s);
  delay(1);
}

//Turn off both motors
void MotorControl::stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
