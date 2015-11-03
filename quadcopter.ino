#include <Servo.h>
#include <PID_v1.h>

#include "freeram.h"

#include "mpu.h"
#include "I2Cdev.h"

#define MAX_SPEED 100
#define MIN_SPEED 50

/*
   red
 1\   /2
   \ /
    O
   / \
 4/   \3
  black
*/

double rollRads = 0;

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

float v1 = 0;
float v2 = 0;
float v3 = 0;
float v4 = 0;

// Pitch PID variables
double pitch = 0;
double pSetpoint = 0;
double pOutput;
double pKp = 0.07, pKi = 0.03, pKd = 0.04;
PID pPID(&pitch, &pOutput, &pSetpoint, pKp, pKi, pKd, DIRECT);

// Roll PID variables
double roll = 0;
double rSetpoint = 0;
double rOutput;
double rKp = 0.07, rKi = 0.03, rKd = 0.04;
PID rPID(&roll, &rOutput, &rSetpoint, rKp, rKi, rKd, DIRECT);

int ret;
unsigned long startTime;

int throttle = 0;

void calculateVelocities()
{
  pPID.Compute();
  rPID.Compute();
  Serial.println(rOutput);

  v2 = 60 + throttle - pOutput - rOutput;
  v1 = 60 + throttle - pOutput + rOutput;
  v3 = 60 + throttle + pOutput - rOutput;
  v4 = 60 + throttle + pOutput + rOutput;

  if (v3 > MAX_SPEED)
    v3 = MAX_SPEED;
  if (v3 < MIN_SPEED)
    v3 = MIN_SPEED;

  if (v4 > MAX_SPEED)
    v4 = MAX_SPEED;
  if (v4 < MIN_SPEED)
    v4 = MIN_SPEED;
}

void writeToMotors()
{
  //  Serial.print(v3);
  //  Serial.print("\t");
  //  Serial.print(v4);
  //  Serial.print("\n");

  motor3.write(v3);
  motor4.write(v4);
  motor2.write(v3);
  motor1.write(v4);
}

void setup() {
  Fastwire::setup(400, 0);
  Serial.begin(115200);
  ret = mympu_open(2000);
  Serial.print("MPU init: "); Serial.println(ret);
  Serial.print("Free mem: "); Serial.println(freeRam());

  motor1.attach(6);
  motor2.attach(7);
  motor3.attach(8);
  motor4.attach(9);

  pPID.SetOutputLimits(-200, 200);
  pPID.SetMode(AUTOMATIC);

  rPID.SetOutputLimits(-200, 200);
  rPID.SetMode(AUTOMATIC);
}

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

void loop() {
  if (Serial.available() > 0)
  {
    char ch = Serial.read();
    if (ch != 10) {
      throttle = (ch - 48) * 4;
      if (throttle > 30)
        throttle = 30;
      if (throttle < 0)
        throttle = 0;
    }
  }

  startTime = micros();

  ret = mympu_update();

  switch (ret) {
    case 0: c++; break;
    case 1: np++; return;
    case 2: err_o++; return;
    case 3: err_c++; return;
    default:
      Serial.print("READ ERROR!  ");
      Serial.println(ret);
      return;
  }

  //    if (!(c%25)) {
  //	    Serial.print(np); Serial.print("  "); Serial.print(err_c); Serial.print(" "); Serial.print(err_o);
  //	    Serial.print(" Y: "); Serial.print(mympu.ypr[0]);
  //	    Serial.print(" P: "); Serial.print(mympu.ypr[1]);
  //	    Serial.print(" R: "); Serial.println(mympu.ypr[2]);
  //    }

  pitch = mympu.ypr[1] - 6;

  if (mympu.ypr[2] > 0)
    roll = mympu.ypr[2] - 175;
  else
    roll = 185 + mympu.ypr[2];

  //    rollRads = roll * 0.017444; // convert to rads
  //    roll = abs(roll) * sin(rollRads);

  Serial.print(" P: "); Serial.println(pitch);
  Serial.print(" R: "); Serial.println(roll);

  calculateVelocities();
  writeToMotors();

  Serial.println(micros() - startTime);
  Serial.println(throttle);
}

