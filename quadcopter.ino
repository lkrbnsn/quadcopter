#include <Servo.h>
#include <math.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "BMP180.h"

#define MAX_MOTOR 80
#define MIN_MOTOR 50

/*
   red
 1\   /2
   \ /
    O
   / \
 4/   \3
  black
*/

//Servo motor1;
//Servo motor2;
Servo motor3;
Servo motor4;

float v3 = 60;
float v4 = 60;

String incomingString;

MPU9250 accelgyro;
float Axyz[3];
float Gxyz[3];
float AvgAxyz[] = {0, 0, 0};
float AvgGxyz[] = {0, 0, 0};
float IniAxyz[] = {0, 0, 0};
float IniGxyz[] = {0, 0, 0};
int ax, ay, az;
int gx, gy, gz;
int mx, my, mz;

int i = 0;
float rollGyr = 0;
float rollAcc = 0;
float roll = 0;

void getAccelData(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384;
  Gxyz[0] = ((double) gx * 250 / 32768) + 1.45;
  Gxyz[1] = ((double) gy * 250 / 32768) - 1.6;
  Gxyz[2] = ((double) gz * 250 / 32768) - 6.75;

  AvgAxyz[0] = (AvgAxyz[0] * 9 + Axyz[0]) / 10;
  AvgAxyz[1] = (AvgAxyz[1] * 9 + Axyz[1]) / 10;
  AvgAxyz[2] = (AvgAxyz[2] * 9 + Axyz[2]) / 10;
  AvgGxyz[0] = (AvgGxyz[0] * 9 + Gxyz[0]) / 10;
  AvgGxyz[1] = (AvgGxyz[1] * 9 + Gxyz[1]) / 10;
  AvgGxyz[2] = (AvgGxyz[2] * 9 + Gxyz[2]) / 10;
}

void calculateVelocities()
{
  rollGyr -= (AvgGxyz[0] - IniGxyz[0]) * 0.01;
  rollAcc = asin(AvgAxyz[1] - IniAxyz[1]) * 57.2958;
  roll = (rollGyr * 0.95 + rollAcc * 0.5);
  
  v3 = v3 - roll / 10;
  v4 = v4 + roll / 10;
  if (v3 > MAX_MOTOR) {
    v3 = MAX_MOTOR;
  }
  else if (v3 < MIN_MOTOR) {
    v3 = MIN_MOTOR;
  }
  if (v4 > MAX_MOTOR) {
    v4 = MAX_MOTOR;
  }
  else if (v4 < MIN_MOTOR) {
    v4 = MIN_MOTOR;
  }
}

void writeToMotors()
{
  motor3.write(v3);
  motor4.write(v4);
}

void setup()
{
  Wire.begin();
  accelgyro.initialize();

  //  motor1.attach(6);
  //  motor2.attach(7);
  motor3.attach(8);
  motor4.attach(9);

  getAccelData();
  IniAxyz[1] = Axyz[1];
  IniAxyz[2] = Axyz[2];
  IniAxyz[3] = Axyz[3];

  for (int i = 0; i < 12; i++)
  {
    IniGxyz[1] = ((IniGxyz[1]*9) + Gxyz[1]) / 10;
    IniGxyz[2] = ((IniGxyz[2]*9) + Gxyz[2]) / 10;
    IniGxyz[3] = ((IniGxyz[3]*9) + Gxyz[3]) / 10;
  }

  Serial.begin(9600);
  Serial.println("Initializing");

  //  motor1.write(30);
  //  delay(3000);
  //  Serial.println("Motor 1 armed");
  //
  //  motor2.write(30);
  //  delay(3000);
  //  Serial.println("Motor 2 armed");

  //  motor3.write(30);
  //  Serial.println("Motor 3 armed");
  //
  //  motor4.write(30);
  //  Serial.println("Motor 4 armed");
}


////////////////////////////////////////////////////


void loop() {
  i++;
  getAccelData();
  delay(10);

  calculateVelocities();


  if (i == 100)
  {
    Serial.print("IniGxyz[]: ");
    Serial.print(IniGxyz[1]); Serial.print(" "); Serial.print(IniGxyz[2]); Serial.print(" "); Serial.println(IniGxyz[3]);
    Serial.print("IniAxyz[1]: ");
    Serial.println(IniAxyz[1]);
    Serial.print("AvgGxyz[0]: ");
    Serial.println(AvgGxyz[0]);
    Serial.print("rollGyr: ");
    Serial.println(rollGyr);
    Serial.print("Axyz[1]: ");
    Serial.println(Axyz[1]);
    Serial.print("AvgAxyz[1]: ");
    Serial.println(AvgAxyz[1]);
    Serial.print("rollAcc: ");
    Serial.println(rollAcc);
    Serial.print("roll: ");
    Serial.println(roll);
    Serial.print("");
    Serial.println(v3);
    Serial.println(v4);
    Serial.println();
    i = 0;
  }

  if (Serial.available() > 0)
  {
    char ch = Serial.read();
    if (ch != 10) {
      Serial.print("I have received: ");
      Serial.print(ch, DEC);
      Serial.print('\n');
      incomingString += ch;
    }
    else
    {
      // print the incoming string
      Serial.println("I am printing the entire string");
      Serial.println(incomingString);

      // Convert the string to an integer
      int val = incomingString.toInt();

      // print the integer
      Serial.println("Printing the value: ");
      Serial.println(val);
      if (val > -1 && val < 181)
      {
        Serial.println("Value is between 0 and 180");
        motor3.write(val);
        motor4.write(val);
        delay(10000);
      }
      else
      {
        Serial.println("Value is NOT between 0 and 180");
      }
      incomingString = "";
    }
  }
}
