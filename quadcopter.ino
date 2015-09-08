#include <Servo.h>

/*

 1\   /2
   \ /
    O
   / \
 4/   \3
  
*/

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

float Axyz[3];
int ax, ay, az;
int gx, gy, gz;
int mx, my, mz;

void getAccelData(void)
{
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}

void setup() {
  motor1.attach(6);
  motor2.attach(7);
  motor3.attach(8);
  motor4.attach(9);
  
  Serial.begin(9600);
  Serial.println("Initializing");
  
  motor1.write(30);
  delay(3000);
  Serial.println("Motor 1 armed");

  motor2.write(30);
  delay(3000);
  Serial.println("Motor 2 armed");

  motor3.write(30);
  delay(3000);
  Serial.println("Motor 3 armed");

  motor4.write(30);
  delay(3000);
  Serial.println("Motor 4 armed");
  
  
  motor1.write(50);
  motor2.write(50);
  motor3.write(50);
  motor4.write(50);
  delay(5000);
  motor1.write(40);
  motor2.write(40);
  motor3.write(40);
  motor4.write(40);
}

void loop() {
  // put your main code here, to run repeatedly:

}
