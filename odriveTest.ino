#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveUART.h>
#include <math.h>
#include <Adafruit_BNO055.h>

#define HLen 1
#define HRen 10
#define VLen 11
#define VRen 12
#define HLpwm 5
#define HRpwm 6
#define VLpwm 7
#define VRpwm 2
#define dcv 3
#define pie 3.14

HardwareSerial& odrive_serial1 = Serial1;
HardwareSerial& odrive_serial3 = Serial3;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

ODriveUART odrive1(Serial1);
ODriveUART odrive3(Serial3);

float range, theta, u, W, Viroller, rpm, rps, currentAngle, bnoy, a;
int DCVcommand;


void setup() {
  pinMode(HLen, OUTPUT);
  pinMode(HRen, OUTPUT);
  pinMode(VLen, OUTPUT);
  pinMode(VRen, OUTPUT);
  pinMode(HLpwm, OUTPUT);
  pinMode(HRpwm, OUTPUT);
  pinMode(VLpwm, OUTPUT);
  pinMode(VRpwm, OUTPUT);
  pinMode(dcv, OUTPUT);

  digitalWrite(HLen, HIGH);
  digitalWrite(HRen, HIGH);
  digitalWrite(VLen, HIGH);
  digitalWrite(VRen, HIGH);

  odrive_serial1.begin(115200);
  odrive_serial3.begin(115200);

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial3.begin(115200);
  // while (!bno.begin()) {
  //   Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //   delay(100);
  // }
  // bno.setExtCrystalUse(true);
  Serial.println("Ready!");

  // Serial.println("Enter the Range: ");
  // while (Serial.available() == 0);
  // range = Serial.parseInt();
  // Serial.println(range);

  // Serial.println("Enter the Angle: ");
  // while (Serial.available() == 0);
  // theta = Serial.parseInt();
  // Serial.println(theta);
  // theta = theta * (pie / 180);

  Serial.println("Enter the rps: ");
  while (Serial.available() == 0);
  rps = Serial.parseInt();
  Serial.println(rps);
}

void calculations() {

  u = sqrt((4.9 * range * range) / (((range * tan(theta) + 0.44) * cos(theta) * cos(theta))));//1.59
  Viroller = 1.755 * (u - 1);
  W = Viroller / 0.0755;0 0
  rpm = 9.55 * W;
  rps = rpm / 60;

  Serial.print(u);
  Serial.print("   ");
  Serial.print(Viroller);
  Serial.print("   ");
  Serial.print(W);
  Serial.print("   ");
  Serial.print(rpm);
  Serial.print("   ");
  Serial.println(rps);
}

void DCV() {
  DCVcommand = Serial.parseInt();
  if (DCVcommand == 1) {
    digitalWrite(dcv, HIGH);
    Serial.println("Closed");
    delay(3000);
    digitalWrite(dcv, LOW);
    Serial.println("Open");
  } else if (DCVcommand == 2) {
    digitalWrite(dcv, LOW);
    Serial.println("Open");
  } else if (DCVcommand) {
    rps = DCVcommand;
    a = Serial.parseInt();
  } else {
    Serial.println("Wrong DCV command");
  }
}

void setAngle() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  currentAngle = 90 - bnoy;
  if (theta >= 23 && theta <= 43) {
    if (theta > currentAngle) {
      while (currentAngle != theta + 2 || theta - 2) {
        analogWrite(VLpwm, 20);
        analogWrite(VRpwm, 0);
        analogWrite(HLpwm, 0);
        analogWrite(HRpwm, 20);
      }
    } else if (theta < currentAngle) {
      while (currentAngle != theta + 2 || theta - 2) {
        analogWrite(VLpwm, 0);
        analogWrite(VRpwm, 20);
        analogWrite(HLpwm, 20);
        analogWrite(HRpwm, 0);
      }
    } else {
      analogWrite(VLpwm, 0);
      analogWrite(VRpwm, 0);
      analogWrite(HLpwm, 0);
      analogWrite(HRpwm, 0);
    }
  }
}

void loop() {
  // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  // bnoy = euler.y();

  // setAngle();

  // calculations();
  odrive1.SetVelocity(0, rps);
  odrive3.SetVelocity(0, -rps);  //rps


  Serial.println("Enter the 1 to Close and 2 to Open");
  if (Serial.available() > 0) {
    DCV();
  }
}