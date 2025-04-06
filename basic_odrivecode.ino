#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveUART.h>
float a = 0;
HardwareSerial& odrive_serial1 = Serial1;
HardwareSerial& odrive_serial3 = Serial3;

int rps = 0, rps2 = 0;

ODriveUART odrive1(odrive_serial1);
ODriveUART odrive3(odrive_serial3);

void setup() {

  odrive_serial1.begin(115200);
  odrive_serial3.begin(115200);

  Serial.begin(115200);
  Serial1.begin(115200);
  Serial3.begin(115200);

  Serial.println("Ready!");
}

void loop() {
  odrive1.SetVelocity(0,-30);
  odrive3.SetVelocity(0,30);
}