#include <Servo.h>
Servo esc1;
Servo esc2;
int rpm, currentRPM = 1000;

int esc1_pin = 9;
int esc2_pin = 10;
int min_speed = 1000;
int max_speed = 2000;

void setup() {
  Serial.begin(9600);
  esc1.attach(esc1_pin, min_speed, max_speed);
  esc2.attach(esc2_pin, min_speed, max_speed);  //(pin,min,max)
  esc1.write(1000);                             // starting at 0
  esc2.write(1000);
  delay(2000);
  Serial.println("Enter value");
}
void loop() {
  while (Serial.available() == 0)
    ;
  rpm = Serial.parseInt();
  if (rpm > currentRPM) {
    while (currentRPM <= rpm) {
      esc1.write(currentRPM);
      esc2.write(currentRPM);
      Serial.println(currentRPM);
      currentRPM++;
      delay(60);
    }
  } else if (currentRPM == rpm) {
    esc1.write(currentRPM);
    esc2.write(currentRPM);
    Serial.println(currentRPM);
    delay(50);
  } else if (rpm < currentRPM) {
    while (rpm <= currentRPM) {
      esc1.write(currentRPM);
      esc2.write(currentRPM);
      Serial.println(currentRPM);
      currentRPM--;
      delay(60);
    }
  }
}
