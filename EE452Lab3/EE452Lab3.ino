#include <Servo.h>

Servo myServo;

int val;

const int potPin = A0;
const int servoPin = 2;

void setup() {
    myServo.attach(servoPin);
}

void loop() {
    val = analogRead(potPin);
    val = map(val, 0, 1023, 0, 180);
    myServo.write(val);
    delay(15);
}
