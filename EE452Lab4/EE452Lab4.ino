#include <LiquidCrystal.h>
#include <Ultrasonic.h>

const int rs = 2;
const int en = 3;
const int d4 = 4;
const int d5 = 5;
const int d6 = 6;
const int d7 = 7;
const int trig = 12;
const int echo = 13;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
Ultrasonic ultrasonic(trig, echo);

void setup() {
    lcd.begin(16, 2);
}

void loop() {
    lcd.setCursor(0, 0);
    lcd.print("Distance(cm): ");
    lcd.println(ultrasonic.read());
    lcd.setCursor(0, 1);
    lcd.print("Distance(in): ");
    lcd.println(ultrasonic.read(INC));
    delay(2000);
}
