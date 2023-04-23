//ESC Diagnostics

#include <Servo.h>

Servo ESC1, ESC2, ESC3, ESC4;

void setup() {
    ESC1.attach(4, 1000, 2000);
    ESC2.attach(5, 1000, 2000);
    ESC3.attach(6, 1000, 2000);
    ESC4.attach(7, 1000, 2000);

    ESC1.write(2000);
    delay(10000);
    for (int i=0; i<1000; i++) {
        ESC1.write(2000-i);
        delay(1);
    }
    ESC1.write(1000);
    delay(3000);
    
    ESC2.write(2000);
    delay(3000);
    for (int i=0; i<1000; i++) {
        ESC2.write(2000-i);
        delay(1);
    }
    ESC2.write(1000);
    delay(3000);

    ESC3.write(2000);
    delay(3000);
    for (int i=0; i<1000; i++) {
        ESC3.write(2000-i);
        delay(1);
    }
    ESC3.write(1000);
    delay(3000);

    ESC4.write(2000);
    delay(3000);
    for (int i=0; i<1000; i++) {
        ESC4.write(2000-i);
        delay(1);
    }
    ESC4.write(1000);
    delay(3000);
}

void loop() {
    
    ESC1.write(1200);
    ESC2.write(1200);
    ESC3.write(1200);
    ESC4.write(1200);
}
