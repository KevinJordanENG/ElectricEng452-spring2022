//Lab5 7Seg display/counters
//EE452L, Kevin Jordan ID:1301006

const int segA = 2;
const int segB = 3;
const int segC = 4;
const int segD = 5;
const int segE = 6;
const int segF = 7;
const int segG = 8;
const int reset = 13;

int digit_pin[] = {9, 10, 11, 12};

#define DIGIT_ON LOW
#define DIGIT_OFF HIGH

#define SEGMENT_ON HIGH
#define SEGMENT_OFF LOW

int count = 0;

void setup() {
    pinMode(segA, OUTPUT);
    pinMode(segB, OUTPUT);
    pinMode(segC, OUTPUT);
    pinMode(segD, OUTPUT);
    pinMode(segE, OUTPUT);
    pinMode(segF, OUTPUT);
    pinMode(segG, OUTPUT);
    pinMode(reset, INPUT_PULLUP);

    for (int i=0; i<4; i++) {
        pinMode(digit_pin[i], OUTPUT);
    }
}

void loop() {
    for (int count = 0; count<10000; count++) {
        if (count<10) {
            if (!digitalRead(reset)) {
                count = 0;
                lightNumber(10);
                break;
            }
            digitalWrite(digit_pin[0], DIGIT_OFF);
            digitalWrite(digit_pin[1], DIGIT_OFF);
            digitalWrite(digit_pin[2], DIGIT_OFF);
            digitalWrite(digit_pin[3], DIGIT_ON);
            lightNumber(count);
            delay(500);
        }
        else if (count>=10 && count<100) {
            if (!digitalRead(reset)) {
                count = 0;
                lightNumber(10);
                break;
            }
            digitalWrite(digit_pin[0], DIGIT_OFF);
            digitalWrite(digit_pin[1], DIGIT_OFF);
            digitalWrite(digit_pin[2], DIGIT_ON);
            digitalWrite(digit_pin[3], DIGIT_OFF);
            lightNumber(count/10);
            delay(100);
            digitalWrite(digit_pin[0], DIGIT_OFF);
            digitalWrite(digit_pin[1], DIGIT_OFF);
            digitalWrite(digit_pin[2], DIGIT_OFF);
            digitalWrite(digit_pin[3], DIGIT_ON);
            lightNumber(count%10);
            delay(400);
        }
        else if (count>=100 && count<1000) {
            if (!digitalRead(reset)) {
                count = 0;
                lightNumber(10);
                break;
            }
            digitalWrite(digit_pin[0], DIGIT_OFF);
            digitalWrite(digit_pin[1], DIGIT_ON);
            digitalWrite(digit_pin[2], DIGIT_OFF);
            digitalWrite(digit_pin[3], DIGIT_OFF);
            lightNumber(count/100);
            delay(50);
            digitalWrite(digit_pin[0], DIGIT_OFF);
            digitalWrite(digit_pin[1], DIGIT_OFF);
            digitalWrite(digit_pin[2], DIGIT_ON);
            digitalWrite(digit_pin[3], DIGIT_OFF);
            lightNumber((count%100)/10);
            delay(100);
            digitalWrite(digit_pin[0], DIGIT_OFF);
            digitalWrite(digit_pin[1], DIGIT_OFF);
            digitalWrite(digit_pin[2], DIGIT_OFF);
            digitalWrite(digit_pin[3], DIGIT_ON);
            lightNumber((count%100)%10);
            delay(350);
        }
        else if (count>=1000) {
            if (!digitalRead(reset)) {
                count = 0;
                lightNumber(10);
                break;
            }
            digitalWrite(digit_pin[0], DIGIT_ON);
            digitalWrite(digit_pin[1], DIGIT_OFF);
            digitalWrite(digit_pin[2], DIGIT_OFF);
            digitalWrite(digit_pin[3], DIGIT_OFF);
            lightNumber(count/1000);
            delay(50);
            digitalWrite(digit_pin[0], DIGIT_OFF);
            digitalWrite(digit_pin[1], DIGIT_ON);
            digitalWrite(digit_pin[2], DIGIT_OFF);
            digitalWrite(digit_pin[3], DIGIT_OFF);
            lightNumber((count%1000)/100);
            delay(50);
            digitalWrite(digit_pin[0], DIGIT_OFF);
            digitalWrite(digit_pin[1], DIGIT_OFF);
            digitalWrite(digit_pin[2], DIGIT_ON);
            digitalWrite(digit_pin[3], DIGIT_OFF);
            lightNumber(((count%1000)%100)/10);
            delay(100);
            digitalWrite(digit_pin[0], DIGIT_OFF);
            digitalWrite(digit_pin[1], DIGIT_OFF);
            digitalWrite(digit_pin[2], DIGIT_OFF);
            digitalWrite(digit_pin[3], DIGIT_ON);
            lightNumber(((count%1000)%100)%10);
            delay(300);
        }
    }
}

void lightNumber(int numberToDisplay) {
    switch(numberToDisplay){
        case 0:
            digitalWrite(segA, SEGMENT_ON);
            digitalWrite(segB, SEGMENT_ON);
            digitalWrite(segC, SEGMENT_ON);
            digitalWrite(segD, SEGMENT_ON);
            digitalWrite(segE, SEGMENT_ON);
            digitalWrite(segF, SEGMENT_ON);
            digitalWrite(segG, SEGMENT_OFF);
            break;
        case 1:
            digitalWrite(segA, SEGMENT_OFF);
            digitalWrite(segB, SEGMENT_ON);
            digitalWrite(segC, SEGMENT_ON);
            digitalWrite(segD, SEGMENT_OFF);
            digitalWrite(segE, SEGMENT_OFF);
            digitalWrite(segF, SEGMENT_OFF);
            digitalWrite(segG, SEGMENT_OFF);
            break;
        case 2:
            digitalWrite(segA, SEGMENT_ON);
            digitalWrite(segB, SEGMENT_ON);
            digitalWrite(segC, SEGMENT_OFF);
            digitalWrite(segD, SEGMENT_ON);
            digitalWrite(segE, SEGMENT_ON);
            digitalWrite(segF, SEGMENT_OFF);
            digitalWrite(segG, SEGMENT_ON);
            break;
        case 3:
            digitalWrite(segA, SEGMENT_ON);
            digitalWrite(segB, SEGMENT_ON);
            digitalWrite(segC, SEGMENT_ON);
            digitalWrite(segD, SEGMENT_ON);
            digitalWrite(segE, SEGMENT_OFF);
            digitalWrite(segF, SEGMENT_OFF);
            digitalWrite(segG, SEGMENT_ON);
            break;
        case 4:
            digitalWrite(segA, SEGMENT_OFF);
            digitalWrite(segB, SEGMENT_ON);
            digitalWrite(segC, SEGMENT_ON);
            digitalWrite(segD, SEGMENT_OFF);
            digitalWrite(segE, SEGMENT_OFF);
            digitalWrite(segF, SEGMENT_ON);
            digitalWrite(segG, SEGMENT_ON);
            break;
        case 5:
            digitalWrite(segA, SEGMENT_ON);
            digitalWrite(segB, SEGMENT_OFF);
            digitalWrite(segC, SEGMENT_ON);
            digitalWrite(segD, SEGMENT_ON);
            digitalWrite(segE, SEGMENT_OFF);
            digitalWrite(segF, SEGMENT_ON);
            digitalWrite(segG, SEGMENT_ON);
            break;
        case 6:
            digitalWrite(segA, SEGMENT_ON);
            digitalWrite(segB, SEGMENT_OFF);
            digitalWrite(segC, SEGMENT_ON);
            digitalWrite(segD, SEGMENT_ON);
            digitalWrite(segE, SEGMENT_ON);
            digitalWrite(segF, SEGMENT_ON);
            digitalWrite(segG, SEGMENT_ON);
            break;
        case 7:
            digitalWrite(segA, SEGMENT_ON);
            digitalWrite(segB, SEGMENT_ON);
            digitalWrite(segC, SEGMENT_ON);
            digitalWrite(segD, SEGMENT_OFF);
            digitalWrite(segE, SEGMENT_OFF);
            digitalWrite(segF, SEGMENT_OFF);
            digitalWrite(segG, SEGMENT_OFF);
            break;
        case 8:
            digitalWrite(segA, SEGMENT_ON);
            digitalWrite(segB, SEGMENT_ON);
            digitalWrite(segC, SEGMENT_ON);
            digitalWrite(segD, SEGMENT_ON);
            digitalWrite(segE, SEGMENT_ON);
            digitalWrite(segF, SEGMENT_ON);
            digitalWrite(segG, SEGMENT_ON);
            break;
        case 9:
            digitalWrite(segA, SEGMENT_ON);
            digitalWrite(segB, SEGMENT_ON);
            digitalWrite(segC, SEGMENT_ON);
            digitalWrite(segD, SEGMENT_ON);
            digitalWrite(segE, SEGMENT_OFF);
            digitalWrite(segF, SEGMENT_ON);
            digitalWrite(segG, SEGMENT_ON);
            break;
        case 10:
            digitalWrite(segA, SEGMENT_OFF);
            digitalWrite(segB, SEGMENT_OFF);
            digitalWrite(segC, SEGMENT_OFF);
            digitalWrite(segD, SEGMENT_OFF);
            digitalWrite(segE, SEGMENT_OFF);
            digitalWrite(segF, SEGMENT_OFF);
            digitalWrite(segG, SEGMENT_OFF);
            break;
    }
}
