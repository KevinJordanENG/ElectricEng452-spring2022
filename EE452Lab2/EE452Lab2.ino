//Lab 2 EE452
//Kevin Jordan

String serialInput;
const int redPin = 11;
const int greenPin = 10;
const int bluePin = 9;
const int buttPin1 = 6;
const int buttPin2 = 5;
const int buttPin3 = 4;

void setup() {
    Serial.begin(9600);
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
    pinMode(buttPin1, INPUT_PULLUP);
    pinMode(buttPin2, INPUT_PULLUP);
    pinMode(buttPin3, INPUT_PULLUP);
}

void loop() {
    if (Serial.available() > 0) {
        serialInput = Serial.readString();
        if (serialInput.equals("off")) {
            changeColor(0,0,0); //off
            Serial.println("LED is now off");
        }
    }
    if (!digitalRead(buttPin1)) {
        changeColor(120,0,220); //violet
    }
    if (!digitalRead(buttPin2)) {
        changeColor(70,220,0); //grass
    }
    if (!digitalRead(buttPin3)) {
        changeColor(0,120,120); //turquoise
    }
}

void changeColor(int R, int G, int B) {
    analogWrite(redPin, R);
    analogWrite(greenPin, G);
    analogWrite(bluePin, B);
}
