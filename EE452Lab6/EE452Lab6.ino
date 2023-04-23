#include <MFRC522.h>
#include <SPI.h>
#include <LiquidCrystal.h>

#define RST_PIN 10
#define SS_PIN 53

MFRC522 mfrc522(SS_PIN, RST_PIN);

const int buzz = 9;
const int rs = 2;
const int en = 3;
const int d4 = 4;
const int d5 = 5;
const int d6 = 6;
const int d7 = 7;

LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
    Serial.begin(9600);
    SPI.begin();
    mfrc522.PCD_Init();
    Serial.println("Place card next to reader");
    Serial.println();
    pinMode(buzz, OUTPUT);
    lcd.begin(16, 2);
}

void loop() {
    if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
        delay(50);
        return;
    }
    String ID = "";
    Serial.print("Tag UID: ");
    for (byte i=0; i<mfrc522.uid.size; i++) {
        Serial.print(mfrc522.uid.uidByte[i]<0x10 ? " 0": " ");
        Serial.print(mfrc522.uid.uidByte[i], HEX);
        ID.concat(String(mfrc522.uid.uidByte[i]<0x10 ? " 0": " "));
        ID.concat(String(mfrc522.uid.uidByte[i], HEX));
    }
    ID.toUpperCase();
    Serial.println();
    if (ID.substring(1) == "E4 F6 BC 4D") {
        //Serial.print("Access Granted");
        Serial.println();
        lcd.setCursor(0, 0);
        lcd.print("ACCESS GRANTED");
        delay(2000);
        lcd.clear();
    }
    else {
        unsigned char i;
        //Serial.print("Access Denied");
        Serial.println();
        lcd.setCursor(0,1);
        lcd.print("ACCESS DENIED");
        for (int j=0; j<5; j++) {
            for (i=0; i<100; i++) {
                digitalWrite(buzz, HIGH);
                delay(1);
                digitalWrite(buzz, LOW);
                delay(1);
            }
            for (i=0; i<100; i++) {
                digitalWrite(buzz, HIGH);
                delay(2);
                digitalWrite(buzz, LOW);
                delay(2);
            }
        }
        delay(2000);
        lcd.clear();
    }
    //delay(2000);
}
