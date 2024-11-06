#include <Arduino.h>
#include <toneAC.h>

// Pin definitions

#define SENS 14
#define LASER 15
#define SENS_EN 16

#define hitTimeout 3000
unsigned long hitTimer = 0;
bool alarm = false;



int alarminc = 7;
unsigned long freq = 2000;



void setup() {
    pinMode(LASER, OUTPUT);
    pinMode(SENS_EN, OUTPUT);
    pinMode(SENS, INPUT);
    digitalWrite(SENS_EN, HIGH);
    digitalWrite(LASER, HIGH);
}

void loop() {
    unsigned long currentMillis = millis();
    if (digitalRead(SENS) && alarm == false) {
        alarm = true;
        // Laser not hitting the sensor
        hitTimer = currentMillis;
        // toneAC(3000);
        delay(10);
        digitalWrite(SENS_EN, LOW);
    }
    if (alarm) {
        freq = freq + alarminc;
        if (freq < 1000 || freq > 8000) {
            alarminc = alarminc * -1;
        }
        toneAC(freq);
    }
    if (currentMillis - hitTimer > hitTimeout) {
        noToneAC();
        digitalWrite(SENS_EN, HIGH);
        alarm = false;
    }
    delay(1);
}
