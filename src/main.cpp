#include <Arduino.h>
#include <toneAC.h>
#include <SPI.h>
#include <RF24.h>

// Change based on the box:
uint8_t address[6] = "SMAL1";

// Pin definitions
#define RGB_RED 3
#define RGB_BLUE 6
#define RGB_GREEN 5

#define CE_PIN 7
#define CSN_PIN 8

#define SENS 14
#define LASER 15
#define SENS_EN 16

// Radio stuff
RF24 radio(CE_PIN, CSN_PIN);
struct Packet {
    uint8_t from[6];
    uint8_t event;
} packet;
#define EVENT_ALARM_ON  1
#define EVENT_ALARM_OFF 2

// Hit Alarm stuff
#define hitTimeout 3000
unsigned long hitTimer = 0;
bool alarm = false;

// Siren Stuff
int alarminc = 7;
unsigned long freq = 2000;
bool selfSiren = true;

void rgbLight(byte red, byte green, byte blue) {
    analogWrite(RGB_RED, red);
    analogWrite(RGB_GREEN, green);
    analogWrite(RGB_BLUE, blue);
}

void setup() {
    pinMode(RGB_RED, OUTPUT);
    pinMode(RGB_GREEN, OUTPUT);
    pinMode(RGB_BLUE, OUTPUT);

    pinMode(LASER, OUTPUT);
    pinMode(SENS_EN, OUTPUT);
    pinMode(SENS, INPUT);

    digitalWrite(SENS_EN, HIGH);
    digitalWrite(LASER, HIGH);

    // Set packet from address for sending
    memcpy(packet.from, address, 6);

    // initialize the transceiver on the SPI bus
    if (!radio.begin()) {
        // Serial.println(F("radio hardware is not responding!!"));
        while (1) {}  // hold in infinite loop
    }
    radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
    radio.openWritingPipe(address);
    radio.openReadingPipe(1, address);
    radio.setPayloadSize(sizeof(Packet));
    radio.startListening();
}

void sendEvent(uint8_t event) {
    radio.stopListening();
    packet.event = event;
    radio.write(&packet, sizeof(Packet));
    radio.startListening();
}

void loop() {
    unsigned long currentMillis = millis();

    uint8_t pipe;
    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that received it
        radio.read(&packet, sizeof(Packet));             // fetch payload from FIFO
        switch (packet.event) {
            case EVENT_ALARM_ON:
                alarm = true;
                selfSiren = false;
            break;
            case EVENT_ALARM_OFF:
                noToneAC();
                rgbLight(0, 0, 0);
                alarm = false;
            break;
            default:
            break;
        }
    }


    if (digitalRead(SENS) && alarm == false) {
        // Laser not hitting the sensor
        alarm = true;
        hitTimer = currentMillis;
        selfSiren = true;
        // Send alarm packet
        sendEvent(EVENT_ALARM_ON);
    }

    if (alarm) {
        freq = freq + alarminc;
        if (freq < 1000 || freq > 8000) {
            alarminc = alarminc * -1;
        }
        byte ledValue = (freq - 1000) / 28;
        rgbLight(selfSiren ? 0 : ledValue, selfSiren ? ledValue : 0, 0);
        toneAC(freq);
    }

    if (alarm && selfSiren && currentMillis - hitTimer > hitTimeout) {
        noToneAC();
        rgbLight(0, 0, 0);
        // digitalWrite(SENS_EN, HIGH);
        sendEvent(EVENT_ALARM_OFF);
        alarm = false;
    }
    delay(1);
}
