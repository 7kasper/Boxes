#include <Arduino.h>
#include <toneAC.h>
#include <SPI.h>
#include <RF24.h>
#ifdef BIGBOX
    #include <I2C_LCD.h>
    #include <I2C_LCD_special_chars.h>
    I2C_LCD lcd(0x27);
#endif

// === Per Device Settings ===
uint8_t name[6] = "SMAL1";
int alarminc = 7; // Alarm tone speed.

// === Pin definitions ===
#define RGB_RED 3
#define RGB_BLUE 6
#define RGB_GREEN 5

#define CE_PIN 7
#define CSN_PIN 8

#define SENS 14
#define LASER 15
#define SENS_EN 16

// === Radio stuff ====

// Channel Names
uint8_t broadcast_all[6]        = "BROAD";
uint8_t broadcast_smallbox[6]   = "BSMAL";
uint8_t broadcast_bigbox[6]     = "BROAD";

// Radio Events
#define EVENT_ALARM_ON  1
#define EVENT_ALARM_OFF 2
RF24 radio(CE_PIN, CSN_PIN);
struct Packet {
    uint8_t from[6];
    uint8_t event;
} packet;


// === Alarm Stuff ====
#define hitTimeout 3000
unsigned long hitTimer = 0;
bool alarm = false;
unsigned long freq = 2000;

bool iActivatedTheAlarm = false;

// === Helper Functions ===

/**
 * Set the onboard RGB light to specified rgb color.
 * @param red - How much red [0-255]
 * @param green - How much green [0-255]
 * @param blue - How much blue [0-255]
 */
void rgbLight(byte red, byte green, byte blue) {
    analogWrite(RGB_RED, red);
    analogWrite(RGB_GREEN, green);
    analogWrite(RGB_BLUE, blue);
}

/**
 * Send simple radio event to specified channel
 * @param to - Whom to send the event to
 * @param event - The radio event to send
 */
void sendEvent(uint8_t to[6],  uint8_t event) {
    radio.stopListening();
    radio.openWritingPipe(to);
    packet.event = event;
    radio.write(&packet, sizeof(Packet));
    radio.startListening();
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
    memcpy(packet.from, name, 6);

    // Setup the radio tranceiver
    if (!radio.begin()) {
        while (1) {}  // freeze program if somehow the radio doesn't work.
    }
    radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default. (We are indoor not much power is needed)
    radio.setDataRate(RF24_250KBPS);
    radio.setPayloadSize(sizeof(Packet));
    // Channel pipes
    radio.openReadingPipe(1, broadcast_all); // Listen to the broadcast channel
    #ifdef SMALLBOX
        radio.openReadingPipe(2, broadcast_smallbox); // Listen to smallbox channel
    #endif
    #ifdef BIGBOX
        radio.openReadingPipe(2, broadcast_bigbox); // Listen to bigbox channel
    #endif
    radio.openReadingPipe(5, name);      // Listen to personal channel
    radio.startListening();

    // Setup the LCD
    #ifdef BIGBOX
        Wire.begin();
        Wire.setClock(100000);
        lcd.begin(16, 2);
        lcd.display();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("  = LaserBym =  ");
        lcd.setCursor(0, 1);
    #endif
}

void loop() {
    // === Common ====
    unsigned long currentMillis = millis();

    // === RX Handling ===
    uint8_t channel;
    if (radio.available(&channel)) {
        radio.read(&packet, sizeof(Packet));
        switch (packet.event) {
            case EVENT_ALARM_ON:
                alarm = true;
                iActivatedTheAlarm = false;
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

    // === Laser Handling ===
    #ifdef SMALLBOX
    if (digitalRead(SENS) && alarm == false) {
        // Laser not hitting the sensor
        alarm = true;
        hitTimer = currentMillis;
        iActivatedTheAlarm = true;
        // Send alarm packet
        sendEvent(broadcast_all, EVENT_ALARM_ON);
    }
    #endif

    // === IR Handling ===
    #ifdef BIGBOX

    #endif

    // === Alarm Reset Handling ===
    if (alarm && iActivatedTheAlarm && currentMillis - hitTimer > hitTimeout) {
        noToneAC();
        rgbLight(0, 0, 0);
        // digitalWrite(SENS_EN, HIGH);
        sendEvent(broadcast_all, EVENT_ALARM_OFF);
        alarm = false;
    }

    // === Alarm Output Handling ===
    if (alarm) {
        freq = freq + alarminc;
        if (freq < 1000 || freq > 8000) {
            alarminc = alarminc * -1;
        }
        byte ledValue = (freq - 1000) / 28;
        rgbLight(iActivatedTheAlarm ? 0 : ledValue, iActivatedTheAlarm ? ledValue : 0, 0);
        toneAC(freq);
    }

    // Sleep a bit
    delay(1);
}
