#include <rocs.hpp>

const uint8_t ROCS_ID = 0x01;
const char ROCS_NAME[] = "suspension";

const int L_RELAY_PIN = 12;
const int R_RELAY_PIN = 13;

const int L_DIR_PIN = 11;
const int L_SPEED_PIN = 9;

const int R_DIR_PIN = 5;
const int R_SPEED_PIN = 6;

const uint8_t MIN_SPEED = 255;
const uint8_t MAX_SPEED = 0;

#define RECEIVE_TIMEOUT 1000

#define RELAY_ACTIVE_SIGNAL HIGH
#define RELAY_INACTIVE_SIGNAL LOW

uint32_t last_receive_time;

bool enabled = true;

static void stop() {
    analogWrite(L_SPEED_PIN, MIN_SPEED);
    analogWrite(R_SPEED_PIN, MIN_SPEED);
}

static void disable() {
    enabled = false;

    digitalWrite(L_RELAY_PIN, RELAY_INACTIVE_SIGNAL);  
    digitalWrite(R_RELAY_PIN, RELAY_INACTIVE_SIGNAL);  
}

static void enable() {
    digitalWrite(L_RELAY_PIN, RELAY_ACTIVE_SIGNAL);  
    digitalWrite(R_RELAY_PIN, RELAY_ACTIVE_SIGNAL);  

    enabled = true;
}

void setup() {
    pinMode(L_RELAY_PIN, OUTPUT);
    pinMode(R_RELAY_PIN, OUTPUT);

    pinMode(L_DIR_PIN, OUTPUT);
    pinMode(L_SPEED_PIN, OUTPUT);

    pinMode(R_DIR_PIN, OUTPUT);
    pinMode(R_SPEED_PIN, OUTPUT);

    // Flip left for now.
    digitalWrite(L_DIR_PIN, LOW);
    digitalWrite(R_DIR_PIN, HIGH);

    enable();
    stop();
  
    rocs::init(ROCS_ID, ROCS_NAME, sizeof(ROCS_NAME) - 1);

    rocs::set_write_handler(write_handler);

    last_receive_time = millis();
}


void write_handler(uint8_t reg, uint8_t value) {
    if (reg == 0x01) {
        analogWrite(L_SPEED_PIN, MIN_SPEED - value);
    } else if (reg == 0x02) {
        // Flip left for now so that they all go forward.
        if (value == 0) digitalWrite(L_DIR_PIN, LOW);
        else if (value == 1) digitalWrite(L_DIR_PIN, HIGH);
    } else if (reg == 0x03) {
        analogWrite(R_SPEED_PIN, MIN_SPEED - value);
    } else if (reg == 0x04) {
        if (value == 0) digitalWrite(R_DIR_PIN, HIGH);
        else if (value == 1) digitalWrite(R_DIR_PIN, LOW);
    } else {
        return;
    }

    last_receive_time = millis();
}

void loop() {
    // put your main code here, to run repeatedly:
    if (millis() - last_receive_time > RECEIVE_TIMEOUT) {
        if (enabled) {
            stop();
            disable();
        }
    } else {
        if (!enabled) {
            // We are within receive timeout, and disabled.
            // Reenable.
            enable();
        }
    }
}
