#include <rocs.hpp>

const uint8_t ROCS_ID = 0x01;
const char ROCS_NAME[] = "suspension";

const int L_DIR_PIN = 11;
const int L_SPEED_PIN = 9;

const int R_DIR_PIN = 5;
const int R_SPEED_PIN = 6;

const uint8_t MIN_SPEED = 255;
const uint8_t MAX_SPEED = 0;

#define RECEIVE_TIMEOUT 1000

uint32_t last_receive_time;

static void stop() {
    analogWrite(L_SPEED_PIN, MIN_SPEED);
    analogWrite(R_SPEED_PIN, MIN_SPEED);
}

void setup() {
    pinMode(L_DIR_PIN, OUTPUT);
    pinMode(L_SPEED_PIN, OUTPUT);

    pinMode(R_DIR_PIN, OUTPUT);
    pinMode(R_SPEED_PIN, OUTPUT);

    digitalWrite(L_DIR_PIN, HIGH);
    digitalWrite(R_DIR_PIN, HIGH);

    stop();
  
    rocs::init(ROCS_ID, ROCS_NAME, sizeof(ROCS_NAME) - 1);

    rocs::set_write_handler(write_handler);

    last_receive_time = millis();
}


void write_handler(uint8_t reg, uint8_t value) {
    if (reg == 0x01) {
        analogWrite(L_SPEED_PIN, MIN_SPEED - value);
    } else if (reg == 0x02) {
        if (value == 0) digitalWrite(L_DIR_PIN, HIGH);
        else if (value == 1) digitalWrite(L_DIR_PIN, LOW);
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
        stop();
    }
}
