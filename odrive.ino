// includes
#include <rocs.hpp>
#include <HardwareSerial.h>
//#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj, T arg){
  obj.print(arg);
  return obj; 
}
template<> inline Print& operator <<(Print &obj, float arg){
  obj.print(arg, 4);
  return obj;
}

const uint8_t ROCS_ID = 0x01;
const char ROCS_NAME[] = "suspension";

HardwareSerial& odrive_serial_1 = Serial2;
HardwareSerial& odrive_serial_2 = Serial3;
HardwareSerial& odrive_serial_3 = Serial4;

ODriveArduino odrive_1(odrive_serial_1);
ODriveArduino odrive_2(odrive_serial_2);
ODriveArduino odrive_3(odrive_serial_3);

float data_dump;
uint8_t read_request;
uint8_t reg_dump_1;
uint8_t reg_dump_2;
uint8_t reg_dump_3;
uint8_t reg_dump_4;

uint8_t left_dir;
uint8_t left_speed;
uint8_t right_dir;
uint8_t right_speed;

static void config_odrive(HardwareSerial& odrive_serial, ODriveArduino& odrive, int counter){
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  Serial << "Odrive " << counter << ": initializing...\n";

  float sensorless_ramp_current;
  float pm_flux_linkage;

  odrive_serial << "w axis0.controller.config.vel_gain 0.01\n";
  odrive_serial << "w axis0.controller.config.vel_integrator_gain 0.05\n";
  odrive_serial << "w axis0.controller.config.control_mode 2\n";
  odrive_serial << "w axis0.controller.config.vel_limit 255\n";
  odrive_serial << "r axis0.config.sensorless_ramp.current\n";  
  sensorless_ramp_current = odrive.readFloat();
  sensorless_ramp_current *= 2;
  odrive_serial << "w axis0.motor.config.current_lim " << sensorless_ramp_current << "\n"; 
  odrive_serial << "w axis0.motor.config.direction 1\n";
  // pm_flux_linkage ~= 0.001378322238555
  pm_flux_linkage = 5.51328895422f / (40.0f * 100.0f);
  odrive_serial << "w axis0.sensorless_estimator.config.pm_flux_linkage " << pm_flux_linkage << "\n";

  
  odrive_serial << "w axis1.controller.config.vel_gain 0.01\n";
  odrive_serial << "w axis1.controller.config.vel_integrator_gain 0.05\n";
  odrive_serial << "w axis1.controller.config.control_mode 2\n";
  odrive_serial << "w axis1.controller.config.vel_limit 255\n";
  odrive_serial << "r axis1.config.sensorless_ramp.current\n";  
  sensorless_ramp_current = odrive.readFloat();
  sensorless_ramp_current *= 2;
  odrive_serial << "w axis1.motor.config.current_lim " << sensorless_ramp_current << "\n"; 
  odrive_serial << "w axis1.motor.config.direction 1\n";
  // pm_flux_linkage ~= 0.001378322238555
  pm_flux_linkage = 5.51328895422f / (40.0f * 100.0f);
  odrive_serial << "w axis1.sensorless_estimator.config.pm_flux_linkage " << pm_flux_linkage << "\n";

  Serial << "Odrive " << counter <<  " : Done!\n";
}

static void stop() {
  odrive_serial_1 << "w axis0.controller.config.input_vel 0\n";
  odrive_serial_1 << "w axis1.controller.config.input_vel 0\n";
  odrive_serial_2 << "w axis0.controller.config.input_vel 0\n";
  odrive_serial_2 << "w axis1.controller.config.input_vel 0\n";
  odrive_serial_3 << "w axis0.controller.config.input_vel 0\n";
  odrive_serial_3 << "w axis1.controller.config.input_vel 0\n";
}

void setup() {
  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open
  config_odrive(odrive_serial_1, odrive_1, 1);
  config_odrive(odrive_serial_2, odrive_2, 2);
  config_odrive(odrive_serial_3, odrive_3, 3);

  rocs::init(ROCS_ID, ROCS_NAME, sizeof(ROCS_NAME) - 1);
  rocs::set_write_handler(write_handler);
  rocs::set_read_handler(read_handler);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't') {
      for(int i = 10; i < 41; i++){
        odrive_serial_1 << "w axis0.controller.input_vel "<< i << "\n";
        odrive_serial_1 << "w axis1.controller.input_vel "<< i << "\n";
        delay(100);
      }
      odrive_serial_1 << "w axis0.controller.input_vel 0\n";
      odrive_serial_1 << "w axis1.controller.input_vel 0\n";
    }
  }
}

void write_handler(uint8_t reg, uint8_t value) {
  // update
  if(reg == 0x01){
    read_request = value;
    switch (read_request)
    {
    case 0x01:
      //dump velocity data
      break;
    case 0x02:
      //dump motor current data
    default:
      break;
    }
  }
  else if (reg == 0x06) {
    left_dir = value;
  }
  else if (reg == 0x07) {
    left_speed = value;
  }
  else if (reg == 0x08) {
    right_dir = value;
  }
  else if (reg == 0x09) {
    right_dir = value;
  }
}

uint8_t read_handler(uint8_t reg, uint8_t value) {
    switch (reg) {
    case 0x01:
      //convert float to reg values
      break;
    case 0x02:
      return reg_dump_1;
    case 0x03:
      return reg_dump_2;
    case 0x04:
      return reg_dump_3;
    case 0x05:
      return reg_dump_4;
    default:
      break;
    }
    return 0;
  }
}