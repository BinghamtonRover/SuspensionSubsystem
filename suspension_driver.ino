// includes
#include <rocs.hpp>
#include <HardwareSerial.h>
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

const uint8_t ROCS_ID = 0x11;
const char ROCS_NAME[] = "suspension";


HardwareSerial& odrive_serial_1 = Serial2;
HardwareSerial& odrive_serial_2 = Serial3;
HardwareSerial& odrive_serial_3 = Serial4;

ODriveArduino odrive_1(odrive_serial_1);
ODriveArduino odrive_2(odrive_serial_2);
ODriveArduino odrive_3(odrive_serial_3);

uint32_t float_to_uint32;
uint8_t* uint32_to_uint8_arr;
uint8_t reg_dump_1;
uint8_t reg_dump_2;
uint8_t reg_dump_3;
uint8_t reg_dump_4;

uint8_t left_dir;
uint8_t left_speed;
uint8_t right_dir;
uint8_t right_speed;

int left_side_movement;
int right_side_movement;
float motor_current;
float left_side_speed;
float right_side_speed;

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

static void stop_left_wheels(){
  left_side_movement = 0;
  odrive_serial_1 << "w axis0.controller.config.input_vel 0\n";
  odrive_serial_2 << "w axis0.controller.config.input_vel 0\n";
  odrive_serial_3 << "w axis0.controller.config.input_vel 0\n";  
}

static void stop_right_wheels(){
  right_side_movement = 0;
  left_side_movement = 0;
  odrive_serial_1 << "w axis1.controller.config.input_vel 0\n";
  odrive_serial_2 << "w axis1.controller.config.input_vel 0\n";
  odrive_serial_3 << "w axis1.controller.config.input_vel 0\n";  
}

static void adjust_left_wheels_speed(){
  if(left_dir == 1){
    left_side_movement = (-1) * left_speed;
  }
  else{
    left_side_movement = left_speed;
  }
  odrive_serial_1 << "w axis0.controller.config.input_vel " << left_side_movement << "\n";
  odrive_serial_2 << "w axis0.controller.config.input_vel " << left_side_movement << "\n";
  odrive_serial_3 << "w axis0.controller.config.input_vel " << left_side_movement << "\n";  
}

static void adjust_right_wheels_speed(){
  if(right_dir == 1){
    right_side_movement = (-1) * right_speed;
  }
  else{
    right_side_movement = right_speed;
  }
  odrive_serial_1 << "w axis1.controller.config.input_vel " << right_side_movement << "\n";
  odrive_serial_2 << "w axis1.controller.config.input_vel " << right_side_movement << "\n";
  odrive_serial_3 << "w axis1.controller.config.input_vel " << right_side_movement << "\n";
}

void write_handler(uint8_t reg, uint8_t value) {
  // update
  switch(reg){
    case 0x05:
      left_dir = value;
      break;
    case 0x06:
      left_speed = value;
      break;
    case 0x07:
      right_dir = value;
      break;
    case 0x08:
      right_speed = value;
      break;
    case 0x09:
      uint32_t float_to_uint32 = rocs::float_to_uint32(motor_current);
      uint32_to_uint8_arr = rocs::uint8_batch(temp_uint32);
      reg_dump_1 = rocs::get_uint8_from_arr(float_to_uint8_arr, 0);
      reg_dump_2 = rocs::get_uint8_from_arr(float_to_uint8_arr, 1);
      reg_dump_3 = rocs::get_uint8_from_arr(float_to_uint8_arr, 2);
      reg_dump_4 = rocs::get_uint8_from_arr(float_to_uint8_arr, 3);
      break;
  }
}

uint8_t read_handler(uint8_t reg) {
    switch (reg) {
    case 0x01:
      return reg_dump_1;
    case 0x02:
      return reg_dump_2;
    case 0x03:
      return reg_dump_3;
    case 0x04:
      return reg_dump_4;
    default:
      break;
    }
    return 0;
}

void setup() {
  // Serial to PC
  Serial.begin(115200);
  while (!Serial); // wait for Arduino Serial Monitor to open
  config_odrive(odrive_serial_1, odrive_1, 1);
  config_odrive(odrive_serial_2, odrive_2, 2);
  config_odrive(odrive_serial_3, odrive_3, 3);

  left_dir = 0;
  left_speed = 0;
  right_dir = 0;
  right_speed = 0;

  rocs::init(ROCS_ID, ROCS_NAME, strlen(ROCS_NAME));
  rocs::set_write_handler(write_handler);
  rocs::set_read_handler(read_handler);
}

void loop() {
  if(left_speed != 0 || right_speed != 0){
    adjust_left_wheels_speed();
    adjust_right_wheels_speed();
  }
  else{
    stop_left_wheels();
    stop_right_wheels();
  }
  Serial << "left: " << left_side_movement << "; right: " << right_side_movement << "\n";
  delay(100);
}