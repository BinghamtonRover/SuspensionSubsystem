// includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
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

HardwareSerial& odrive_serial_1 = Serial2;
HardwareSerial& odrive_serial_2 = Serial3;
HardwareSerial& odrive_serial_3 = Serial4;

ODriveArduino odrive(odrive_serial);
ODriveArduino odrive2(odrive_serial);
ODriveArduino odrive3(odrive_serial);

static void config_odrive(HardwareSerial& odrive_serial, int counter){
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  Serial.println("Odrive " + counter + ": initializing...");

  float sensorless_ramp_current;
  float pm_flux_linkage;

  odrive_serial << "w axis0.controller.config.vel_gain 0.01\n";
  odrive_serial << "w axis0.controller.config.vel_integrator_gain 0.05\n";
  odrive_serial << "w axis0.controller.config.control_mode 2\n";
  odrive_serial << "w axis0.controller.config.vel_limit 100\n";
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
  odrive_serial << "w axis1.controller.config.vel_limit 100\n";
  odrive_serial << "r axis1.config.sensorless_ramp.current\n";  
  sensorless_ramp_current = odrive.readFloat();
  sensorless_ramp_current *= 2;
  odrive_serial << "w axis1.motor.config.current_lim " << sensorless_ramp_current << "\n"; 
  odrive_serial << "w axis1.motor.config.direction 1\n";
  // pm_flux_linkage ~= 0.001378322238555
  pm_flux_linkage = 5.51328895422f / (40.0f * 100.0f);
  odrive_serial << "w axis1.sensorless_estimator.config.pm_flux_linkage " << pm_flux_linkage << "\n";

  Serial.println("Odrive " + counter +  " : Done!");
}

void setup() {
  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open
  config_odrive(odrive_serial_1, 1);
  config_odrive(odrive_serial_2, 2);
  config_odrive(odrive_serial_3, 3);

}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't') {
      for(int i = 10; i < 41; i++){
        odrive_serial << "w axis0.controller.input_vel "<< i <<"\n";
        odrive_serial << "w axis1.controller.input_vel "<< i <<"\n";
        delay(100);
      }
      odrive_serial << "w axis0.controller.input_vel 0\n";
      odrive_serial << "w axis1.controller.input_vel 0\n";
    }
}