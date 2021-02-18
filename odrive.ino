// includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////

// Below are some sample configurations.
// You can comment out the default Teensy one and uncomment the one you wish to use.
// You can of course use something different if you like
// Don't forget to also connect ODrive GND to Arduino GND.

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
//HardwareSerial& odrive_serial = Serial1;

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
// HardwareSerial& odrive_serial = Serial1;

// Arduino without spare serial ports (such as Arduino UNO) have to use software serial.
// Note that this is implemented poorly and can lead to wrong data sent or read.
// pin 8: RX - connect to ODrive TX
// pin 9: TX - connect to ODrive RX
 SoftwareSerial odrive_serial(8, 9);
 //SoftwareSerial odrive_serial2(10, 11);


// ODrive object
ODriveArduino odrive(odrive_serial);
ODriveArduino odrive2(odrive_serial);

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  odrive_serial << "w axis0.controller.config.vel_gain 0.01\n";
  odrive_serial << "w axis0.controller.config.vel_integrator_gain 0.05\n";
  odrive_serial << "w axis0.controller.config.control_mode 2\n";
  odrive_serial << "w axis0.controller.input_vel 20\n";
  odrive_serial << "w axis0.controller.config.vel_limit 100\n";
  odrive_serial << "r axis0.config.sensorless_ramp.current\n";  
  float sensorless_ramp_current_m0 = odrive.readFloat();
  sensorless_ramp_current_m0 *= 2;
  odrive_serial << "w axis0.motor.config.current_lim " << sensorless_ramp_current_m0 << "\n"; 
  odrive_serial << "w axis0.motor.config.direction 1\n";
  float pm_flux_linkage_m0 = 5.51328895422f / (40.0f * 100.0f);
  odrive_serial << "w axis0.sensorless_estimator.config.pm_flux_linkage " << pm_flux_linkage_m0 << "\n";
  
  odrive_serial << "w axis0.requested_state AXIS_STATE_SENSORLESS_CONTROL\n";
  
  /*
  odrive_serial << "w axis1.controller.config.vel_gain 0.01\n";
  odrive_serial << "w axis1.controller.config.vel_integrator_gain 0.05\n";
  odrive_serial << "w axis1.controller.config.control_mode 2\n";
  odrive_serial << "w axis1.controller.config.vel_limit 100\n";
  odrive_serial << "r axis1.config.sensorless_ramp.current\n";  
  float sensorless_ramp_current_m1 = odrive.readFloat();
  sensorless_ramp_current_m1 *= 2;
  odrive_serial << "w axis1.motor.config.current_lim " << sensorless_ramp_current_m1 << "\n"; 
  odrive_serial << "w axis1.motor.config.direction 1\n";
  float pm_flux_linkage_m1 = 5.51328895422f / (40.0f * 100.0f);
  odrive_serial << "w axis1.sensorless_estimator.config.pm_flux_linkage " << pm_flux_linkage_m1 << "\n";
  
  odrive_serial << "w axis1.requested_state AXIS_STATE_SENSORLESS_CONTROL\n";
  */
  odrive_serial << "w axis0.controller.input_vel 20\n";
  

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  /*
  odrive_serial << "w axis1.controller.config.vel_gain 0.01\n";
  odrive_serial << "w axis1.controller.config.vel_integrator_gain 0.05\n";
  odrive_serial << "w axis1.controller.config.control_mode 2\n";
  odrive_serial << "w axis1.controller.input_vel 10\n";
  odrive_serial << "w axis1.controller.config.vel_limit 100\n";
  odrive_serial << "r axis1.config.sensorless_ramp.current\n";  
  float sensorless_ramp_current = odrive.readFloat();
  sensorless_ramp_current *= 2;
  odrive_serial << "w axis1.motor.config.current_lim " << sensorless_ramp_current << "\n"; 
  odrive_serial << "w axis1.motor.config.direction 1\n";
  float pm_flux_linkage = 5.51328895422f / (40.0f * 100.0f);
  odrive_serial << "w axis1.sensorless_estimator.config.pm_flux_linkage " << pm_flux_linkage << "\n";

   odrive_serial << "r axis1.config.sensorless_ramp.current\n";
  Serial << odrive.readFloat() << "\n";
  
  odrive_serial << "w axis1.requested_state AXIS_STATE_SENSORLESS_CONTROL\n";
  */

  Serial.println("Ready!");
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();
    // Left motor
    if (c == 'l') {
      for(int i = 10; i < 31; i+=10){
        odrive_serial << "w axis0.controller.input_vel "<<i<<"\n";
        odrive_serial << "w axis1.controller.input_vel "<<i<<"\n";
        delay(5000);
      }
    }
  }
}