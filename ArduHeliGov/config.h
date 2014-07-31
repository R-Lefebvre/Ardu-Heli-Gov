// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _CONFIG_H
#define _CONFIG_H

#define ENABLED 1
#define DISABLED 0

// Software config
#define Serial_Debug ENABLED
#define FrSky_Telemetry DISABLED

// Hardware config
#define Governor_Mode ENABLED
#define BoardLED 13
#define RPM_Input_1_Pin 2        // Uno, Ethernet 2, Mega2560 2, Leonado 3
#define Arming_Pin 4
#define RPM_Input_1_Mode INPUT   // INPUT, INPUT_PULLUP
#define Arming_Mode INPUT_PULLUP // INPUT, INPUT_PULLUP
#define SCOutput_Pin 8

// Gov PID Gains
#define PID_kp 1.0
#define PID_ki 0.0
#define PID_imax 0.0

// Drivetrain config
#define Direct_Measurement 1
#define Motor_Measurement 2
#define Measurement_Type Direct_Measurement
#define Motor_Poles 2
#define Gear_Ratio 2
#define PulsesPerRevolution 1
#define RSC_Ramp_Up_Rate 10						// Soft Start Ramp Rate in seconds
#define Target_RPM 1000

// Pinmode defines
#define INPUT 1
#define INPUT_PULLUP 2
#define OUTPUT 3

// I2C config
#define I2C                     ENABLED
#define SLAVE_ADDRESS      		0x29 // Slave address,any number from 0x01 to 0x7F
#define DEVICE_ID      			122  // Not sure what this should be

#endif // _CONFIG_H