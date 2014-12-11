/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduHeliGov V0.1"

/*
ArduHeliGov 0.1
Lead author:	Robert Lefebvre

This firmware is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This program is intended to operate with a maximum rotorspeed of 4000rpm.
4000rpm = 66.667rev/sec = 0.015sec/rev = 15ms/rev = 15000uS/rev
Also:
800rpm = 13.333rev/sec = 0.075sec/rev = 75ms/rev = 75000uS/rev

1ms accuracy would give between 10 and 250rpm resolution which is not acceptable
Thus, we must use microseconds to measure RPM.
Atmega chip operating at 16MHz has 4uS resolution, 8MHz gives 8uS resolution. 
4uS resolution will give an RPM resolution of ~ 0.05 to 1 rpm.
Better than 0.025% accuracy!

Maximum intended motor RPM is 50,000 representing a 450 heli running 4S battery
50000rpm = 833rev/sec = 0.0012sec/rev = 1200uS/rev
4uS accuracy would give 166rpm accuracy, or 0.3%.

micros() counter will overflow after ~70 minutes, we must protect for that
to avoid an error or blip in the speed reading.

Measurement Type can be either Direct_Measurement, meaning actual
propeller measurement.  Or it can be Motor_Measurement.  Motor_Measurement
requires input of number of poles, and gear ratio.

*/

#define ENABLED 1
#define DISABLED 0

#define Serial_Debug DISABLED

#define BoardLED 13
#define RPM_Input_1 2
#define Direct_Measurement 1
#define Motor_Measurement 2
#define Measurement_Type Direct_Measurement
#define Motor_Poles 2
#define Gear_Ratio 2
#define PulsesPerRevolution 1


float rpm_measured = 0.0;						// Latest measured RPM value
volatile unsigned long trigger_time = 0;		// Trigger time of latest interrupt
volatile unsigned long trigger_time_old = 0;	// Trigger time of last interrupt
unsigned long last_calc_time = 0;				// Trigger time of last speed calculated
unsigned long timing = 0;						// Timing of last rotation
unsigned long timing_old = 0;					// Old rotation timing
bool timing_overflow_skip = true;				// Bit used to signal micros() timer overflowed
												// We set true to start so that we will throw out
												// the first data point collected after booting
												// because it is flaky.



unsigned long fast_loop_timer = 0;			// Time in microseconds of 1000hz control loop
unsigned long last_fast_loop_timer = 0;		// Time in microseconds of the previous fast loop
unsigned long fiftyhz_loop_timer = 0;		// Time in milliseconds of 50hz control loop
unsigned long last_fiftyhz_loop_timer = 0;	// Time in milliseconds of the previous loop, used to calculate dt
unsigned int fiftyhz_dt= 0 ;				// Time since the last 50 Hz loop
unsigned long tenhz_loop_timer = 0;			// Time in milliseconds of the 10hz control loop
unsigned long onehz_loop_timer = 0;			// Time in milliseconds of the 1hz control loop


unsigned int rotation_time;						// Time in microseconds for one rotation of rotor

void setup(){
   
   pinMode(RPM_Input_1, INPUT_PULLUP);
   attachInterrupt(0, rpm_fun, RISING);
   pinMode(BoardLED, OUTPUT);  
#if Serial_Debug == ENABLED
    serial_debug_init();
#endif
}

void loop(){

unsigned long timer = millis();						// Time in milliseconds of current loop


	if (( micros() - fast_loop_timer) >= 1000){	
		fast_loop_timer = micros();
		if (!micros_overflow()){
			fastloop();
		} else {
			trigger_time_old = 0;					//we will throw out whatever data we have
			trigger_time = 0;
			timing_overflow_skip == true;			//and the next one
		}
		last_fast_loop_timer = fast_loop_timer;
	}	
	
	if ((timer - fiftyhz_loop_timer) >= 20) {	
		last_fiftyhz_loop_timer = fiftyhz_loop_timer;
		fiftyhz_loop_timer = timer;
		fiftyhz_dt = last_fiftyhz_loop_timer - fiftyhz_loop_timer;
		mediumloop();	
	}
	
	if ((timer - tenhz_loop_timer) >= 10) {	
		tenhz_loop_timer = timer;
		slowloop();	
	}
	
	if ((timer - onehz_loop_timer) >= 1000) {	
		onehz_loop_timer = timer;
		superslowloop();	
	}	
}

void rpm_fun(){							//Each pulse, this interrupt function is run	
	trigger_time = micros();
}

void fastloop(){			//1000hz stuff goes here	
	if ( (trigger_time_old + (3 * timing)) < fast_loop_timer ){			// We have lost more than 1 expected pulse, start to decay the measured RPM
		rpm_measured -= 0.25;
		if (rpm_measured <0){
			rpm_measured = 0;
		}
	}
	
	if (trigger_time_old != trigger_time){				// We have new timing data to consume
		if (!timing_overflow_skip){						// If micros has not overflowed, we will calculate timing based on this data
			timing_old = timing;
			timing = trigger_time - trigger_time_old;
			rpm_measured = calc_rpm();
			digitalWrite(BoardLED, HIGH);
		} else {									
			timing_overflow_skip = false;				// If micros has overflowed, reset the skip bit since we have thrown away this bad data
		}
		trigger_time_old = trigger_time;				// In either case, we need to do this so we can look for new data		
	}	
}

void mediumloop(){			//50hz stuff goes here
	rpm_error = rpm_demand - rpm_measured;
	if (rpm_demand == 0){
		torque_demand = 0;
	} else {
		torque_demand = get_pi(rpm_error, fiftyhz_dt);
		torque_demand = constrain (torque_demand, 0, 1000);
	}
	digitalWrite(BoardLED, LOW);	
}

void slowloop(){			//10hz stuff goes here

}

void superslowloop(){		//1hz stuff goes here

#if Serial_Debug == ENABLED
	do_serial_debug();	
#endif

}

float calc_rpm(){
#if Measurement_Type == Direct_Measurement
	return (rpm_measured + (60000000.0/(float)timing)/PulsesPerRevolution)/2 ;				//Simple Low-pass Filter
#elif Measurement_Type == Motor_Measurement
	return (rpm_measured + (((60000000.0/(float)timing)/Gear_Ratio)/(Motor_Poles/2))/2;
#endif
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/*
The micros() timer will overflow roughly ever 70 minutes.  This is within the possible operating
time of a UAV so we must protect for it.  When the micros() timer overflows, we must
ignore any data collected during the period.
*/
///////////////////////////////////////////////////////////////////////////////////////////////////

bool micros_overflow(){

	if (micros() > last_fast_loop_timer) {			// Micros() have not overflowed because it has incremented since last fast loop
		return false;
	} else {
		return true;
	}
}

#if Serial_Debug == ENABLED

void serial_debug_init(){
    Serial.begin(9600);
    Serial.println("Tachometer Test");
    Serial.print("Startup Micros:");
    Serial.println(micros());
    Serial.print("Startup Timing:");
    Serial.println(timing);
}

void do_serial_debug(){	
	Serial.print ("RPM =");
	Serial.println(rpm_measured);
	Serial.print ("RPM Demand =");
	Serial.println(rpm_demand);
	Serial.print ("Error =");
	Serial.println(rpm_error);
	Serial.print ("Torque =");
	Serial.println (torque_demand);
}

#endif