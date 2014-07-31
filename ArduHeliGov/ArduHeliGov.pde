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

#include <SCDriver.h> 
#include <config.h>

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

#if Governor_Mode == ENABLED
float rpm_demand;								// RPM setpoint after the soft-start ramp
float rpm_error;								// Current RPM error
int	torque_demand;								// % throttle to request from controller
long PID_integrator;							// Integrator for the PID loop
#endif

unsigned long fast_loop_timer = 0;			    // Time in microseconds of 1000hz control loop
unsigned long last_fast_loop_timer = 0;		// Time in microseconds of the previous fast loop
unsigned long fiftyhz_loop_timer = 0;		    // Time in milliseconds of 50hz control loop
unsigned long last_fiftyhz_loop_timer = 0;	    // Time in milliseconds of the previous loop, used to calculate dt
unsigned int fiftyhz_dt= 0 ;				    // Time since the last 50 Hz loop
unsigned long tenhz_loop_timer = 0;			// Time in milliseconds of the 10hz control loop
unsigned long onehz_loop_timer = 0;			// Time in milliseconds of the 1hz control loop
unsigned int rotation_time;					// Time in microseconds for one rotation of rotor

SCDriver SCOutput;								// Create Speed Control output object

void setup(){
   
   pinMode(RPM_Input_1, RPM_Input_1_Mode);
   pinMode(Arming_Pin, Arming_Mode);
   attachInterrupt(0, rpm_fun, RISING);
   SCOutput.attach(SCOutput_Pin);
   pinMode(BoardLED, OUTPUT);  
#if Serial_Debug == ENABLED
    serial_debug_init();
#endif
#if FrSky_Telemetry == ENABLED
    frsky_init();
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
    
    #if FrSky_Telemetry == ENABLED
        telemetry_frsky();
    #endif
	
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
    #if Governor_Mode == ENABLED
    rpm_demand = soft_start();
    rpm_error = rpm_demand - rpm_measured;
    if (rpm_demand == 0){
        torque_demand = 0;
    } else {
        torque_demand = get_pi(rpm_error, fiftyhz_dt);
        torque_demand = constrain (torque_demand, 0, 1000);
    }
    SCOutput.write(torque_demand);
    digitalWrite(BoardLED, LOW);
	#endif
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

#if Governor_Mode == ENABLED

float soft_start(){

    static int rsc_ramp;
    float rsc_output;
            
    if ( armed() ){
        if (rsc_ramp < RSC_Ramp_Up_Rate * 50){
            rsc_ramp++;
            rsc_output = (float)map(rsc_ramp, 0, RSC_Ramp_Up_Rate * 50, 0, Target_RPM);
        } else {
            rsc_output = (float)Target_RPM;
        }
        return rsc_output;
    } else {
        rsc_ramp--;				//Return RSC Ramp to 0 slowly, allowing for "warm restart"
        if (rsc_ramp < 0){
            rsc_ramp = 0;
        }
        rsc_output = 0; 		//Just to be sure RSC output is 0
        return rsc_output;
    }
}

bool armed(){
    if ( digitalRead(Arming_Pin) == LOW ){
        return true;
    }else{
        return false;
    }
}



float get_pi(float error, float dt){

    return get_p(error) + get_i(error, dt);
}


float get_p(float error) {

    return error * PID_kp;
}

float get_i(float error, float dt){

    if((PID_ki != 0) && (dt != 0)){
        PID_integrator += (error * PID_ki) * dt;
        if (PID_integrator < -PID_imax) {
            PID_integrator = -PID_imax;
        } else if (PID_integrator > PID_imax) {
            PID_integrator = PID_imax;
        }
        return PID_integrator;
    }
    return 0;
}

#endif

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

//////////////////////////////////////////////////////////////////////////////////////////////////
/*
FrSky Telemetry
Based on work for Multiwii by "Quadbow"

*/
//////////////////////////////////////////////////////////////////////////////////////////////////

#if FrSky_Telemetry == ENABLED

// Frame protocol
#define PROTOCOL_HEADER    0x5E
#define PROTOCOL_TAIL      0x5E

// Data Ids  (bp = before point; af = after point)
// Official data IDs

#define ID_RPM                0x03

void frsky_init(){
    Serial.begin(9600);
}

void telemetry_frsky(){
    static unsigned long lastTime;
    static unsigned int tele_loop;
    if ((millis() - lastTime) > 250) {
        // Data sent every 250ms
        lastTime = millis();
        tele_loop++;         
        // Data sent every 1s
        switch (tele_loop) {
            case 1:               
            break;
            case 2:
                send_RPM();               
            break;
            case 3:
            break;
            case 4:
               send_RPM();
               tele_loop = 0;
            break;
            default:
            break;
        }
    sendDataTail();         
    }
}

void inline write_FrSky8(uint8_t Data){
    Serial.write(Data);
}

void inline write_FrSky16(uint16_t Data){
    uint8_t Data_send;
    Data_send = Data;      
    check_FrSky_stuffing(Data_send);
    Data_send = Data >> 8 & 0xff;
    check_FrSky_stuffing(Data_send);
}
   
void inline check_FrSky_stuffing(uint8_t Data){                 //Byte Stuffing Necessary for FrSky Protocol
    if (Data == 0x5E){
        write_FrSky8(0x5D);
        write_FrSky8(0x3E);
    }
    else if (Data == 0x5D){
        write_FrSky8(0x5D);
        write_FrSky8(0x3D);
    }
    else{
        write_FrSky8(Data);         
    }
}

static void inline sendDataHead(uint8_t Data_id){
    write_FrSky8(PROTOCOL_HEADER);
    write_FrSky8(Data_id);
}

static void inline sendDataTail(void){
    write_FrSky8(PROTOCOL_TAIL);      
}

// RPM
void inline send_RPM(void){
    unsigned long Data_RPM = long(rpm_measured);

    sendDataHead(ID_RPM);
    write_FrSky16(Data_RPM);
}

#endif //FrSky_Telemetry