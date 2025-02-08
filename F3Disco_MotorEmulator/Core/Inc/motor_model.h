/*
 * motor_model.h
 *
 *  Created on: Jan 28, 2025
 *      Author: Cetin24
 */

#ifndef INC_MOTOR_MODEL_H_
#define INC_MOTOR_MODEL_H_

#include "math.h"

// Motor parameters
#define R 	1.0f      	// Resistance (ohms)
#define L 	0.5f    	// Inductance (henries)
#define Ke 	0.01f	    // Back EMF constant
#define Kt 	0.01f    	// Torque constant
#define J 	0.01f     	// Rotor inertia
#define B 	0.1f    	// Damping coefficient
#define Ts 	0.00001f 	// 100kHz timer

void update_motor_model(float applied_voltage);

extern volatile float dbg_voltage;
extern volatile float dbg_current;
extern volatile float dbg_omega;

#endif /* INC_MOTOR_MODEL_H_ */
