/*
 * motor_model.c
 *
 *  Created on: Jan 28, 2025
 *      Author: Cetin24
 */

#include <stdint.h>
#include "motor_model.h"

volatile float dbg_voltage = 0.0f;
volatile float dbg_current = 0.0f;
volatile float dbg_omega = 0.0f;

static float voltage = 0.0f;
static float current = 0.0f; // motor current (A)
static float omega = 0.0f; // angular velocity (rad/s)

float omega_arr[500];

#define INV_L (1.0f / L)  // Instead of: / L
#define INV_J (1.0f / J)  // Instead of: / J

void update_motor_model(float applied_voltage)
{
	voltage = applied_voltage;

	// compute derivatives
	float di_dt = (voltage - R * current - Ke * omega) * INV_L;
	float domega_dt = (Kt * current - B * omega) * INV_J;

	// calculate next states using forward Euler approx.
	current = current + Ts * di_dt;
	omega = omega + Ts * domega_dt;

	// to plot
	dbg_current = current;
	dbg_omega = omega;
}


