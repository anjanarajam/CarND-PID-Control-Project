#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;

	p_error = 0;
	i_error = 0;
	d_error = 0;

	prev_cte = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
	
	/* Proportional error */
	p_error = cte;

	/* Derivative error */
	d_error = prev_cte - cte;
	prev_cte = cte;

	/* Integral error */
	i_error += cte;	
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
	double total_error = p_error * Kp + d_error * Kd + i_error * Ki;

	return total_error;  // TODO: Add your total error calc here!
}

double PID::GetControl() {
	double control_value = -Kp * p_error - Kd * d_error - Ki * i_error;
	if (control_value < -1) {
		control_value = -1;
	}
	else if (control_value > 1) {
		control_value = 1;
	}
	return control_value;
}
