#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	p_error_ = 0;
	i_error_ = 0;
	d_error_ = 0;

	prev_cte_ = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
	
	/* Proportional error */
	p_error_ = cte;

	/* Derivative error */
	d_error_ = prev_cte_ - cte;
	prev_cte_ = cte;

	/* Integral error */
	i_error_ += cte;	
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
	double total_error = p_error_ * Kp_ + d_error_ * Kd_ + i_error_ * Ki_;

	return total_error;  // TODO: Add your total error calc here!
}
