#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

/**
* Constructor
*/
PID::PID() {}

/**
 * Destructor.
 */
PID::~PID() {}

/*
 * Initialize PID coefficients (and errors, if needed)
 */
void PID::Init(double Kp, double Ki, double Kd) {
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	p_error_ = 0;
	i_error_ = 0;
	d_error_ = 0;

	prev_cte_ = 0;
}

/**
 * Update PID errors based on cte.
 */
void PID::UpdateError(double cte) {

	/* Proportional error */
	p_error_ = cte;

	/* Derivative error */
	d_error_ = prev_cte_ - cte;
	prev_cte_ = cte;

	/* Integral error */
	i_error_ += cte;	
}

/*
 * TODO:Calculate and return the total error
 */
double PID::TotalError() {

	double total_error = p_error_ * Kp_ + d_error_ * Kd_ + i_error_ * Ki_;

	return total_error;  
}
