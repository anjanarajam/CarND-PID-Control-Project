#include <iostream>
#include <vector>
#include "twiddle.h"


static constexpr int INITIAL_STEPS = 100;
static constexpr int MAX_STEPS = 500;

/*
 * Calculate the sum of delta parameters
 */
double Twiddle::CalculateSumDp(int num_params, std::vector<double> dp) {
	double sum_dp = 0;
	for (int i = 0; i < num_params; i++) {
		sum_dp += dp[i];
	}

	return sum_dp;
}

/*
 * Increment the count
 */
void Twiddle::IncrementSteps() {
	num_steps_++;
}

/*
 * Calculate cumulative error after an initial count
 */
void Twiddle::AccumulateError(double cte) {
	if (num_steps_ > INITIAL_STEPS) {
		error_ += cte * cte;
	}
}

/*
 * Calculate average of the error once it reaches the maximum count
 */
void Twiddle::CalculateAverageError() {
	if (num_steps_ > MAX_STEPS) {				
		average_error_ = error_ / static_cast<double>(num_steps_ - static_cast<long int>(INITIAL_STEPS));
		reached_count_ = true;
	}
}

/*
 * Twiddle algorithm to tune parameters automatically
 */
void Twiddle::PerformTwiddle(PID &pid_steer) {
	param_ = { pid_steer.Kp_, pid_steer.Ki_, pid_steer.Kd_};
	dp_ = { 0.05, 0.001, 0.05 };
	double threshold = 0.00001;	
	int num_params = 3;

	while (CalculateSumDp(num_params, dp_) > threshold) {
		switch (twiddle_state_) {
		case INIT:
			/* Set the best error , increment p by dp and set the state to increment */
			best_error_ = average_error_;
			param_[param_index_]  += dp_[param_index_];
			twiddle_state_ = INCREMENT;
			break;
		case INCREMENT:
			/* If increment is resulting in lower average error, then increment in the next index by dp */
			if (average_error_ < best_error_) {
				best_error_ = average_error_;
				dp_[param_index_] *= 1.1;
				/* Go to the next parameter index */
				GetNextParamIndex();
				param_[param_index_]  += dp_[param_index_];
			}
			/* Else decrement by dp by subtracting twice of dp to p and the state becomes decrement */
			else {
				param_[param_index_] -= 2 * dp_[param_index_];
				twiddle_state_ = DECREMENT;
			}
			break;
		case DECREMENT:
			/* If decrement is resulting in lower average error, then increment more next time by dp */
			if (average_error_ < best_error_) {
				best_error_ = average_error_;
				dp_[param_index_] *= 1.1;
		    /* Else reset p to its original value by adding dp and reduce the value of dp and then increment 
			in the next index by dp */
			} else {
				param_[param_index_]  += dp_[param_index_];
				dp_[param_index_] *= 0.9;
			}
			/* Go to the next parameter index */
			GetNextParamIndex();
			param_[param_index_] += dp_[param_index_];

			/* Set the state to increment*/
			twiddle_state_ = INCREMENT;
			break;
		default:
			break;
		}
	}

	/* Set the automatically tuned parameters to the coefficients */
	pid_steer.Kp_ = param_[0];
	pid_steer.Ki_ = param_[1];
	pid_steer.Kd_ = param_[2];
}

/*
 * Get to the next parameter index
 */
void Twiddle::GetNextParamIndex() {
	param_index_ = (param_index_ + 1) % param_.size();

	if (!param_index_) {
		iter_++;
	}
}

/*
 * Print the values
 */
void Twiddle::PrintValues() {
	std::cout << "Iteration: " << iter_
		<< " best error: " << best_error_
		<< " curren error: " << average_error_
		<< " current param: " << param_index_
		<< " current state: " << twiddle_state_ << std::endl;
}




