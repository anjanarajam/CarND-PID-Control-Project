#include <iostream>
#include <vector>
#include "twiddle.h"
#include "PID.h"

static constexpr int INITIAL_STEPS = 100;

void CalculateSumDp(int num_params, std::vector<double> dp) {
	double sum_dp = 0;
	for (int i = 0; i < num_params; i++) {
		sum_dp += dp[i];
	}

	return sum_dp;
}

void IncrementSteps() {
	n_steps_++;
}

void AccumulateError(double cte) {
	if (n_steps_ > INITIAL_STEPS) {
		error_ += cte * cte;
	}
}

void CalculateAverageError() {
	average_error_ = error_ / (n_steps_ - INITIAL_STEPS);
}


void PerformTwiddle(PID &pid_steer) {

	double best_error;
	param_ = { pid_steer.kp_, pid_steer.Kd_, pid_steer.Ki_};
	dp_ = {1, 1, 1};
	double threshold = 0.00001;	
	int num_params = 3;

	while (add_dp(num_params, dp) > threshold) {		
		std::cout << "iterator:" << "best_error:" << iter << best_error;

		switch (twiddle_state_) {
		case INIT:
			/* Set the best error , increment p by dp and set the state to increment */
			best_error_ = average_error_;
			param_[param_index_] = += dp_[param_index_];
			twiddle_state_ = INCREMENT;
			break;
		case INCREMENT:
			/* If increment is resulting in lower average error, then increment in the next index by dp */
			if (average_error_ < best_error_) {
				best_error_ = average_error_;
				dp_[param_index_] *= 1.1;
				GetNextParamIndex();
				param_[param_index_] = += dp_[param_index_];
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
				param_[param_index_] = += dp_[param_index_];
				dp_[twiddle_param_] *= 0.9;
			}

			GetNextParamIndex();
			param_[param_index_] = += dp_[param_index_];

			/* Set the state to increment*/
			twiddle_state_ = DECREMENT;
			break;
		default:
			break;
		}
	}
}

void GetNextParamIndex() {
	param_index_ = (param_index_ + 1) % param_.size();

	if (!param_index_) {
		iter_++;
	}
}

void PrintValues() {
	std::cout << "Iteration: " << iter_
		<< " best error: " << best_error_
		<< " curren error: " << average_error_
		<< " current param: " << param_
		<< " current state: " << twiddle_state_ << std::endl;
}




