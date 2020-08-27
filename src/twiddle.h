#include "PID.h"

enum state {
	INIT,
	INCREMENT,
	DECREMENT
};

class Twiddle {
private:
	void GetNextParamIndex();

	long int num_steps_{};
	double error_{};
	double average_error_{};
	double best_error_{};
	int twiddle_state_{ INIT };
	double param_index_{ 0 };
	std::vector<double> param_{};
	std::vector<double> dp_{};
	long int iter_{ 0 };

public:
	double CalculateSumDp(int num_params, std::vector<double> dp);
	void IncrementSteps();
	void AccumulateError(double cte);
	void CalculateAverageError();
	void PerformTwiddle(PID& pid_steer);
	void PrintValues();

	bool reached_count_{ false };
};