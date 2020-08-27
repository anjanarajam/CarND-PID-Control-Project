enum state {
	INIT,
	INCREMENT,
	DECREMENT
};

class Twiddle {
private:
	void GetNextParamIndex();

	int num_steps_{};
	double error_{};
	double average_error_{};
	double best_error_{};
	int twiddle_state_{ INIT };
	int param_index{ 0 };
	std::vector<double> param_{};
	std::vector<double> dp_{};
	long int iter{ 0 };

public:
	void CalculateSumDp(int num_params, std::vector<double> dp);
	void IncrementSteps();
	void AccumulateError(double cte);
	void CalculateAverageError();
	void PerformTwiddle(PID& pid_steer);
	void PrintValues();
	void ResetSimulator();
};