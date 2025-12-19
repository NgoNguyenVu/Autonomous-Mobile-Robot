	#ifndef PID_H
	#define PID_H

	#include "motor_control.h" // Để lấy hằng số W_MAX

	typedef struct {
		// Hằng số PID (bạn sẽ tinh chỉnh sau)
		float Kp;
		float Ki;
		float Kd;

		// Các biến tính toán
		float integral;
		float prev_measured;

		// Giới hạn để chống "Integral Windup" và giới hạn output
		float output_limit_max;
		float output_limit_min;
		float integral_limit_max;
		float integral_limit_min;

	} PID_Controller;

	void PID_Init(PID_Controller *pid);
	float PID_Compute(PID_Controller *pid, float setpoint, float measured_value, float dt);

	#endif
