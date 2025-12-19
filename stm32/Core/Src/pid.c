#include "pid.h"

void PID_Init(PID_Controller *pid)
{
    // === ĐÂY LÀ CÁC GIÁ TRỊ BAN ĐẦU ĐỂ BẮT ĐẦU TUNING ===

	    pid->Kp = 3.0f;
	    pid->Ki = 1.0f;
	    pid->Kd = 0.35f;

    // Giới hạn output bằng vận tốc tối đa của động cơ
    pid->output_limit_max = W_MAX;
    pid->output_limit_min = -W_MAX;

    // Chống "Integral Windup" - rất quan trọng!
    pid->integral_limit_max = 10.0f;
    pid->integral_limit_min = -10.0f;

    // Reset các biến
    pid->integral = 0.0f;
    pid->prev_measured = 0.0f;
}

// Trong file pid.c

float PID_Compute(PID_Controller *pid, float setpoint, float measured_value, float dt)
{
    // Nếu mục tiêu là dừng, hãy reset integral để tránh bị trôi
    if (fabsf(setpoint) < 0.01f) {
        pid->integral = 0.0f;
    }

    float error = setpoint - measured_value;

    // Thành phần P
    float p_out = pid->Kp * error;

    // --- LOGIC ANTI-WINDUP THÔNG MINH ---
    // Chỉ tích lũy sai số khi output chưa bị bão hòa (chưa ở mức tối đa/tối thiểu)
    // Hoặc khi sai số đang giúp kéo output ra khỏi vùng bão hòa.
    float potential_output = p_out + (pid->Ki * pid->integral); // Output tiềm năng trước khi thêm D

    if ( (potential_output < pid->output_limit_max && potential_output > pid->output_limit_min) ||
         (error > 0 && potential_output < pid->output_limit_min) ||
         (error < 0 && potential_output > pid->output_limit_max) )
    {
        pid->integral += error * dt;
        // Giới hạn integral để an toàn
        if (pid->integral > pid->integral_limit_max) pid->integral = pid->integral_limit_max;
        if (pid->integral < pid->integral_limit_min) pid->integral = pid->integral_limit_min;
    }
    // ------------------------------------

    float i_out = pid->Ki * pid->integral;

    // Thành phần D
    float derivative = (measured_value - pid->prev_measured) / dt;
    float d_out = -pid->Kd * derivative;

    // Tính tổng output
    float output = p_out + i_out + d_out;

    // Giới hạn output cuối cùng
    if (output > pid->output_limit_max) output = pid->output_limit_max;
    if (output < pid->output_limit_min) output = pid->output_limit_min;

    // Lưu lại sai số cho lần tính tiếp theo
    pid->prev_measured = measured_value;

    return output;
}
