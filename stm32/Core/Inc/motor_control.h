#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"

#include <math.h>

#define PWM_MAX 4799

#define W_MAX 26.18f // rad/s ~ tốc độ chịu tải

#define MOTOR_FL 1
#define MOTOR_FR 2
#define MOTOR_BL 3
#define MOTOR_BR 4

// Khởi tạo motor (nếu cần)
void Motor_Init(void);

// Đặt tốc độ motor, giá trị speed từ -1.0 đến 1.0
void setMotorPWM(float speed, uint8_t motor_id);


static inline float normalize_speed(float w) {
    float s = w / W_MAX;   // scale về -1..1
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    return s;
}

#endif // MOTOR_CONTROL_H
