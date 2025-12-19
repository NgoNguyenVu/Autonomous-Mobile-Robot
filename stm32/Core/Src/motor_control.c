#include "motor_control.h"

// Khai báo timer extern (phải khai báo extern trong main.h)
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

// Hàm khởi tạo motor (đặt PWM = 0 ban đầu)
void Motor_Init(void)
{
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
}

// Hàm set tốc độ motor
void setMotorPWM(float speed, uint8_t motor_id)
{
    if(speed > 1.0f) speed = 1.0f;
    if(speed < -1.0f) speed = -1.0f;

    int pwm_val = (int)(fabsf(speed) * PWM_MAX);
    if(pwm_val > PWM_MAX) pwm_val = PWM_MAX;

    GPIO_PinState dir_a_state, dir_b_state;

    // Xác định chiều quay qua direction pins
    if(speed >= 0)
    {
        dir_a_state = GPIO_PIN_SET;
        dir_b_state = GPIO_PIN_RESET;
    }
    else
    {
        dir_a_state = GPIO_PIN_RESET;
        dir_b_state = GPIO_PIN_SET;
    }

    switch(motor_id)
    {
        case MOTOR_FL:
            // Direction pins: PB12 (DIR_A), PB13 (DIR_B)
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, dir_a_state);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, dir_b_state);
            // PWM TIM9_CH1 (PA2)
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, pwm_val);
            break;

        case MOTOR_FR:
            // Direction pins: PB14 (DIR_A), PB15 (DIR_B)
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, dir_a_state);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, dir_b_state);
            // PWM TIM9_CH2 (PA3)
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, pwm_val);
            break;

        case MOTOR_BL:
            // Direction pins: PA4 (DIR_A), PA5 (DIR_B)
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, dir_a_state);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, dir_b_state);
            // PWM TIM10_CH1 (PB8)
            __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm_val);
            break;

        case MOTOR_BR:
            // Direction pins: PB0 (DIR_A), PB1 (DIR_B)
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, dir_a_state);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, dir_b_state);
            // PWM TIM11_CH1 (PB9)
            __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, pwm_val);
            break;

        default:
            // Motor không xác định => tắt motor
            break;
    }
}
