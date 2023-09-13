#include "motors.h"

#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define PIN_L_E (14)
#define PIN_R_E (15)
#define SLICE (7)

void motors_init(void)
{
    gpio_set_function(PIN_L_E, GPIO_FUNC_PWM);
    gpio_set_function(PIN_R_E, GPIO_FUNC_PWM);
    pwm_set_wrap(SLICE, 255);
    pwm_set_chan_level(SLICE, PWM_CHAN_A, 0);
    pwm_set_chan_level(SLICE, PWM_CHAN_B, 0);
    pwm_set_enabled(SLICE, true);
}

void motors_set(uint8_t left, uint8_t right)
{
    pwm_set_chan_level(SLICE, PWM_CHAN_A, left);
    pwm_set_chan_level(SLICE, PWM_CHAN_B, right);
}
