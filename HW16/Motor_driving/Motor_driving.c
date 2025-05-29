#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define B_PWM 21
#define B_PHASE 20
#define A_PWM 19
#define A_PHASE 18


int main()
{
    stdio_init_all();
    gpio_init(B_PHASE);
    gpio_set_dir(B_PHASE, GPIO_OUT);
    gpio_put(B_PHASE,1);

    gpio_init(A_PHASE);
    gpio_set_dir(A_PHASE, GPIO_OUT);
    gpio_put(A_PHASE,1);

    float div = 1; // must be between 1-255
    uint16_t wrap = 65535; // when to rollover, must be less than 65535

    gpio_set_function(B_PWM, GPIO_FUNC_PWM); // Set the LED Pin to be PWM
    uint slice_B = pwm_gpio_to_slice_num(B_PWM); // Get PWM slice number
    pwm_set_clkdiv(slice_B, div); // divider
    pwm_set_wrap(slice_B, wrap);
    pwm_set_enabled(slice_B, true); // turn on the PWM

    gpio_set_function(A_PWM, GPIO_FUNC_PWM); // Set the LED Pin to be PWM
    uint slice_A = pwm_gpio_to_slice_num(A_PWM); // Get PWM slice number
    pwm_set_clkdiv(slice_A, div); // divider
    pwm_set_wrap(slice_A, wrap);
    pwm_set_enabled(slice_A, true); // turn on the PWM

    int duty_cycle = 0;
    uint16_t init_speed = (uint16_t)(wrap*(10/100.0));
    pwm_set_gpio_level(A_PWM,init_speed);
    pwm_set_gpio_level(B_PWM,init_speed);

    while (true) {
        uint16_t PWM_speed;
        char c;
        printf("Enter + or - to change speed\n");
        scanf(" %c",&c);
        if (c == '+'){
            if(duty_cycle < 100){
                duty_cycle++;
            }
        }
        if (c=='-'){
            if(duty_cycle > -100){
                duty_cycle--;
            }
        }

        if (duty_cycle > 0){
            gpio_put(B_PHASE,1);
            gpio_put(A_PHASE,1);
            PWM_speed = (uint16_t)(wrap*(duty_cycle/100.0));
        }
        if(duty_cycle < 0){
            gpio_put(B_PHASE,0);
            gpio_put(A_PHASE,0);
            PWM_speed = (uint16_t)(wrap*(-duty_cycle/100.0));
        }
        if(duty_cycle == 0){
            PWM_speed = 0;
        }

        pwm_set_gpio_level(A_PWM,PWM_speed);
        pwm_set_gpio_level(B_PWM,PWM_speed);

        printf("The duty cycle is now: %d",duty_cycle);
        sleep_ms(100);
    }
}
