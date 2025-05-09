#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define LEDPin 25 // the built in LED on the Pico
#define servoPIN 10 //The servos directional pin

void set_angle(uint gpio, uint16_t wrap, float angle);

int main(){

    stdio_init_all();

    //Setting the servo to 50Hz, so every 20ms the servo is updated
    gpio_set_function(servoPIN, GPIO_FUNC_PWM); // Set the LED Pin to be PWM
    uint slice_num = pwm_gpio_to_slice_num(servoPIN); // Get PWM slice number
    float div = 50; // must be between 1-255
    pwm_set_clkdiv(slice_num, div); // divider
    uint16_t wrap = 60000; // when to rollover, must be less than 65535
    pwm_set_wrap(slice_num, wrap);
    pwm_set_enabled(slice_num, true); // turn on the PWM
  
    pwm_set_gpio_level(servoPIN, wrap / 2); // set the duty cycle to 50%

    while (true) {
        for (float angle=0; angle<=180;angle+=0.72){ //5sec/.02 ms = 250 steps, 250/180 = .72 deg
            set_angle(servoPIN,wrap,angle);
            sleep_ms(20);
        }
        for (float angle=180; angle>=0;angle-=0.72){
            set_angle(servoPIN,wrap,angle);
            sleep_ms(20);
        }
    }
}

void set_angle(uint gpio, uint16_t wrap, float angle) {
    float pulse = 0.5+ (angle/180.0)*2.0; //Create the pulse based off the angle, 5ms 0deg -- 2.5ms 180deg
    uint16_t level = (pulse/ 20.0) * wrap; //calculate duty cycle
    pwm_set_gpio_level(gpio, level); //Update the duty cycle
}