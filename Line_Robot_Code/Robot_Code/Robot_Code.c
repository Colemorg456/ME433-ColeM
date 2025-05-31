#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "font.h"
#include "hardware/pwm.h"

//Motor Defines
#define B_PWM 21
#define B_PHASE 20
#define A_PWM 19
#define A_PHASE 18

//OLED Screen defines
#define I2C_PORT2 i2c0
#define I2C_SDA2 16
#define I2C_SCL2 17
#define LED_PIN 25

void drawMessage(int x, int y, char * m);
void drawLetter(int x, int y, char c);

int main()
{
    stdio_init_all();

    // I2C Initialisation
    i2c_init(I2C_PORT2, 1000*1000);
    gpio_set_function(I2C_SDA2, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL2, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA2);
    gpio_pull_up(I2C_SCL2);

    //Heartbeat LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    bool LED_state = false;

    //OLED startup
    ssd1306_setup();
    ssd1306_clear();
    ssd1306_update();

    //Hbridge directional pins setup
    gpio_init(B_PHASE);
    gpio_set_dir(B_PHASE, GPIO_OUT);
    gpio_put(B_PHASE,1);
    gpio_init(A_PHASE);
    gpio_set_dir(A_PHASE, GPIO_OUT);
    gpio_put(A_PHASE,1);

    //PWM Initialization
    float div = 1; // must be between 1-255
    uint16_t wrap = 65535; // when to rollover, must be less than 65535
    gpio_set_function(B_PWM, GPIO_FUNC_PWM);
    uint slice_B = pwm_gpio_to_slice_num(B_PWM); // Get PWM slice number
    pwm_set_clkdiv(slice_B, div); // divider
    pwm_set_wrap(slice_B, wrap);
    pwm_set_enabled(slice_B, true); // turn on the PWM
    gpio_set_function(A_PWM, GPIO_FUNC_PWM);
    uint slice_A = pwm_gpio_to_slice_num(A_PWM); // Get PWM slice number
    pwm_set_clkdiv(slice_A, div); // divider
    pwm_set_wrap(slice_A, wrap);
    pwm_set_enabled(slice_A, true); // turn on the PWM
    int duty_cycle = 0;


    while (true) {
        //Heartbeat LED
        LED_state = !LED_state;
        gpio_put(LED_PIN, LED_state);

        //Motor Control
        uint16_t PWM_speed;
        //HERE RECEIVE INPUT from motor controller for PWM
        int c;
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
            gpio_put(A_PHASE,0);
            PWM_speed = (uint16_t)(wrap*(duty_cycle/100.0));
        }
        if(duty_cycle < 0){
            gpio_put(B_PHASE,0);
            gpio_put(A_PHASE,1);
            PWM_speed = (uint16_t)(wrap*(-duty_cycle/100.0));
        }
        if(duty_cycle == 0){
            PWM_speed = 0;
        }

        pwm_set_gpio_level(A_PWM,PWM_speed);
        pwm_set_gpio_level(B_PWM,PWM_speed);

        //OLED Display updating Duty cycles
        ssd1306_clear();
        int left_duty=75;
        int right_duty= 60;
        char left_msg[50];
        char right_msg[50];
        sprintf(left_msg,"Left duty cycle: %d",left_duty);
        sprintf(right_msg,"Right duty cycle: %d",right_duty);
        drawMessage(0,5,left_msg);
        drawMessage(0,15,right_msg);
        ssd1306_update();
        sleep_ms(1000);
    }
}
