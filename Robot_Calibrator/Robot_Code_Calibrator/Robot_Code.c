#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "font.h"
#include "hardware/pwm.h"
#include "cam.h"


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

//Motor Controller defines
#define MAX_DUTY 40
#define MIN_DUTY 10 //To keep motor from locking up
#define DEADBAND 4
#define LEFT_GAIN 0.98
#define RIGHT_GAIN 0.996
#define Kp 1
#define Kd 1

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
    gpio_put(A_PHASE,0);

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

    //Camera Startup
    init_camera_pins();

    while (true) {
        //Heartbeat LED
        LED_state = !LED_state;
        gpio_put(LED_PIN, LED_state);

        //Camera Code
        // setSaveImage(1);
        // while(getSaveImage()==1){}
        // convertImage();
        // int com = findLine(IMAGESIZEY/2); // calculate the position of the center of the ine
        // int line_position = com-(IMAGESIZEX/2);  // convert it to -40 to +40
        // //Saving the final result from the camera into line_position

        int line_position = 0;
        int left_duty,right_duty;
        int left_correction, right_correction;
        float correction;
        static int pos_prev;
        int edot;

        //Derivative Controller
        edot = line_position - pos_prev;
        pos_prev = line_position;
        correction = ((Kp*abs(line_position))+(Kd*edot));

        if(correction > MAX_DUTY){
                correction = MAX_DUTY;
            }

        if (abs(line_position) < DEADBAND){ //Robot is directly on the line
            //Go straight, max PWM
            left_duty = MAX_DUTY;
            right_duty = MAX_DUTY;
        }else if (line_position > 0){ //The line is going to the right
            //Slow the right wheel while keeping the left wheel going
            left_duty = MAX_DUTY;
            right_duty = MAX_DUTY-correction;
        }else {
            //Line is going to left, slow left wheel and speed up right wheel
            left_duty = MAX_DUTY-correction;
            right_duty = MAX_DUTY;
        }

        int fix_right_duty = (int)(RIGHT_GAIN*right_duty);
        int fix_left_duty = (int)(LEFT_GAIN*left_duty);
        if(fix_right_duty < MIN_DUTY){
            fix_right_duty = MIN_DUTY;
        }
        if(fix_left_duty < MIN_DUTY){
            fix_left_duty = MIN_DUTY;
        }


        //Motor Control
        uint16_t PWM_speed_right = (uint16_t)(wrap*(fix_right_duty/100.0));
        uint16_t PWM_speed_left = (uint16_t)(wrap*(fix_left_duty/100.0));
        gpio_put(B_PHASE,1);
        gpio_put(A_PHASE,0);
        pwm_set_gpio_level(B_PWM,PWM_speed_right);
        pwm_set_gpio_level(A_PWM,PWM_speed_left);

        //OLED Display updating Duty cycles
        ssd1306_clear();
        char left_msg[50];
        char right_msg[50];
        sprintf(left_msg,"Left duty cycle: %d",left_duty);
        sprintf(right_msg,"Right duty cycle: %d",right_duty);
        drawMessage(0,5,left_msg);
        drawMessage(0,15,right_msg);
        ssd1306_update();
        sleep_ms(1);
    }
}
