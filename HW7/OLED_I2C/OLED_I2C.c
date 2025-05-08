#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "font.h"
#include "hardware/adc.h"


// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 16
#define I2C_SCL 17
#define LED_PIN 25

void drawMessage(int x, int y, char * m);
void drawLetter(int x, int y, char c);
float read_adc_voltage(); 

int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 1000*1000);//400*1000
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    //Heartbeat LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    bool LED_state = false;

    //ADC Startup
    adc_init();
    adc_gpio_init(26); //Init ADC0 at GP26
    adc_select_input(0);// chnnel 0

    //OLED startup
    ssd1306_setup();
    ssd1306_clear();
    ssd1306_update();

    while (true) {
        uint32_t t1 = to_us_since_boot(get_absolute_time()); //start timer
        //toggle the heartbeat
        LED_state = !LED_state;
        gpio_put(LED_PIN, LED_state);

        //Clear OLED
        ssd1306_clear();

        //ADC function
        float voltage = read_adc_voltage();

        // Sprintf to show on screen
        int i = 15;
        char fps_val[50];
        char message[50];
        sprintf(message, "ADC is %.3f ",voltage);
        drawMessage(20,5,message); // draw starting at x=20,y=10

        sleep_ms(10);

        //End timer and draw FPS counter to screen
        uint32_t t2 = to_us_since_boot(get_absolute_time());
        float fps=  1e6f/ (t2 - t1); //convert to micro seconds
        sprintf(fps_val, "FPS is %.2f", fps);
        drawMessage(0, 20,fps_val);

        ssd1306_update();
    }
}

//Function for drawing the message and moving over a space
void drawMessage(int x, int y, char * m){
    int i = 0;
    while(m[i]!=0){
        drawLetter(x+i*6,y,m[i]);
        i++;
    }
}

//Function for drawinf the 5x8 bit letters
void drawLetter(int x, int y, char c){
    int row, col;
    row = c - 0x20;
    for(col=0; col<5; col++){
        char byte = ASCII[row][col];

        for(int i=0;i<8;i++){
            char on_or_off = (byte>>i)&0b1;

            ssd1306_drawPixel(x+col,y+i,on_or_off);
        }
    }
}

//Reading ADC from POT function
float read_adc_voltage() {
    uint16_t adc_val = adc_read();
    return (3.3*adc_val)/4095.0;//convert to voltage
}