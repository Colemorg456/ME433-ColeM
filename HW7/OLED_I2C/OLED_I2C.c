#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "font.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 16
#define I2C_SCL 17
#define LED_PIN 25

void drawMessage(int x, int y, char * m);
void drawLetter(int x, int y, char c);

int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    //Heartbeat LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    bool LED_state = false;

    //OLED startup
    ssd1306_setup();
    ssd1306_clear();
    ssd1306_update();

    while (true) {
        //toggle the heartbeat
        LED_state = !LED_state;
        gpio_put(LED_PIN, LED_state);

        //Clear OLED
        ssd1306_clear();

        // Sprintf to show on screen
        int i = 15;
        char message[50];
        sprintf(message, "My var = %d", i);
        drawMessage(20,10,message); // draw starting at x=20,y=10  

        //Push to OLED
        ssd1306_update();

        sleep_ms(1000);
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