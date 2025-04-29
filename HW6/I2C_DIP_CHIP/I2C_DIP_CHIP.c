#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 16
#define I2C_SCL 17

#define LED_PIN 25

#define MCP23008_ADDR 0x20 //Chip address since all A pins are grounded, see datasheet page 7
#define IODIR 0x00 //Register addresses
#define GPIO  0x09
#define OLAT  0x0A

void setPin(unsigned char address, unsigned char reg, unsigned char value);
unsigned char readPin(unsigned char address, unsigned char reg);

int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    //gpio_pull_up(I2C_SDA); using physical 10k pull-ups
    //gpio_pull_up(I2C_SCL);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

    //Built-in LED heartbeat
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    bool LED_state = false;

    setPin(MCP23008_ADDR,IODIR, 0x7F); //Set GP7 to be an output

    while (true) {
        printf("Hello, world!\n");

        //Read the current state of all the pins on the chip
        unsigned char current_state = readPin(MCP23008_ADDR,GPIO);

        if ((current_state & 0x1)==0){ //if GP0 is low/button is pressed
            setPin(MCP23008_ADDR,OLAT, 0x80);  // We want the LED to be on, 0x80 puts a 1 for GP7
        }
        else{
            setPin(MCP23008_ADDR,OLAT, 0x00); // Else GP7 stays off
        }

        //toggle the heartbeat
        LED_state = !LED_state;
        gpio_put(LED_PIN, LED_state);

        //fast refresh rate so the light responds quickly to the button
        sleep_ms(50);
    }
}

void setPin(unsigned char address, unsigned char reg, unsigned char value){
    unsigned char buf[2];
    buf[0] = reg;
    buf[1] = value;
    i2c_write_blocking(I2C_PORT, address, buf, 2, false);
}

unsigned char readPin(unsigned char address, unsigned char reg){
    unsigned char buf;
    i2c_write_blocking(I2C_PORT, address, &reg, 1, true);  // true to keep master control of bus
    i2c_read_blocking(I2C_PORT, address, &buf, 1, false);  // false - finished with bus
    return buf;
} 

