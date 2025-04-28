#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19



int main()
{
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    //while (true) {
    //    printf("Hello, world!\n");
    //    sleep_ms(1000);
    //}

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("Start!\n");

    volatile float f1, f2;
    printf("Enter two floats to use:");
    scanf("%f %f", &f1, &f2);
    volatile float f_add, f_sub, f_mult, f_div;

    absolute_time_t t1 = get_absolute_time(); //get the time and then convert to microseconds
    uint64_t t11 = to_us_since_boot(t1);
    for (int i =0; i<1000; i++){ //Make loop to do operation 1000 times 
        f_add = f1+f2;
    }
    absolute_time_t t2 = get_absolute_time();
    uint64_t t22 = to_us_since_boot(t2); // get time again
    //printf("\n 1 %llu %llu",t11,t22);
    uint64_t abs_time = (t22-t11);
    uint64_t clk_cycles = (abs_time*150)/1000; //Multiply by 150 clock cycles per microsecond since we are in microseconds
    printf("\n Addition took %llu clock cycles \n",clk_cycles); // print

    //Repeat for each math operation
    t1 = get_absolute_time();
    t11 = to_us_since_boot(t1);
    for (int i = 0; i<1000; i++) {
       f_sub = f1 - f2;
    }
    t2 = get_absolute_time();
    t22 = to_us_since_boot(t2);
    abs_time = (t22 - t11);
    clk_cycles = (abs_time * 150)/1000;
    printf("\n Subtraction took %llu clock cycles\n",clk_cycles);
 
    t1 = get_absolute_time();
    t11 = to_us_since_boot(t1);
    for (int i = 0; i < 1000; i++) {
        f_mult = f1 * f2;
    }
    t2 = get_absolute_time();
    t22 = to_us_since_boot(t2);
    abs_time = (t22 - t11);
    clk_cycles = (abs_time*150)/1000;
    printf("\n Multiplication took %llu clock cycles\n",clk_cycles);
 
    t1 = get_absolute_time();
    t11 = to_us_since_boot(t1);
    for (int i = 0; i < 1000; i++) {
        f_div = f1 / f2;
    }
    t2 = get_absolute_time();
    t22 = to_us_since_boot(t2);
    abs_time = (t22-t11);
    clk_cycles =(abs_time*150)/ 1000;
    printf("\n Division took %llu clock cycles\n",clk_cycles);
}
