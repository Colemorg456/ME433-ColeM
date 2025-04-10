#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#define PIN_NUM 15

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("Start!\n");
    
    //Initialize LED Pin
    gpio_init(PIN_NUM);
    gpio_set_dir(PIN_NUM, GPIO_OUT);
    gpio_put(PIN_NUM,1);

    //Initilalize PB switch pin
    gpio_init(14);
    gpio_set_dir(14, GPIO_IN);
    gpio_set_pulls(14,true,false);


    //ADC Startup
    adc_init(); // init the adc module
    adc_gpio_init(26); // set ADC0 pin to be adc input instead of GPIO
    adc_select_input(0); // select to read from ADC0


    while (1) {

        while(gpio_get(14) == 1){
            ;
        }
            gpio_put(PIN_NUM,0);

            char message[100];
            scanf("%s", message);
            printf("message: %s\r\n",message);
            sleep_ms(50);

            for(int i=0; message;i++){
                uint16_t result = adc_read();
                printf("The ADC count is: %d\r\n",result);
            }
        }
    }
