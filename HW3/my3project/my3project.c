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

        while(gpio_get(14) == 1){ //While the button IO pin is high, do nothing
            ;
        }
        gpio_put(PIN_NUM,0); //When the button is pressed, turn the LED off


        int sample_num=0;
        printf("Enter the number of samples 1-100: ");
        scanf("%d",&sample_num); //Collect the number of ADC count samples

        if(sample_num<1){ //Make sure the ADC sample counts are between 1-100
            sample_num = 1;
        }
        if(sample_num > 100){
            sample_num = 100;
        }
        sleep_ms(50);

        for(int i=0; i < sample_num;i++){ //Cycle through each ADC count 
            uint16_t result = adc_read();
            float voltage = ((result/(4096.0-1.0))*3.30); //Convert ADC to voltage
            printf("The ADC count is: %.4f\r\n",voltage); //print
            sleep_ms(10); //Wait 100hz or 10 milliseconds
        }
        gpio_put(PIN_NUM,1); //Turn LED back on to indicate the program has finished
        sample_num = 0;
        printf("Press the button again to restart");
    }
return 0;
}
