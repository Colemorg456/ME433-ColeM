/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/adc.h"

#define FLAG_VALUE 123
#define LEDpin 14

float read_adc_voltage(); 
volatile static float voltage;

void core1_entry() {
    //ADC Startup
    adc_init();
    adc_gpio_init(26); //Init ADC0 at GP26
    adc_select_input(0);// channel 0

    //Turn on LED pin
    gpio_init(LEDpin);
    gpio_set_dir(LEDpin, GPIO_OUT);
    gpio_put(LEDpin,0);


    multicore_fifo_push_blocking(FLAG_VALUE); //Telling core 0 were ready to receive

    while(1){

        uint32_t g = multicore_fifo_pop_blocking();//waiting for a command from core 0

        if(g == 0){
            voltage = read_adc_voltage(); //Comman 0, read the voltage
        }
        if(g == 1){
            gpio_put(LEDpin,1); //LED On when 1
        }
        if(g == 2){
            gpio_put(LEDpin,0); //LED off when 2
        }
        multicore_fifo_push_blocking(FLAG_VALUE); //Say that core 1 is done

    }
}

int main() {
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    printf("Hello, multicore!\n");

    /// \tag::setup_multicore[]

    multicore_launch_core1(core1_entry);


    // Wait for it to start up

    uint32_t g = multicore_fifo_pop_blocking();

    if (g != FLAG_VALUE)
        printf("Hmm, that's not right on core 0!\n");
    else {
        multicore_fifo_push_blocking(FLAG_VALUE);
        printf("It's all gone well on core 0!\n");
    }

    while(1){
        int msg;
        printf("Give me a command 0,1,2: ");
        scanf("%d",&msg); //scan commmand from user

        multicore_fifo_push_blocking(msg); // send command to core1

        uint32_t result = multicore_fifo_pop_blocking(); // wait for result from ADC

        if(result==FLAG_VALUE){
            if(msg==1){
                printf("LED turned on\n");
            }
            if(msg==2){
                printf("LED turned off\n");
            }
            if(msg==0){ //Print ADC Voltage
                printf("The ADC voltage is: %.2f\n",voltage);
            }//Note: There is some glitchy thing going on with the cores due to the volatile int
        }//Where it takes 2 passes for the ADC to work correcrtly, I tried to fix it but it does not seem like an easy fix.
    }
}
    /// \end::setup_multicore[]

//Reading ADC function
float read_adc_voltage(){
    uint16_t adc_val = adc_read();
    return (3.3*adc_val)/4095.0;//convert to voltage
}