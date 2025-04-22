#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "math.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   20
#define PIN_SCK  18
#define PIN_MOSI 19

static float t_sin = 0;  //Time increments for sin and trinagle wave
static float t_tri = 0; 

static inline void cs_select(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop"); // FIXME
}

static inline void cs_deselect(uint cs_pin) {
    asm volatile("nop \n nop \n nop"); // FIXME
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop"); // FIXME
}

void writeDAC(int,float);

int main()
{
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000); //1000*1000
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    float freq_sin = 2.0;    //2Hz sin wave
    float freq_tri = 1.0;    //1Hz triangle wave
    float update_rate = 100.0; //Updates the DAC at 100Hz, every 10 milliseconds
    float update_period = (1.0/update_rate); //calculats hte period for the update rate

    while (true) {
        // Generate a full sin wave cycle, 2pi radiands
        for(t_sin = 0; t_sin < 2*M_PI; t_sin += 2*M_PI*freq_sin*update_period) {
            // Get the value of sin at that time indice
            float sine_value = sin(t_sin);
            // Convert the sin value to voltage, move values between 0 and 2 not -1 and 1
            float sine_voltage = (sine_value + 1.0)*(3.3/2.0); 
            // Write to DAC at Channel A using the translated sin voltage value
            writeDAC(0, sine_voltage);
            
            // For every cycle of the sin wave we can also do the treeianlge wave
            //Update the triangle wave time indice and make sure it stays between 0 and 2
            t_tri += 2*freq_tri*update_period;
            if(t_tri >= 2.0){
                t_tri-=2.0;
            }
            
            //Make the wave go up when rising and down when falling
            float tri_value;
            if(t_tri < 1.0) {
                tri_value = t_tri;
            } else {
                tri_value = 2.0f - t_tri;
            }

            //Convert the value to voltage and read to channel B
            float tri_voltage = tri_value * 3.3f; 
            writeDAC(1, tri_voltage); 
            
            sleep_ms(10);
        }
    }
}

void writeDAC(int channel, float voltage){
    uint8_t data[2];
    int len = 2;
    if(voltage < 0){
        voltage = 0;
    }
    if(voltage > 3.3f){
        voltage = 3.3f;
    }
    //Convert the voltage to a 10-bit number using the equation from the datahseet
    uint16_t dac_value = ((voltage / 3.3)*1023);


    // Create an int called control bits for all the bit shifted needed to fix the voltage data
    uint8_t control_bits = 0;
    // Channel A or B
    if (channel == 1) {
        control_bits |= (1 << 7);
    } else {
        control_bits &= ~(1 << 7);
    }

    // Set next 3 to 1 always
    control_bits |= (1 << 6);
    control_bits |= (1 << 5);
    control_bits |= (1 << 4);

    // Get the top bits to move to the rest of the voltage bitoutput and combine into the data array
    uint8_t topdata_bits = (dac_value >> 6) & 0x0F;
    data[0] = control_bits | topdata_bits;

    //Do the same for the lower 6 bits
    uint8_t lower_data_bits = (dac_value & 0x3F);
    data[1] = lower_data_bits << 2;

    //Write the data to SPI
    cs_select(PIN_CS);
    spi_write_blocking(SPI_PORT, data, len);
    cs_deselect(PIN_CS);
}