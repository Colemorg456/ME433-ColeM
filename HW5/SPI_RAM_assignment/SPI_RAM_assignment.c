#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "math.h"

#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS_DAC   20  //Specify CS for the DAC
#define PIN_CS_RAM   15  //RAM CS using GP15
#define PIN_SCK  18
#define PIN_MOSI 19

static float t_sin = 0;
static float t_tri = 0;


static inline void cs_select(uint cs_pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect(uint cs_pin) {
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 1);
    asm volatile("nop \n nop \n nop");
}

void writeDAC(int, float);

//Struct for the ram chip that holds the CS pin
typedef struct{
    uint cs_pin;
}spi_ram_t;

//RAM initiliazaiion function that sets the RAM chip to sequential mode
void spi_ram_init(spi_ram_t *ram);
// Read and write functions that use the struct and address to combine the bytes to read or write a lfoat
void spi_ram_write_float(spi_ram_t *ram, uint16_t addr, float val);
float spi_ram_read_float(spi_ram_t *ram, uint16_t addr);
//The pointer ram points to the struct
// The pointer addr points to a 16-bit unsigned int to hold the address
// val is a stored float vlaue

int main() {
    stdio_init_all();
    
    // SPI initialization
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS_DAC, GPIO_FUNC_SIO);
    //Initialize 2nd CS Pin for the RAM
    gpio_set_function(PIN_CS_RAM, GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    gpio_set_dir(PIN_CS_DAC, GPIO_OUT);
    gpio_put(PIN_CS_DAC, 1);
    //Again
    gpio_set_dir(PIN_CS_RAM, GPIO_OUT);
    gpio_put(PIN_CS_RAM, 1);

    //Set the ram pointer and pass its address
    spi_ram_t ram;
    ram.cs_pin = PIN_CS_RAM;
    spi_ram_init(&ram);

    //Load 1000 floats into a sin wave using method from HW4
    for (int i=0; i<1000;i++){
        // Deterine the angle based on the period 
        float angle = (2.0*i*M_PI)/1000.0;
        // Get the value of sin at that time indice
        float sin_val = sin(angle);
        // Convert the sin value to voltage, move values between 0 and 2 not -1 and 1
        float sin_voltage =(sin_val+1.0)*(3.3/2.0);
        spi_ram_write_float(&ram,i*4, sin_voltage); // 4 bytes per float
    }

    while (true) {
        static int index = 0;
        float value = spi_ram_read_float(&ram, index*4); // Read back float
        writeDAC(0, value); // Write to DAC channel A

        //Index to indicate when the sin wave has gone through a full period
        index++;
        if(index>=1000) {
            index=0;
        }
        // Make a 1millisecond delay for a 1Hz sin wave
        sleep_ms(1);
    }
}


// Set RAM to sequential mode
void spi_ram_init(spi_ram_t *ram) {
    uint8_t buffer[2];
    buffer[0] = 0x01; //Instruction Bit 0b00000001
    buffer[1] = 0x40; //Set to sequential mode 0b01000000

    // Lower the CS pin to send the data and then raise it again
    cs_select(ram-> cs_pin);
    spi_write_blocking(SPI_PORT, buffer, 2);
    cs_deselect(ram-> cs_pin);
}

//Functino to write a float to the RAM at a specific address
void spi_ram_write_float(spi_ram_t *ram,uint16_t addr,float val){
    //Floatint struct to enable bit shifting with ints
    union Floatint{
        float f;
        uint32_t i;
    }num;

    //Store the float in the val
    num.f = val;
    uint8_t buffer[7];
    buffer[0] = 0x02; //Write command 0b00000010

    buffer[1] =(addr>> 8)&0xFF;//2nd Byte stores bits 15-8 of the address
    buffer[2] = addr & 0xFF; //3rd Byte stores bits 7-0 of the address

    //Do the same thing again, split a 32-bit float into 4 seperate bytes by shifting
    buffer[3] = (num.i>>24) &0xFF;
    buffer[4] = (num.i>>16) &0xFF;
    buffer[5] = (num.i>>8) &0xFF;
    buffer[6] = num.i &0xFF;

    //Put the pin low to send the data and then put it back high
    cs_select(ram->cs_pin);
    spi_write_blocking(SPI_PORT,buffer,7);
    cs_deselect(ram->cs_pin);
}

// Read a float from RAM at address
float spi_ram_read_float(spi_ram_t *ram, uint16_t addr) {
    uint8_t buffer[4];

    //Makes another buffer command that send the read instruction and splits up the
    // 16 bits into 2 bytes
    uint8_t read_command[3];
    read_command[0] = 0x03; //Read 0b00000011
    read_command[1] = (addr >> 8) & 0xFF;
    read_command[2] = addr & 0xFF;

    //Put the pin low to send the read command while also sending blank data
    cs_select(ram->cs_pin);
    spi_write_blocking(SPI_PORT,read_command, 3);
    spi_read_blocking(SPI_PORT,0x00,buffer, 4);
    cs_deselect(ram->cs_pin);

    // Use the union again and more tedious bitshifting to reconstruct the float
    union Floatint{
        float f;
        uint32_t i;
    } num;
    num.i = (buffer[0]<<24)| (buffer[1]<< 16)| (buffer[2] <<8) | (buffer[3]);
    return num.f;
}


//WriteDAC from previous assignment
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
    cs_select(PIN_CS_DAC);
    spi_write_blocking(SPI_PORT, data, len);
    cs_deselect(PIN_CS_DAC);
}