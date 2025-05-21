#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "font.h"
#include "ssd1306.h"


// I2C defines
#define I2C_PORT i2c0
#define I2C_SDA 16
#define I2C_SCL 17
#define LED_PIN 25

#define MPU6050_addr 0x68

// config registers
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
// sensor data registers:
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H   0x41
#define TEMP_OUT_L   0x42
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48
#define WHO_AM_I     0x75

void writeData(unsigned char address, unsigned char reg, unsigned char value);
unsigned char readData(unsigned char address, unsigned char reg);
void MPU6050_startup();
void burst_read();
int16_t combine_bytes(uint8_t high, uint8_t low);
//void drawMessage(int x, int y, char * m);
void drawHorizontalLine(int x_origin, int y_origin, int length);
void drawVerticalLine(int x_origin, int y_origin, int length);

int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 1000*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    //Built-in LED heartbeat
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    bool LED_state = false;

    //OLED startup
    ssd1306_setup();
    ssd1306_clear();
    ssd1306_update();

    //MPU startup
    MPU6050_startup();
    unsigned char test = readData(MPU6050_addr,WHO_AM_I); //Check to see if its on
    printf("test is %d",test);
    if(test != 0x68){
        printf("TEST FAILED");
    }

    while (true) {
        //toggle the heartbeat
        LED_state = !LED_state;
        gpio_put(LED_PIN, LED_state);

        ssd1306_clear();// Clear OLED
        
        burst_read(); //Read data from MPU and plot on OLED

        ssd1306_update(); //Update teh values on the OLED screen
        sleep_ms(10); //10ms is 100Hz
    }
}

//i2c_write_blocking(I2C_PORT, ADDR, &reg, 14, true);  // true to keep master control of bus
//i2c_read_blocking(I2C_PORT, ADDR, &buf, 14, false);  // false - finished with bus

void writeData(unsigned char address, unsigned char reg, unsigned char value){
    unsigned char buf[2];
    buf[0] = reg;
    buf[1] = value;
    i2c_write_blocking(I2C_PORT, address, buf, 2, false);
}

unsigned char readData(unsigned char address, unsigned char reg){
    unsigned char buf;
    i2c_write_blocking(I2C_PORT, address, &reg, 1, true);  // true to keep master control of bus
    i2c_read_blocking(I2C_PORT, address, &buf, 1, false);  // false - finished with bus
    return buf;
} 

void  MPU6050_startup(){
    writeData(MPU6050_addr,PWR_MGMT_1,0x00); //Turn chip on
    writeData(MPU6050_addr,ACCEL_CONFIG,0x00); //set range to +-2g
    writeData(MPU6050_addr,GYRO_CONFIG,0x18); //set gyro to 2000 dps
}

void burst_read(){
    uint8_t buf[14]; //14 bit buffer for all the registers
    uint8_t reg = ACCEL_XOUT_H; //start with the top accel register
    i2c_write_blocking(I2C_PORT,MPU6050_addr,&reg, 1, true); 
    i2c_read_blocking(I2C_PORT,MPU6050_addr,buf, 14, false); 

    //combine the high and low bits of every x,y,z for each register
    int16_t ax = combine_bytes(buf[0],buf[1]);
    int16_t ay = combine_bytes(buf[2],buf[3]);
    int16_t az = combine_bytes(buf[4],buf[5]);
    int16_t temp = combine_bytes(buf[6],buf[7]);
    int16_t gx = combine_bytes(buf[8],buf[9]);
    int16_t gy = combine_bytes(buf[10],buf[11]);
    int16_t gz = combine_bytes(buf[12],buf[13]);

    //Print all the values with converted units, g, deg/sec, and Celsius
    printf("A: X=%.3f Y=%.3f Z=%.3f\n",ax*0.000061,ay*0.000061,az*0.000061);
    printf("G: X=%.2f Y=%.2f Z=%.2f\n",gx*0.00763,gy*0.00763,gz*0.00763);
    printf("T: %.2f \n", (temp/340.0)+36.53);

    //OLED DATA CODE
    //Scale everything to 16x16 so the data appears evenly on the screen
    int screen_range = 16;
    int s_ax = (int) ((-ax*0.000061)*(screen_range));
    int s_ay = (int) ((ay*0.000061)*(screen_range));
    int x_origin = 64;
    int y_origin = 16;

    drawHorizontalLine(x_origin, y_origin, s_ax);
    drawVerticalLine(x_origin, y_origin, s_ay);
}

int16_t combine_bytes(uint8_t high, uint8_t low){
    //Recombining the two 8 bit numbers from the MPU and putting it in one 16-bit int
    uint16_t high_shifted = (uint16_t)high<<8; //Shift the high byte left by 8 bits
    uint16_t combined = high_shifted | low; //OR to combine with the low byte
    return (int16_t)combined; //Return combined number
}

void drawHorizontalLine(int x_origin,int y_origin,int length){
    // Line goes right if positive
    // Going in a loop and drawing a pixel for every value in int length
    if(length >= 0){
        for (int i=0; i<length;i++) {
            int x = x_origin+i;
            int y = y_origin;
            ssd1306_drawPixel(x,y,1);
        }
    }
    // If line negative, go left
    else{
        for (int i=0; i>length;i--) {
            int x = x_origin + i;
            int y = y_origin;
            ssd1306_drawPixel(x, y, 1);  // draw pixel at (x, y)
        }
    }
}

//Same as the horizontal function except with the y data from MPU
void drawVerticalLine(int x_origin, int y_origin, int length) {
    if (length >= 0){
        for (int i=0; i<length;i++) {
            int x = x_origin;
            int y = y_origin+i;
            ssd1306_drawPixel(x,y,1);
        }
    }
    else {
        for (int i=0; i>length;i--) {
            int x = x_origin;
            int y = y_origin +i;
            ssd1306_drawPixel(x,y,1);
        }
    }
}

