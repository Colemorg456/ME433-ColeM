#ifndef UTILITIES__H__
#define UTILITIES__H__

//Contains all variables, arrays, and #defines for the full program

enum mode_t{IDLE, PWM, ITEST, HOLD, TRACK}; //Creates reference for the 5 operating modes of the PIC32

//Prototype functions
enum mode_t get_mode(); 
void set_mode(enum mode_t mode_choice);


#define BUF_SIZE 200 // size for buffer
#define EINTMAX 100 
#define PLOT_POINTS 100
#define CUBIC_POINTS 2000

//volatile enum mode_t temp_mode
volatile float REFarray[PLOT_POINTS];   // Stores reference values for current controller
volatile float current_array[PLOT_POINTS]; // Stores measured values for current controller

volatile int storing_data;
volatile int duty_cycle;

volatile float Kp, Ki;
volatile float pos_Kp, pos_Ki, pos_Kd;
volatile float input_angle;
volatile float global_ref_current;

volatile int N;
volatile float ref_pos_array[CUBIC_POINTS]; //Reference values for position controller
volatile float angle_array[CUBIC_POINTS]; //Stores actual angle values of the motor
volatile float current_angle;



#endif