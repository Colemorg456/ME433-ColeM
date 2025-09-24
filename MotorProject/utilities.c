#include "utilities.h"
#include "NU32DIP.h"

volatile enum mode_t mode; //Declare enum mode_t here

enum mode_t get_mode(){ //Return the current mode the motor is in by using the enum mode_t
    return mode;
}

void set_mode(enum mode_t mode_choice){ //Make the mode into the mode that the user inputs
    mode = mode_choice;
}

