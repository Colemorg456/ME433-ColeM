#include <stdio.h>
#include "NU32DIP.h"
#include "encoder.h"
#include "utilities.h"
#include "ina219.h"
#include "currentcontrol.h"
#include "positioncontrol.h"

void __ISR(_TIMER_4_VECTOR, IPL6SOFT) PositionController(void){ //200 Hz Timer4 position controller ISR
    enum mode_t temp_mode = get_mode();
    static float eint = 0, e, eprev=0,edot; //Same initializations and switch-case format as the current controller ISR

    switch (temp_mode){
        case IDLE:{ //Do nothing if in IDLE, PWM, or ITEST
            break;
        }
        case PWM:{
            break;
        }
        case ITEST:{
            break;
        }
        case HOLD:{ //Hold mode will keep the motor at the same angle
            WriteUART2("a");     //Print the current encoder count of the motor
            while(!get_encoder_flag()){} //Wait to get value from the PICO
            set_encoder_flag(0);
            current_angle = get_encoder_count();
            current_angle = (current_angle*360.0)/1336; //Calculate the encoder count in degrees and store in a variable

            e = input_angle - current_angle; //caluclate error
            eint = eint + e; //calculate eint
            edot = e - eprev; //calculate derivative term
            eprev = e; //update eprev
            
            if (eint > EINTMAX) {         //integrator anti-windup
                eint = EINTMAX;
               } else if (eint < -EINTMAX) { 
                eint = -EINTMAX;
               }
            
            global_ref_current = ((pos_Kp*e) + (pos_Ki*eint) + (pos_Kd*edot)); // PID calculaion
            break;
        }
        case TRACK:{ //Track mode will take in data from the reference array to make the motor go in a certain trajectory
            static int array_num = 0; // initialize counter once

            WriteUART2("a");     //Print the current encoder count of the motor
            while(!get_encoder_flag()){} //Wait to get value from the PICO
            set_encoder_flag(0);

            angle_array[array_num] = ((get_encoder_count())*360.0)/1336; //Calculate the encoder count in degrees and store in a global variable

            e = ref_pos_array[array_num] - angle_array[array_num]; //caluclate error
            eint = eint + e; //calculate eint
            edot = e - eprev; //calculate derivative term
            eprev = e;
                
            if (eint > EINTMAX) {         //integrator anti-windup
                eint = EINTMAX;
            } else if (eint < -EINTMAX) { 
                eint = -EINTMAX;
            }
                
            global_ref_current = ((pos_Kp*e) + (pos_Ki*eint) + (pos_Kd*edot)); // PID calculaion

            array_num++; //Add to the index value
            
            if (array_num >= N) {
                input_angle = angle_array[array_num]; //Set final angle to the HOLD angle
                array_num = 0; //Reset index and other error variables
                eprev = 0;
                eint = 0;
                set_mode(HOLD); //Set mode ot HOLD when finished
            }
            break;
        }
        default:{
            NU32DIP_YELLOW = 0; //turn on error LED as default
            break;
        }
    }
    IFS0bits.T4IF = 0; //clear flag
}


void Timer4_Setup(void){
    T4CONbits.TCKPS = 6;  // set prescale to 6
    PR4 = 3750-1;  //Period for 200hz timer, 48Mhz/200hz = 240000  240000/64 scaler = 3750
    TMR4 = 0;      // Initial Timer4 value is 0
    T4CONbits.ON = 1; // when TMR4 4quals PR4 generate an interupt
  
    IEC0bits.T4IE = 1; // Enable and clear interupt
    IFS0bits.T4IF = 0;
    IPC4bits.T4IP = 6; //set prioirty to 6
    IPC4bits.T4IS = 0; // set sub-priority to zero
}

