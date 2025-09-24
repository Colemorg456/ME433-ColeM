//Cole Morgan ME333 Final Project: Motor Control
//
// The purpose of this project is to understand, build, and implement an intellgient motor driver which can follow
// a desired motor trajectory which is controlled with a Python menu on my PC

//Given Files:
//NU32DIP.h - contains all the SFRs and identifiers for the PIC32MX
//genref.py - for plotting trajectories
//encoder.c and .h - for reading the encoder using the PICO2
//ina219.c and .h - for reading the current sensor INA219 chip
//i2c_master_noint - for communicating with the current sensor using I2C

//Created Files:
//main.c - contains main program which houses the peripheral startups and the switch-cases for each menu option
//utilities.h - contains all global variables, enums, and #define numbers used in the project
//utilities.c - contains the functions to get and change the motor mode
//currentcontrol.h - contains the prototypes for the Timer2 interrupt startup and PWM startup
//currentcontrol.c - contains functions for timer2 and PWM startup and houses the current control ISR using Timer2
//positioncontrol.h - contains timer4 startup function prototype
//positioncontrol.c - contains the timer4 startup function and the position interrupt which sends the reference current
//to the current controller

//IMPORTANT --- GAIN INFORMATION
//After experimentation, the following gains work well for my system and my motor:
// Current Gains - Kp = 0.2  Ki = 0.001
// Position Gains (STEP) - Kp = 1100  Ki = 0  Kd = 1000
// Position Gains (CUBIC) - Kp = 1000  Ki = 0  Kd = 1000
// Units: Kp: [V/deg] Ki: [V/(deg*sec)] [(V*sec)/deg] Kd: [V/(deg*sec)]


#include <stdio.h>
#include "NU32DIP.h"
#include "encoder.h"
#include "utilities.h"
#include "ina219.h"
#include "currentcontrol.h"
#include "positioncontrol.h"

int main(){
    char buffer[BUF_SIZE];
    NU32DIP_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
    NU32DIP_GREEN = 1; //Turn LEDs off
    NU32DIP_YELLOW = 1;
    __builtin_disable_interrupts();   //initialize modules and peripherals here
        UART2_Startup(); 
        INA219_Startup();
        PWMStartup();
        Timer2_Setup();
        Timer4_Setup();
    __builtin_enable_interrupts();
    set_mode(IDLE); //Set mode IDLE by default

    while(1){
        NU32DIP_ReadUART1(buffer,BUF_SIZE); //Expect next character to be a menu command
        NU32DIP_GREEN = 1;                 // Clear the error LED
        
        switch (buffer[0]){
            case 'b':{ // Read the current from the INA219 in mA
                char msg[50];
                int mar = INA219_read_current(); // Get the mode the PIC is in 
                sprintf(msg,"%d\r\n",mar); //Write down the mode that the PIC is currently in
                NU32DIP_WriteUART1(msg); //Send to PIC
                break;
            }
            case 'c':{ //Read the encoder count
                WriteUART2("a");     //Print the current encoder count of the motor
                while(!get_encoder_flag()){} //Wait to get the value from the PICO
                set_encoder_flag(0);
                char m[50];
                int p = get_encoder_count(); //Get the encoder count
                sprintf(m,"%d\r\n",p);
                NU32DIP_WriteUART1(m); //Send to PIC
                break;
            }
            case 'd':{ //Read encoder count and convert to degrees
                WriteUART2("a");     //Print the current encoder count of the motor
                while(!get_encoder_flag()){} //Wait to get value from the PICO
                set_encoder_flag(0);
                char me[50];
                int p = get_encoder_count(); //Get the encoder count
                sprintf(me,"%f\r\n",(p*360.0)/1336); //Convert from counts to degrees
                NU32DIP_WriteUART1(me); //Send to PIC
                break;
            }
            case 'e':{
                WriteUART2("b"); //Tell the PIC to reset the encoder angle count
                break;
            }
            case 'q':{ // q is for quit, or return to IDLE mode
                set_mode(IDLE);
                break;
            }
            case 'r':{ //Get the mode of the PIC
                char msg[50];
                int pr = get_mode(); // Get the mode the PIC is in 
                sprintf(msg,"%d\r\n",pr); //Write down the mode that the PIC is currently in
                NU32DIP_WriteUART1(msg); //Send to PIC
                break;
            }
            case 'p':{ // For case P, set the mode to IDLE
                set_mode(IDLE);
                break;
            }
            case 'f':{ // Setting the PWM cycle
                NU32DIP_ReadUART1(buffer,BUF_SIZE); //Read the value inputted from the user in the python script
                sscanf(buffer, "%d", &duty_cycle);
                set_mode(PWM); //Set the mode to PWM
                break;
            }
            case 'g':{ // Setting the current gains
                NU32DIP_ReadUART1(buffer,BUF_SIZE); //Read the value inputted from the user in the python script
                sscanf(buffer, "%f", &Kp);
                NU32DIP_ReadUART1(buffer,BUF_SIZE); //Read the value inputted from the user in the python script
                sscanf(buffer, "%f", &Ki);
                break;
            }
            case 'h':{ // Get current gains
                char m[50];
                sprintf(m,"%f\r\n", Kp);
                NU32DIP_WriteUART1(m);
                sprintf(m,"%f\r\n", Ki);
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'k':{ //Perorm the ITEST to test gains
                char mssg[100];
                int i=0;
                set_mode(ITEST);
                while (storing_data == 1){
                    ;
                }
                sprintf(mssg, "%d\n", PLOT_POINTS);
                NU32DIP_WriteUART1(mssg);

                for (i=0; i<PLOT_POINTS; i++) { // send plot data to MATLAB
                    sprintf(mssg, "%f %f\r\n", REFarray[i], current_array[i]);
                    NU32DIP_WriteUART1(mssg);
                }
                break;
            }
            case 'i':{ // Setting the position gains
                NU32DIP_ReadUART1(buffer,BUF_SIZE); //Read the value inputted from the user in the python script
                sscanf(buffer, "%f", &pos_Kp);
                NU32DIP_ReadUART1(buffer,BUF_SIZE); 
                sscanf(buffer, "%f", &pos_Ki);
                NU32DIP_ReadUART1(buffer,BUF_SIZE); 
                sscanf(buffer, "%f", &pos_Kd);
                break;
            }
            case 'j':{ // Get position gains
                char m[50];
                sprintf(m,"%f\r\n", pos_Kp);
                NU32DIP_WriteUART1(m);
                sprintf(m,"%f\r\n", pos_Ki);
                NU32DIP_WriteUART1(m);
                sprintf(m,"%f\r\n", pos_Kd);
                NU32DIP_WriteUART1(m);
                break;
            }
            case 'l':{ //HOLD the motor to a certain angle after setting the gains
                NU32DIP_ReadUART1(buffer,BUF_SIZE); //Read the value inputted from the user in the python script
                sscanf(buffer, "%d", &input_angle);
                set_mode(HOLD); //Set the mode to HOLD
                break;
            }
            case 'm':{ //Create the reference step function and read it to an array
                int i=0;
                NU32DIP_ReadUART1(buffer,BUF_SIZE); //Read the value inputted from the user in the python script
                sscanf(buffer, "%d", &N);

                for (i=0; i<N; i++){
                    NU32DIP_ReadUART1(buffer,BUF_SIZE); //Read the value inputted from the user in the python script
                    sscanf(buffer, "%f", &ref_pos_array[i]);
                }
                break;
            }
            case 'n':{ //Create the reference cubic function and save to an array
                int i=0;
                NU32DIP_ReadUART1(buffer,BUF_SIZE); //Read the value inputted from the user in the python script
                sscanf(buffer, "%d", &N);

                for (i=0; i<N; i++){
                    NU32DIP_ReadUART1(buffer,BUF_SIZE); //Read the value inputted from the user in the python script
                    sscanf(buffer, "%f", &ref_pos_array[i]);
                }
                break;
            }
            case 'o':{ //Execute the trajectory by switching to TRACK, read the collected data to Python for plotting
                int i =0;
                set_mode(TRACK);
                while (get_mode() == TRACK){
                    Nop();
                }
                sprintf(buffer, "%d\r\n",N); //The ISR is running at 5ms .005 sec which can be translated to seconds
                NU32DIP_WriteUART1(buffer); //Send the time counts to python

                for (i=0; i < N; i++) { // send plot data to MATLAB
                    sprintf(buffer, "%f %f\r\n", ref_pos_array[i], angle_array[i]);
                    NU32DIP_WriteUART1(buffer);
                }
                break;
            }
            default:{ // Default case for when no other cases are activated
                NU32DIP_GREEN = 0;
                break;
            }
        }

    }

    return 0;
}

