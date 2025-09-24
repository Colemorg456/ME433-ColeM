#include "NU32DIP.h"
#include "currentcontrol.h"
#include "utilities.h"
#include "ina219.h"

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) CurrentController(void){ //5Khz Current Control ISR using Timer2
    enum mode_t temp_mode = get_mode(); //Initialize counters, reference currents, error values
    static int current_val = 0;
    static int counter = 0;
    static int reference_current = 200;
    static float eint=0, e, u;

    switch (temp_mode){ //Create a switch that uses the Enum value to get the current mode of the Motor
        case IDLE:{
            OC1RS = 0; // When IDLE, turn the PWM off
            LATBbits.LATB10 = 0;
            break;
        }
        case PWM:{ //When the mode is in PWM, change the duty cycle and direciton
            if (duty_cycle > 100) duty_cycle = 100; //Make sure the duty cycle is between -100 and 100
            if (duty_cycle < -100) duty_cycle = -100;

            if (duty_cycle >= 0){ //If negative, set direction bit to counter-clockwise. If positive, set direction bit to clockwise
                LATBbits.LATB10 = 1; //CLOCKWISE
                OC1RS = ((int)((PR3 + 1) * (duty_cycle/100.0))); //Do the math to set the PWM duty cycle according to the users input
            } else{
                LATBbits.LATB10 = 0; //COUNTERCLOCKWISE
                OC1RS = ((int)((PR3 + 1) * (-duty_cycle/100.0)));
            }
            break;
        }
        case ITEST:{
            if (counter == 0){ //At the beginning of the counter, turn storing_data on so the PIC knows that
                storing_data = 1; // it should be waiting to send data
            }

            if (counter % 25 == 0 && counter != 0) { //Change PWM at 0,25,50,75,99
                reference_current = -reference_current; // Toggle between +200 and -200 mA
            }

            current_val = INA219_read_current(); //Make a variable that stores the current value at the present moment

            REFarray[counter] = reference_current; //Put the current data point of the reference and experimental values
            current_array[counter] = current_val;  // into an array
            
            //PID Control Algorithim
            e = reference_current - current_val; //caluclate error
            eint = eint + e; //calculate eint
            if (eint > EINTMAX) {         //integrator anti-windup
                eint = EINTMAX;
               } else if (eint < -EINTMAX) { 
                eint = -EINTMAX;
               }
            
            u = ((Kp*e) + (Ki*eint)); // PI calculaion

            if (u > 100) { //Ensure that u stays between -200 and 200 mA
                u = 100;
            } else if (u < -100) {
                u = -100; 
            }

            if (u > 0){       //When current is positive, set direction bit to go clockwise and recaculate the PWM
                LATBbits.LATB10 = 1; //CLOCKWISE
                OC1RS = (unsigned int)((u * PR3) / 100.0); // Scale u from [0, 200] to [0, PR3]
            } else{                 //When current negative, set direction bit counter-clockwise
                LATBbits.LATB10 = 0; //COUNTERCLOCKWISE
                OC1RS = (unsigned int)((-u * PR3) / 100.0); // Scale -u from [0, 200] to [0, PR3]
            }

            counter++; //Iterate the counter
            if (counter == PLOT_POINTS){ //When you reach the amount of data points you need, reset the counter variables,
                counter = 0;             // tell the PIC it can store the data, and set mode to IDLE
                storing_data = 0;
                eint = 0;
                set_mode(IDLE);
            }
            break;
        }
        case HOLD:{
            current_val = INA219_read_current(); //Make a variable that stores the current value at the present moment
            
            //PID Control Algorithim
            e = global_ref_current - current_val;
            eint = eint + e; //calculate eint
            if (eint > EINTMAX) {         //integrator anti-windup
                eint = EINTMAX;
               } else if (eint < -EINTMAX) { 
                eint = -EINTMAX;
               }
            
            u = ((Kp*e) + (Ki*eint)); // PI calculaion

            if (u > 100) { //Ensure that u stays between -200 and 200 mA
                u = 100;
            } else if (u < -100) {
                u = -100; 
            }

            if (u > 0){       //When current is positive, set direction bit to go clockwise and recaculate the PWM
                LATBbits.LATB10 = 1; //CLOCKWISE
                OC1RS = (unsigned int)((u * PR3) / 100.0); // Scale u from [0, 200] to [0, PR3]
            } else{                 //When current negative, set direction bit counter-clockwise
                LATBbits.LATB10 = 0; //COUNTERCLOCKWISE
                OC1RS = (unsigned int)((-u * PR3) / 100.0); // Scale -u from [0, 200] to [0, PR3]
            }
                break;
        }
        case TRACK:{
            current_val = INA219_read_current(); //Make a variable that stores the current value at the present moment
            
            //PID Control Algorithim
            e = global_ref_current - current_val;
            eint = eint + e; //calculate eint
            if (eint > EINTMAX) {         //integrator anti-windup
                eint = EINTMAX;
               } else if (eint < -EINTMAX) { 
                eint = -EINTMAX;
               }
            
            u = ((Kp*e) + (Ki*eint)); // PI calculaion

            if (u > 100) { //Ensure that u stays between -200 and 200 mA
                u = 100;
            } else if (u < -100) {
                u = -100; 
            }

            if (u > 0){       //When current is positive, set direction bit to go clockwise and recaculate the PWM
                LATBbits.LATB10 = 1; //CLOCKWISE
                OC1RS = (unsigned int)((u * PR3) / 100.0); // Scale u from [0, 200] to [0, PR3]
            } else{                 //When current negative, set direction bit counter-clockwise
                LATBbits.LATB10 = 0; //COUNTERCLOCKWISE
                OC1RS = (unsigned int)((-u * PR3) / 100.0); // Scale -u from [0, 200] to [0, PR3]
            }
                break;
        }
        default:{
            OC1RS = 0; // By default, set the MODE to IDLE
            LATBbits.LATB10 = 0;
            NU32DIP_GREEN = 0;
            break;
        }
    }
    IFS0bits.T2IF = 0; //Clear the interrupt flag
  }

void PWMStartup(){
    RPB7Rbits.RPB7R = 0b0101;// Set pin RPB7 to be OC1
    T3CONbits.TCKPS = 0;     // Timer3 prescaler 1
    PR3 = 2400-1;              // period to get 20khz PWM --- PIC32 is 48Mhz so divide by 20khz
    TMR3 = 0;                // initial TMR3 count is 0
    T3CONbits.ON = 1;        // turn on Timer3
    
    OC1CONbits.OCTSEL = 1; //Set timer3 for OC1
    OC1CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
    OC1RS = (PR3+1)/2;             // duty cycle = OC1RS = (PR3+1)/2 for 25% duty cycle
    OC1R = (PR3+1)/2;              // initialize before turning OC1 on; afterward it is read-only
    OC1CONbits.ON = 1;       // turn on OC1
}

void Timer2_Setup(void){
    TRISBbits.TRISB10 = 0; //Set the direction bit (Pin B10) to be an output and connect it to the H-Bridge
    LATBbits.LATB10 = 0;

    T2CONbits.TCKPS = 0;  // set prescale to 0
    PR2 = 9600-1;  //Period for 5khz timer, 48Mhz/5khz = 9600
    TMR2 = 0;      // Initial Timer2 value is 0
    T2CONbits.ON = 1; // when TMR2 equals PR2 generate an interupt
  
    IEC0bits.T2IE = 1; // Enable and clear interupt
    IFS0bits.T2IF = 0;
    IPC2bits.T2IP = 5; //set prioirty to 5
    IPC2bits.T2IS = 0; // set sub-priority to zero
}
