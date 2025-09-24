#ch28.py for the motor project menu and plotting for ITEST and TRACK

import serial
import matplotlib.pyplot as plt 
from statistics import mean 

def read_plot_matrix():
    n_str = ser.read_until(b'\n');  # get the number of data points to receive
    n_int = int(n_str) # turn it into an int
    print('Data length = ' + str(n_int))
    ref = []
    data = []
    data_received = 0
    while data_received < n_int:
        dat_str = ser.read_until(b'\n');  # get the data as a string, ints seperated by spaces
        dat_f = list(map(float,dat_str.split())) # now the data is a list
        ref.append(dat_f[0])
        data.append(dat_f[1])
        data_received = data_received + 1
    meanzip = zip(ref,data)
    meanlist = []
    for i,j in meanzip:
        meanlist.append(abs(i-j))
    score = mean(meanlist)
    t = [i * 0.005 for i in range(len(ref))] # Translate the index to time by multiplying by the current ISR
    plt.plot(t,ref,'r*-',t,data,'b*-')
    plt.title('Score = ' + str(score))
    plt.ylabel('value')
    plt.xlabel('Time (sec)')
    plt.show()

from genref import genRef

ser = serial.Serial('/dev/tty.usbserial-10',230400)
print('Opening port: ')
print(ser.name)

has_quit = False
# menu loop
while not has_quit:
    print('PIC32 MOTOR DRIVER INTERFACE')
    # display the menu options; this list will grow
    print('\tb: Read Current Sensor (mA) \tc: Read Encoder (counts) \td: Read Encoder (deg) \te: Reset Encoder \tf: Set PWM (-100 to 100) ')
    print('\tg: Set Current Gains \th: Get Current Gains \tk: Test Current Gains \tp: Unpower the Motor \tq: Quit \tr: Get Mode') # '\t' is a tab)
    print('\ti: Set Position Gains \tj: Get Position Gains \tl: Go to angle (deg)') # '\t' is a tab)
    print('\tm: Load Step Trajectory \tn: Load Cubic Trajectory \to: Execute Trajectory') # '\t' is a tab)

    # read the user's choice
    selection = input('\nENTER COMMAND: ')
    selection_endline = selection+'\n'
    
    # send the command to the PIC32
    ser.write(selection_endline.encode()); # .encode() turns the string into a char array
    
    # take the appropriate action
    # there is no switch() in python, using if elif instead
    if (selection == 'd'): # Read encoder (degrees)
        n_str = ser.read_until(b'\n'); # Read the value we found in Main until you reach \n
        n_float = float(n_str)
        n_int = int(n_float) # turn it into an int
        print('The motor angle is ' + str(n_int) + ' degrees\n') # Print the motor angle in degrees

    elif (selection == 'b'): # Read current sensor (mA)
        n_str = ser.read_until(b'\n'); # Read the value we found in Main until you reach \n
        n_int = int(n_str) # turn it into an int
        print('The motor current is ' + str(n_int)+ ' mA\n') # Print the motor angle in counts

    elif (selection == 'c'): # Read the encoder counts
        n_str = ser.read_until(b'\n'); # Read the value we found in Main until you reach \n
        n_int = int(n_str) # turn it into an int
        print('The motor angle is ' + str(n_int) + ' counts\n') # Print the motor angle in counts

    elif (selection == 'e'): #Reset the motor angle
        print('Motor angle reset\n') # verify to user that the motor angle has been reset

    elif (selection == 'p'): #Unpower the motor
        print('Motor has been set to IDLE\n') # verify to user that the motor angle has been reset

    elif (selection == 'q'): # Exit
        print('Exiting client')
        has_quit = True; # exit client
        ser.close() # be sure to close the port

    elif (selection == 'f'): # Prompt user to enter PWM value from -100 to 100
        n_str = input('Enter PWM -100 to 100: ') # get the number to send
        n_int = int(n_str) # turn it into an int
        ser.write((str(n_int)+'\n').encode()); # send the number
        print('PWM changed to: ' + str(n_int)) # print it to the screen to double check

    elif (selection == 'g'): # Set current gains
        n_str = input('Set Gain Kp: ') # get the number to send
        n_float = float(n_str) # turn it into a float
        ser.write((str(n_float)+'\n').encode()); # send the number
    
        n_str = input('Set Gain Ki: ') # get the number to send
        n_float = float(n_str) # turn it into a float
        ser.write((str(n_float)+'\n').encode()); # send the number

    elif (selection == 'h'): # Get current gains
        n_str = ser.read_until(b'\n'); # Read the value we found in Main until you reach \n
        n_float = float(n_str) # turn it into a float
        print('The Kp gain is: ' + str(n_float) + '\n') # Print the motor angle in counts

        n_str = ser.read_until(b'\n'); # Read the value we found in Main until you reach \n
        n_float = float(n_str) # turn it into a float
        print('The Ki gain is: ' + str(n_float) + '\n') # Print the motor angle in counts

    elif (selection == 'k'): # Testcurrent gains
        read_plot_matrix()
    
    elif (selection == 'i'): # Set position gains
        n_str = input('Set Gain Kp: ') # get the number to send
        n_float = float(n_str) # turn it into a float
        ser.write((str(n_float)+'\n').encode()); # send the number
    
        n_str = input('Set Gain Ki: ') # get the number to send
        n_float = float(n_str) # turn it into a float
        ser.write((str(n_float)+'\n').encode()); # send the number
    
        n_str = input('Set Gain Kd: ') # get the number to send
        n_float = float(n_str) # turn it into a float
        ser.write((str(n_float)+'\n').encode()); # send the number
    
    elif (selection == 'j'): # Get Position gains
        n_str = ser.read_until(b'\n'); # Read the value we found in Main until you reach \n
        n_float = float(n_str) # turn it into a float
        print('The Kp gain is: ' + str(n_float) + '\n') # Print the motor angle in counts

        n_str = ser.read_until(b'\n'); # Read the value we found in Main until you reach \n
        n_float = float(n_str) # turn it into a float
        print('The Ki gain is: ' + str(n_float) + '\n') # Print the motor angle in counts

        n_str = ser.read_until(b'\n'); # Read the value we found in Main until you reach \n
        n_float = float(n_str) # turn it into a float
        print('The Kd gain is: ' + str(n_float) + '\n') # Print the motor angle in counts
    
    elif (selection == 'l'): # Go to angle in degrees
        n_str = input('Input desired angle (degrees): ') # get the number to send
        n_int = int(n_str) # turn it into a int
        ser.write((str(n_int)+'\n').encode()); # send the number
    
    elif (selection == 'm'): #Generate step trajectory
        ref = genRef('step')
        print(len(ref))
        t = range(len(ref))
        plt.plot(t,ref,'r*-')
        plt.ylabel('angle in degrees')
        plt.xlabel('index')
        plt.show()
        # send
        ser.write((str(len(ref))+'\n').encode())
        for i in ref:
            ser.write((str(i)+'\n').encode())
    
    elif (selection == 'n'): #Generate Cubic Trajectory
        ref = genRef('cubic')
        print(len(ref))
        t = range(len(ref))
        plt.plot(t,ref,'r*-')
        plt.ylabel('angle in degrees')
        plt.xlabel('index')
        plt.show()
        # send
        ser.write((str(len(ref))+'\n').encode())
        for i in ref:
            ser.write((str(i)+'\n').encode())

    elif (selection == 'o'): #Execute the desired trajectory
        read_plot_matrix()

    elif (selection == 'r'): #See what mode the PIC is in 
        n_str = ser.read_until(b'\n'); # Read the value we found in Main until you reach \n
        n_int = int(n_str) # turn it into an int
        try:
            mode_dict = {
                0: 'IDLE',
                1: 'PWM',
                2: 'ITEST',
                3: 'HOLD',
                4: 'TRACK'
            }
            mode_name = mode_dict.get(n_int, 'UNKNOWN')
            print(f'The PIC32 controller mode is currently: {mode_name} \n')# Print the motor angle in degrees

        except:
            print(f'error')
    else:
        print('Invalid Selection ' + selection_endline)



