import matplotlib.pyplot as plt
import numpy as np
import csv

avg_data = []

t = [] # column 0
data1 = [] # column 1
data2 = [] # column 2

#Manually change which CSV youre looking at every time
with open("sigC.csv") as f:
    # open the csv file
    reader = csv.reader(f)
    for row in reader:
        # read the rows 1 one by one
        t.append(float(row[0])) # leftmost column
        data1.append(float(row[1])) # second column
        # data2.append(float(row[2])) # third column

weights = [
    0.000442748077627967,
    0.000425664753561347,
    -0.000000000000000001,
    -0.000479733994137977,
    -0.000559784810924451,
    -0.000085517644955029,
    0.000585456868478028,
    0.000810175815145715,
    0.000238533210758852,
    -0.000740976138023620,
    -0.001200800863158765,
    -0.000508019749281505,
    0.000914578144102027,
    0.001747738828576387,
    0.000947428508297495,
    -0.001061431988030377,
    -0.002457343444490025,
    -0.001613234239149761,
    0.001123416214120970,
    0.003325033614045911,
    0.002564265177419206,
    -0.001028646248419960,
    -0.004334828555108424,
    -0.003862614275367662,
    0.000689849061862343,
    0.005459687726368346,
    0.005577454416227773,
    -0.000000000000000007,
    -0.006662656062156138,
    -0.007794219092104368,
    -0.001177926820668910,
    0.007898754420288073,
    0.010634306258093170,
    0.003033970321646526,
    -0.009117498037370322,
    -0.014297493628052606,
    -0.005863807588901689,
    0.010265876686100720,
    0.019159934906081033,
    0.010198524694656761,
    -0.011291593055154307,
    -0.026032192859423497,
    -0.017172622616130940,
    0.012146333640773896,
    0.036995167820411831,
    0.029843031786925441,
    -0.012788841019330238,
    -0.059237891707163211,
    -0.060447775229432753,
    0.013187568326341490,
    0.143410519498553130,
    0.268318727212215336,
    0.319745407356513744,
    0.268318727212215336,
    0.143410519498553130,
    0.013187568326341490,
    -0.060447775229432753,
    -0.059237891707163211,
    -0.012788841019330238,
    0.029843031786925441,
    0.036995167820411838,
    0.012146333640773896,
    -0.017172622616130940,
    -0.026032192859423497,
    -0.011291593055154307,
    0.010198524694656763,
    0.019159934906081037,
    0.010265876686100720,
    -0.005863807588901690,
    -0.014297493628052611,
    -0.009117498037370326,
    0.003033970321646526,
    0.010634306258093173,
    0.007898754420288078,
    -0.001177926820668910,
    -0.007794219092104371,
    -0.006662656062156143,
    -0.000000000000000007,
    0.005577454416227773,
    0.005459687726368343,
    0.000689849061862343,
    -0.003862614275367663,
    -0.004334828555108427,
    -0.001028646248419959,
    0.002564265177419206,
    0.003325033614045913,
    0.001123416214120970,
    -0.001613234239149762,
    -0.002457343444490029,
    -0.001061431988030377,
    0.000947428508297495,
    0.001747738828576387,
    0.000914578144102027,
    -0.000508019749281506,
    -0.001200800863158765,
    -0.000740976138023620,
    0.000238533210758852,
    0.000810175815145715,
    0.000585456868478028,
    -0.000085517644955029,
    -0.000559784810924451,
    -0.000479733994137977,
    -0.000000000000000001,
    0.000425664753561347,
    0.000442748077627967,
]

#FIR Filtering
for i in range(len(data1)):
    total=0
    for j in range(len(weights)): #Loop over the weights and apply every weight to average
        k = i-j
        if k >= 0:
            total = total + (weights[j]*data1[k])
    avg_data.append(total)

def fft_plotting(t, data1, avg_data):
    
    t =np.array(t)
    y =np.array(data1)
    y2 = np.array(avg_data)

    Fs = (len(t))/(t[-1]) # sample rate
    n = len(y) # length of the signal
    k = np.arange(n)
    T = n/Fs
    frq = k/T # two sides frequency range
    frq = frq[range(int(n/2))] # one side frequency range
    Y = np.fft.fft(y)/n # fft computing and normalization
    Y = Y[range(int(n/2))]

    FY = np.fft.fft(y2)/n # fft computing and normalization
    FY = FY[range(int(n/2))]

    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(10,10))
    fig.suptitle(f'Unfiltered vs. FIR Sample Rate=2500 Cutoff=400Hz Bandwidth=75')

    ax1.plot(t,y,color='black',label='Original Data')
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Amplitude')
    ax1.legend()

    ax2.plot(t,avg_data,color='red',label='Filtered Data')
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Amplitude')
    ax2.legend()

    ax3.loglog(frq,abs(Y),color='black',label='Original Data') # plotting the fft
    ax3.set_xlabel('Freq (Hz)')
    ax3.set_ylabel('|Y(freq)|')

    ax4.loglog(frq,abs(FY),color='red',label='Filtered Data') # plotting the fft
    ax4.set_xlabel('Freq (Hz)')
    ax4.set_ylabel('|Y(freq)|')
    plt.tight_layout()
    plt.show()

fft_plotting(t, data1, avg_data)
