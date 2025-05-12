import matplotlib.pyplot as plt
import numpy as np
import csv

avg_data = []

t = [] # column 0
data1 = [] # column 1
data2 = [] # column 2

with open("sigD.csv") as f:
    # open the csv file
    reader = csv.reader(f)
    for row in reader:
        # read the rows 1 one by one
        t.append(float(row[0])) # leftmost column
        data1.append(float(row[1])) # second column
        # data2.append(float(row[2])) # third column

#Moving average filter
X = 1000
for i in range(len(data1)):
    avg = 0
    if i<X:
        avg_data.append(data1[i])
    else:
        for j in range(X):
            avg = avg + data1[i-j]
        avg = avg/X
        avg_data.append(avg)


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
    fig.suptitle(f'Unfiltered vs. Filtered Data X={X}')

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
