import matplotlib.pyplot as plt
import numpy as np
import csv

def fft_plotting(csv_file,title):
    
    t = [] # column 0
    data1 = [] # column 1
    data2 = [] # column 2

    with open(csv_file) as f:
        # open the csv file
        reader = csv.reader(f)
        for row in reader:
            # read the rows 1 one by one
            t.append(float(row[0])) # leftmost column
            data1.append(float(row[1])) # second column
        # data2.append(float(row[2])) # third column

    t =np.array(t)
    y =np.array(data1)
    Fs = (len(t))/(t[-1]) # sample rate
    n = len(y) # length of the signal
    k = np.arange(n)
    T = n/Fs
    frq = k/T # two sides frequency range
    frq = frq[range(int(n/2))] # one side frequency range
    Y = np.fft.fft(y)/n # fft computing and normalization
    Y = Y[range(int(n/2))]

    fig, (ax1, ax2) = plt.subplots(2, 1)
    fig.suptitle(title)
    ax1.plot(t,y,'b')
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Amplitude')
    ax2.loglog(frq,abs(Y),'b') # plotting the fft
    ax2.set_xlabel('Freq (Hz)')
    ax2.set_ylabel('|Y(freq)|')
    plt.show()


fft_plotting('sigA.csv','sigA transformed data')
fft_plotting('sigB.csv','sigB transformed data')
fft_plotting('sigC.csv','sigC transformed data')
fft_plotting('sigD.csv','sigD transformed data')