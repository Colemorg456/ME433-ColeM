import matplotlib.pyplot as plt
import numpy as np
import csv

avg_data = []

t = [] # column 0
data1 = [] # column 1
data2 = [] # column 2

#Manually change which CSV youre looking at every time
with open("sigB.csv") as f:
    # open the csv file
    reader = csv.reader(f)
    for row in reader:
        # read the rows 1 one by one
        t.append(float(row[0])) # leftmost column
        data1.append(float(row[1])) # second column
        # data2.append(float(row[2])) # third column

weights = [
    -0.000489000512090750,
    -0.000367994568920778,
    -0.000000000000000001,
    0.000414738366032002,
    0.000618263687714292,
    0.000435505782930919,
    -0.000095357987436898,
    -0.000690091419915916,
    -0.000943420415233680,
    -0.000580766216327041,
    0.000304415417010411,
    0.001210927013704465,
    0.001481782607501560,
    0.000746611940714274,
    -0.000724923588226633,
    -0.002051264805900173,
    -0.002230823942649151,
    -0.000849253105406510,
    0.001472132744339777,
    0.003282996965425501,
    0.003166990666069338,
    0.000776086692370890,
    -0.002682627172426499,
    -0.004980112493722304,
    -0.004246625612751881,
    -0.000379378169870621,
    0.004527977381260360,
    0.007232960173703203,
    0.005408982969145276,
    -0.000541598113835718,
    -0.007251172007244224,
    -0.010183953863624258,
    -0.006581085864225707,
    0.002289051260620341,
    0.011260548026206307,
    0.014116737946890109,
    0.007683975938414390,
    -0.005404254040619825,
    -0.017393759762773096,
    -0.019709105259841184,
    -0.008639746516649073,
    0.011120623810626163,
    0.027822765809426744,
    0.028952267198041875,
    0.009378657015448568,
    -0.023390530092978510,
    -0.050581022518543066,
    -0.050457641461031606,
    -0.009845606911917194,
    0.067158798303421433,
    0.158392144466678098,
    0.231966197641967853,
    0.260137953196996174,
    0.231966197641967853,
    0.158392144466678098,
    0.067158798303421446,
    -0.009845606911917194,
    -0.050457641461031606,
    -0.050581022518543066,
    -0.023390530092978513,
    0.009378657015448570,
    0.028952267198041875,
    0.027822765809426747,
    0.011120623810626163,
    -0.008639746516649073,
    -0.019709105259841184,
    -0.017393759762773096,
    -0.005404254040619825,
    0.007683975938414391,
    0.014116737946890112,
    0.011260548026206310,
    0.002289051260620341,
    -0.006581085864225709,
    -0.010183953863624261,
    -0.007251172007244225,
    -0.000541598113835718,
    0.005408982969145279,
    0.007232960173703203,
    0.004527977381260360,
    -0.000379378169870621,
    -0.004246625612751881,
    -0.004980112493722304,
    -0.002682627172426501,
    0.000776086692370890,
    0.003166990666069338,
    0.003282996965425503,
    0.001472132744339779,
    -0.000849253105406511,
    -0.002230823942649155,
    -0.002051264805900173,
    -0.000724923588226633,
    0.000746611940714274,
    0.001481782607501561,
    0.001210927013704467,
    0.000304415417010411,
    -0.000580766216327041,
    -0.000943420415233679,
    -0.000690091419915916,
    -0.000095357987436898,
    0.000435505782930919,
    0.000618263687714292,
    0.000414738366032003,
    -0.000000000000000001,
    -0.000367994568920778,
    -0.000489000512090750,
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
    fig.suptitle(f'Unfiltered vs. FIR Sample Rate=3300 Cutoff=429Hz Bandwidth=99')

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
