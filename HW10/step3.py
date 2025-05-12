import matplotlib.pyplot as plt
import numpy as np


import csv

t = [] # column 0
data1 = [] # column 1
data2 = [] # column 2

with open('sigC.csv') as f:
    # open the csv file
    reader = csv.reader(f)
    for row in reader:
        # read the rows 1 one by one
        t.append(float(row[0])) # leftmost column
        data1.append(float(row[1])) # second column


print(t[-1])
print(len(t))