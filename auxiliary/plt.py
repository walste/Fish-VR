import streamingpickle as spkl
import numpy as np
from matplotlib import pyplot as plt

'''
This program can be executed after the experiment run 'idle grating'. It makes three plots when given an spkl file name.
It plots alphas vs timestamps and draws lines where the CS / laser starts in one function (interactive)
plus: it plots the same only after removing alphas,timestamps +-90 degree,
and makes a bar diagram of tail movement in CS vs off time in the other.

'''

def plot_alphas_ts(timestamps, alphas):
    figure1 = plt.figure()
    plt.plot(timestamps, alphas)

    figure1.suptitle( 'idle_grating', fontsize=14, fontweight='bold')
    plt.xlabel('timestamps [100 s]')
    plt.ylabel('alpha [degree]')
    ax = figure1.add_subplot(111)
    plt.axhline(y=0, color = 'black')
    plt.text(-0.05, 0.33, 'R (f)',
    verticalalignment='bottom', horizontalalignment='left', transform=ax.transAxes,
    color= 'g', fontsize=12)
    plt.text(-0.05, 0.66, 'L (f)',
    verticalalignment='top', horizontalalignment='left', transform=ax.transAxes,
    color= 'g', fontsize=12)
    ymin = -100
    ymax = 100

    trials = ['t3', 't2', 't1', 10,9,8,7,6,5,4,3,2,1,0]
    #black marks intervals (beg. of CS) and red marks of laser
    for x in range(0, int(timestamps[-1])+2, 6000*6): # take * 6 instead of /4 for full 6 min intervals. timestamps.max +02 so that the last line is also drawn.
        plt.axvline(x, ymin, ymax, color = 'black', linewidth = 2. )
    for x in range((6000*6)+300, int(timestamps[-1])-((6000*6)*3), 6000*6): #+300 if the laser sets in after 3s i.e. 300, take * 6 instead of /4 for 6 min, -....*3 for 3 tests
        plt.axvline(x, ymin, ymax, color = 'red', linewidth = 1. )
    for x in range (0, int(timestamps[-1])+2, 6000*6): #timestamps.max +2 so that the last line is also drawn.
        d = trials.pop()
        ds = str (d)
        plt.text(1*x, ymax, '%s'  %ds, color= 'black')

    plt.show()

def plot_off_CS(true_b):
    off = 0
    CS = 0
    for element in true_b:
        if (element[0]%(6000*6) < 5.*100) and (-80<=element[1]<=-45 or 45<=element[1]<= 80):
         #if the remainder of ts/ITI < dur CS + alpha is in between  +-30 to +-80
            CS+=1
            print "cs", element
        elif (element[0]%(6000*6) > 5.*100) and (-80<=element[1]<=-45 or 45<=element[1]<= 80):
         #if the remainder of ts/ITI is outside of CS dur + alpha is in between  +-30 to +-80
            off +=1
        else:
            pass

    CS_per_s = CS/(5.*14.) #14 times the CS for 5s
    off_per_s = off/((78.*60.)-(5.*14.)) #total time 78min - total CS dur
    print CS_per_s, off_per_s

    fig = plt.figure()
    fig.suptitle('Tail flicks in CS and off')
    axx = fig.add_subplot(111)
    plt.ylabel('alpha +-45 to +-80 degree per s')

    y = [CS_per_s, off_per_s]
    N = len(y)
    x = range(N)
    width = 0.1
    plt.xticks(np.arange(2), ("CS", "off") )
    axx.bar(x,y,width,color='g')
    plt.show()


if __name__ == "__main__":
    data = str(raw_input("File: "))
    dataf = "../fishVR_data/" + data + ".spkl"

    f = open(dataf, "rb")
    fl = spkl.s_load(f)
    element = fl.next()

    b= []
    for element in fl:
            b.append(element)

    timestamps = [belem[0] for belem in b]
    alphas = [belem[1] for belem in b]

    #To remove wrong alphas (>90 or <-90)
    true_b = []
    for element in b:
        if element[1] >-90 and element[1] <90:
           true_b.append(element)

    true_timestamps = [belem[0] for belem in true_b]
    true_alphas = [belem[1] for belem in true_b]

    plot_alphas_ts(timestamps, alphas)

    timestamps,alphas = true_timestamps, true_alphas
    plot_alphas_ts(timestamps, alphas)

    plot_off_CS(true_b)








