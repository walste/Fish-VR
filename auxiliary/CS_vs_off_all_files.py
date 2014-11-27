import matplotlib.pyplot as plt
import numpy as np
import streamingpickle as spkl
import math
from scipy import stats
'''
This program reads all spkl files, then sorts the alphas in to CS and off, and takes the absolute values
of all alphas. The upper cut off is 90 (set to 90) and the lower cut off is 5 (set to 0).
The average area is calculated by taking the mean value of all alphas (because the timestamp
is always 1).
Then it plots the mean areas and the SEMs.
'''
spkl_files = ['../fishVR_data/20141112_120446results_file.spkl', '../fishVR_data/20141112_193447results_file.spkl',
              '../fishVR_data/20141113_153643results_file.spkl', '../fishVR_data/20141114_100453results_file.spkl',
              '../fishVR_data/20141118_091259results_file.spkl', '../fishVR_data/20141118_150451results_file.spkl',
              '../fishVR_data/20141120_100137results_file.spkl', '../fishVR_data/20141121_100241results_file.spkl',
              '../fishVR_data/20141121_154836results_file.spkl', '../fishVR_data/20141125_091234results_file.spkl',
              '../fishVR_data/20141126_104159results_file.spkl', '../fishVR_data/20141127_103904results_file.spkl']
#spkl_files = ['../fishVR_data/20141112_120446results_file.spkl', '../fishVR_data/20141112_193447results_file.spkl']

w = []
for myfile in spkl_files:
    f = open(myfile)
    fl = spkl.s_load(f)
    for element in fl:
        w.append(element)

CS_elements = []
off_elements = []

for element in w:
    if (element[0]%(6000*6) < 5.*100):
        CS_elements.append(element)
    elif (element[0]%(6000*6) > 5.*100):
        off_elements.append(element)

print len(CS_elements), len(off_elements)

alphas_CS = [CS_elem[1] for CS_elem in CS_elements]
alphas_off = [off_elem[1] for off_elem in off_elements]

for i in range(len(alphas_CS)):
    alphas_CS[i]=np.abs(alphas_CS[i])

for i in range(len(alphas_off)):
    alphas_off[i]=np.abs(alphas_off[i])

for i in range (len(alphas_CS)):
    if alphas_CS[i] > 90.:
        alphas_CS[i] = 90.
    if alphas_CS[i] < 5. or (math.isnan(alphas_CS[i])==True):
        alphas_CS[i] = 0.

for i in range (len(alphas_off)):
    if alphas_off[i]>90.:
        alphas_off[i]=90.
    if alphas_off[i]<5. or (math.isnan(alphas_off[i])==True):
        alphas_off[i] = 0.

r_CS = np.mean(alphas_CS)
print 'r_CS:', r_CS
r_off = np.mean(alphas_off)
print 'r_off:', r_off

SEM_CS = np.std(alphas_CS)/np.sqrt(len(alphas_CS))
print 'SEM_CS:', SEM_CS
SEM_off = np.std(alphas_off)/np.sqrt(len(alphas_off))
print 'SEM_off:', SEM_off

#paired_sample = stats.ttest_rel(alphas_CS, alphas_off)
#print "The t-statistic is %.3f and the p-value is %.3f." % paired_sample

N = 2
ind = np.arange(N)
width = 0.35
fig = plt.figure()
fig.suptitle('area CS and off (all files)')
axx = fig.add_subplot(111)
plt.ylabel('area')
y = [r_CS, r_off]
plt.xticks(np.arange(2), ("CS", "off"))
p1=axx.bar(ind[0], y[0], width, color='0.75', yerr=SEM_CS)
p2=axx.bar(ind[1], y[1], width,color='0.23',yerr=SEM_off)
plt.show()
