#!/usr/bin/env python
#
# https://www.dexterindustries.com/BrickPi/
# https://github.com/DexterInd/BrickPi3
#
# Copyright (c) 2016 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
# Modified by R. Aragues / A. Murillo / L. Montano. 2018.
#
# This code is an example for reading a CSV file. Some of the entries in this file are used as x,y coordinates
# and they are plotted in a Figure.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import csv


import matplotlib
import matplotlib.pyplot as plt

matplotlib.use("TkAgg")

current_fig = plt.figure()
current_ax = current_fig.add_subplot(111)
current_fig.set_visible(True)
plt.rc('grid', linestyle="--", color='gray')
plt.grid(True)
plt.tight_layout()


step_by_step=0

filename = 'mi_prueba.csv'
file = open(filename, 'r') 
csvreader = csv.reader(file)
header = []
header = next(csvreader)
rows = []
for row in csvreader:
    #rows.append(row) # If we want them together
    print(row)
    print("every element")
    t_val = float(row[0])
    x_val  = float(row[1])
    y_val = float(row[2])
    th_val = float(row[2])
    current_ax.plot(x_val, y_val, 'r.')
    #print(t_val, x_val, y_val, th_val)
    plt.axis('equal')
    current_fig.show()
    if (step_by_step==1):
       print("Press ENTER in the plot window to continue ... ")
       current_fig.waitforbuttonpress()
    elif (step_by_step==2):
        plt.pause(0.01)
        

file.close()
print("Press ENTER in the plot window to continue ... ")
current_fig.waitforbuttonpress()
plt.close()



