#!/usr/bin/env python

import numpy
import matplotlib.pyplot
import glob
import os,sys
import shlex
import collections
import argparse

from pylab import *
from operator import itemgetter

def cmp_to_key(mycmp):
    'Convert a cmp= function into a key= function'
    class K(object):
        def __init__(self, obj, *args):
            self.obj = obj
        def __lt__(self, other):
            return mycmp(self.obj, other.obj) < 0
        def __gt__(self, other):
            return mycmp(self.obj, other.obj) > 0
        def __eq__(self, other):
            return mycmp(self.obj, other.obj) == 0
        def __le__(self, other):
            return mycmp(self.obj, other.obj) <= 0
        def __ge__(self, other):
            return mycmp(self.obj, other.obj) >= 0
        def __ne__(self, other):
            return mycmp(self.obj, other.obj) != 0
    return K

def parse_file(filename):
    m = {}
    f = open(filename)
    # should remain constant within the output file
    iterations = 0
    for l in f:
        data = shlex.split(l)
        if not data:
            continue
        if data[0]=='iterations':
            if iterations==0:
                iterations = int(data[1])
            elif int(data[1])!=iterations:
                msg = ('File "' + filename +
                       '": number of iterations is not consistent')
                raise Exception(msg)
        elif data[0]=='benchmark' or data[0]=='processes' or \
             data[0]=='threads' or data[0]=='locations' or \
             data[0][0]=='#':
            pass
        else:
            # parse raw data
            name, procs, raw_data = data[0], data[1], data[2:len(data)]
            # avg, min, max, conf
            if not m.has_key(name):
                m[name] = {}
            m[name][procs] = array( map(float64, raw_data) ) * 1000.0
    f.close()
    return m

def create_label(t) :
    return 'x'.join([`num` for num in t])

def par_cmp(x,y) :
    return int(x)-int(y)

colors = ['b', 'r', 'm', 'c']
i = -1
def getCycledColor():
    global i, colors
    if i < len(colors)-1:
        i = i + 1
        return colors[i]
    else:
        i = -1

# main program

# read command line arguments
parser = argparse.ArgumentParser(description='Plot bench_gang.')
parser.add_argument('--title', dest='title', default='',
                    help='additional title for the graph')
parser.add_argument('--file', dest='filename', required=True,
                    help='file to parse')
args     = parser.parse_args()
title    = args.title
filename = args.filename

cwd      = os.getcwd()
filename = os.path.join(cwd, filename)
fig_data = parse_file(filename)
print 'Parsing: ' + filename

fig = matplotlib.pyplot.figure()
ax  = fig.add_subplot(111)
ax.set_title(title)
ax.set_xlabel('Gang Size (locations)')
ax.set_ylabel('Time (ms)')
ax.grid(True)
ax.set_xscale('log')
xtick_lbl = ()
print fig_data
# iterate through each kernel and read data for different number of threads
for kernel,data in fig_data.iteritems():
    labels   = ()
    avg_time = ()
    min_time = ()
    max_time = ()
    conf_int = ()
    # iterate through each kernel and read data for different number of threads
    ordered_keys = sorted(data.keys(), key=cmp_to_key(par_cmp))
    for k in ordered_keys:
        v = data[k]
        labels  += (k,)
        avg_time = concatenate( (avg_time, array([v[1]])) )
        min_time = concatenate( (min_time, array([v[2]])) )
        max_time = concatenate( (max_time, array([v[3]])) )
        conf_int = concatenate( (conf_int, array([v[5]])) )

    #nmin = avg_time - min_time
    #nmax = max_time - avg_time
    nmin = conf_int
    nmax = conf_int
    line = ax.plot(labels, avg_time, 'o-', label=kernel, color=getCycledColor())
    #line = ax.plot(labels, avg_time, 'o-', label=kernel)
    errorbar(labels, avg_time, yerr=[nmin,nmax], fmt='k')
    xtick_lbl = labels

xtick_idx = map(int, labels)
[xmin, xmax, ymin, ymax] = ax.axis()
ax.axis( (xtick_idx[0]*0.5, xtick_idx[len(xtick_idx)-1] * 1.5, ymin, ymax) )
ax.set_xticks(xtick_idx)
ax.set_xticklabels(xtick_lbl, rotation=45)
ax.legend(loc=0)
matplotlib.pyplot.show()
