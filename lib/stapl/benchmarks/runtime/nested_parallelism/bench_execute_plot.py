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
    threads    = 0
    iterations = 0
    suffix     = []
    for l in f:
        data = shlex.split(l)
        if not data:
            continue
        if data[0]=='benchmark' or data[0]=='processes' or data[0]=='locations':
            pass
        elif data[0]=='iterations':
            if iterations==0:
                iterations = int(data[1])
            elif int(data[1])!=iterations:
                msg = ('File "' + filename +
                       '": number of iterations is not consistent')
                raise Exception(msg)
        elif data[0]=='threads':
            tmp = int(data[1])
            if threads and threads!=tmp:
                msg = 'File "' + filename + '": "threads" already defined'
                raise Exception(msg)
            threads = tmp
        elif data[0]=='suffix':
            suffix = data[1]
        elif data[0][0]=='#':
            pass
        else:
            # parse raw data
            name, total, raw_data = data[0], data[1], data[2:len(data)]
            if suffix:
                name += suffix
            # avg, min, max, stddev, conf
            if m.has_key(name):
                msg = ('File "' + filename + '": data for "' + name +
                       '" already exists')
                raise Exception(msg)
            m[name] = array( map(float64, raw_data) ) * 1000.0
    f.close()
    return (threads, m)

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

################################################################################
# main program

# read command line arguments
parser = argparse.ArgumentParser(description='Plot bench_execute.')
parser.add_argument('--title', dest='title', default='',
                    help='title of the graph')
parser.add_argument('--baseline', dest='baseline', required=True,
                    help='the baseline for creating the speedup graph')
args      = parser.parse_args()
title     = args.title
baseline  = args.baseline

# read files and create map of {kernel name, data}
fig_data  = collections.defaultdict(lambda: collections.defaultdict(tuple))
out_files = glob.glob1(os.getcwd(), "*.out")
for filename in out_files:
    data = parse_file(filename)
    threads = data[0]
    for key,value in data[1].iteritems():
        fig_data[key][threads] = value

# generate raw data graph
matplotlib.rcParams['axes.color_cycle'] = ['b', 'r', 'm', 'c']
fig = matplotlib.pyplot.figure()
ax  = fig.add_subplot(111)
ax.set_title(title)
ax.set_xlabel('Number of threads')
ax.set_ylabel('Time (ms)')
ax.grid(True)
# iterate through each kernel and read data for different number of threads
xtick_lbl = ()
for kernel,data in fig_data.iteritems():
    labels   = ()
    avg_time = ()
    min_time = ()
    max_time = ()
    conf_int = ()
    ordered_keys = sorted(data.keys(), key=cmp_to_key(par_cmp))
    for k in ordered_keys:
        v = data[k]
        labels  += (k,)
        avg_time = concatenate( (avg_time, array([v[0]])) )
        min_time = concatenate( (min_time, array([v[1]])) )
        max_time = concatenate( (max_time, array([v[2]])) )
        conf_int = concatenate( (conf_int, array([v[4]])) )
    #nmin = (avg_time - min_time)
    #nmax = (max_time - avg_time)
    nmin = conf_int
    nmax = conf_int
    #line = ax.plot(labels, avg_time, 'o-', label=kernel,
    #               color=getCycledColor())
    line = ax.plot(labels, avg_time, 'o-', label=kernel)
    errorbar(labels, avg_time, yerr=[nmin, nmax], fmt='k')
    xtick_lbl = labels
xtick_idx = map(int, labels)
[xmin, xmax, ymin, ymax] = ax.axis()
ax.axis( (xtick_idx[0] - 0.5, xtick_idx[len(xtick_idx)-1] + 0.5, ymin, ymax) )
ax.set_xticks(xtick_idx)
ax.set_xticklabels(xtick_lbl, rotation=45)
ax.legend(loc=0)
matplotlib.pyplot.show()

# generate speedup graph
matplotlib.rcParams['axes.color_cycle'] = ['b', 'r', 'm', 'c']
fig = matplotlib.pyplot.figure()
ax  = fig.add_subplot(111)
ax.set_title(title + ' - speedup over ' + baseline)
ax.set_xlabel('Number of threads')
ax.set_ylabel('speedup')
ax.grid(True)

baseline_avg_time = ()
data = fig_data[baseline]
ordered_keys = sorted(data.keys(), key=cmp_to_key(par_cmp))
for k in ordered_keys:
    v = data[k]
    baseline_avg_time = concatenate( (baseline_avg_time, array([v[0]])) )

# iterate through each kernel and read data for different number of threads
for kernel,data in fig_data.iteritems():
    if kernel==baseline:
        continue
    labels   = ()
    avg_time = ()
    ordered_keys = sorted(data.keys(), key=cmp_to_key(par_cmp))
    for k in ordered_keys:
        v = data[k]
        labels  += (k,)
        avg_time = concatenate( (avg_time, array([v[0]])) )
    #navg = ((avg_time - baseline_avg_time)/baseline_avg_time) * 100
    navg = (baseline_avg_time/avg_time)
    navg = (avg_time - baseline_avg_time)
    line = ax.plot(labels, navg, 'o-', label=kernel)
[xmin, xmax, ymin, ymax] = ax.axis()
ax.axis( (xtick_idx[0] - 0.5, xtick_idx[len(xtick_idx)-1] + 0.5, ymin, ymax) )
ax.set_xticks(xtick_idx)
ax.set_xticklabels(xtick_lbl, rotation=45)
ax.legend(loc=0)
matplotlib.pyplot.show()
