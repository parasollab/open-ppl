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
    kernel    = []
    processes = []
    threads   = []
    elements  = []
    memory    = []
    for l in f:
        data = shlex.split(l)
        if len(data)==0:
            pass
        elif data[0]=='kernel':
            if elements:
                msg = 'File "' + filename + '": "kernel" already defined'
                raise Exception(msg)
            kernel = data[1]
        elif data[0]=='elements':
            if elements:
                msg = 'File "' + filename + '": "elements" already defined'
                raise Exception(msg)
            elements = int(data[1])
        elif data[0]=='processes':
            if processes:
                msg = 'File "' + filename + '": "processes" already defined'
                raise Exception(msg)
            processes = int(data[1])
        elif data[0]=='threads':
            if threads:
                msg = 'File "' + filename + '": "threads" already defined'
                raise Exception(msg)
            threads = int(data[1])
        elif data[0]=='TOTAL:': # for hydra
            if memory:
                msg = 'File "' + filename + '": "memory" already defined'
                raise Exception(msg)
            memory = float(data[2])
        else:
            pass
    f.close()
    if threads*processes!=elements:
        raise Exception("Mixed-mode is broken")
    return (kernel, (processes, threads), memory)

def create_label(t) :
    return 'x'.join([`num` for num in t])

def num_of_PEs(t) :
    return reduce(lambda x, y: x*y, t)

def par_cmp(x,y) :
    if x[0]!=y[0]:
        return y[0]-x[0]
    if len(x[1])!=len(y[1]):
        return len(x[1])-len(y[1])
    for i in range(len(x[1])):
        c = y[1][i]-x[1][i]
        if c!=0:
          return c;
    return 0

# main program

# read command line arguments
parser = argparse.ArgumentParser(description='Plot memory_footprint.')
parser.add_argument('--title',
                    dest='title',
                    default='',
                    help='additional title for the graph')
args       = parser.parse_args()
more_title = args.title

# read files and create map of <kernel name>, <data>
fig_data  = collections.defaultdict(lambda: collections.defaultdict(tuple))
out_files = glob.glob1(os.getcwd(), "*.out")
for filename in out_files:
    data = parse_file(filename)
    name = data[0]
    fig_data[name][data[1]] = data[2]

print fig_data

# iterate through each kernel and read data for different number of threads
fig = matplotlib.pyplot.figure()
ax  = fig.add_subplot(111)
ax.set_title("Memory footprint" + more_title)
ax.set_xlabel('processes x threads')
ax.set_ylabel('Memory (MB)')

for title,data in fig_data.iteritems():
    labels = collections.defaultdict(tuple)
    memory = collections.defaultdict(tuple)
    # iterate through each kernel and read data for different number of threads
    ordered_keys = sorted(data.keys(), key=cmp_to_key(par_cmp))
    for k in ordered_keys:
        v = data[k]
        print v
        labels[k[0]] += (create_label(v[1]),)
        memory[k[0]]  = concatenate( (memory[k[0]], array([v[0]])) )
    # plot data
    bar_width  = 0.1
    bar_dist   = bar_width/4
    group_dist = bar_width
    #xtick_idx = ()
    #xtick_lbl = ()
    last_idx     = 0.0
    ordered_keys = sorted(avg_time.keys())
    for pes in ordered_keys:
        d = avg_time[pes]
        # figure out indices
        N   = len(d)
        idx = numpy.arange(N)
        # plot data
        plot_idx = idx * (bar_width + bar_dist) + last_idx
        last_idx = plot_idx[N-1] + (group_dist + bar_width)
        ax.bar(plot_idx, d, bar_width, color='b')
        # save position for labels
        #xtick_idx += tuple(plot_idx + bar_width/2)
        #xtick_lbl += labels[pes]
# create labels
#ax.set_xticks(xtick_idx)
#ax.set_xticklabels(xtick_lbl, rotation=45)
#[xmin, xmax, ymin, ymax] = ax.axis()
#ax.axis( (xmin-group_dist, last_idx, ymin, ymax) )
matplotlib.pyplot.show()
