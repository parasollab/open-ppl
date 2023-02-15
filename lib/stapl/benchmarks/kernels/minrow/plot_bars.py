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
    parallelism    = []
    container_size = []
    iterations     = 0
    for l in f:
        data = shlex.split(l)
        if data[0]=='parallelism':
            tmp = map(lambda x: int(x), data[1:len(data)])
            if parallelism and parallelism!=tmp:
                msg = 'File "' + filename + '": "parallelism" already defined'
                raise Exception(msg)
            parallelism = tmp
        elif data[0]=='container_size':
            if container_size:
                msg = 'File "' + filename + '": "container_size" already defined' 
                raise Exception(msg)
            container_size = map(lambda x: int(x), data[1:len(data)])
        else:
            # parse raw data
            name, iters, raw_data = data[0], data[1], data[2:len(data)]
            if iterations==0:
                iterations = int(iters)
            elif int(iters)!=iterations:
                msg = 'File "' + filename + '": number of iterations is not consistent across kernels'
                raise Exception(msg)
            # avg, min, max, conf
            if m.has_key(name):
                msg = 'File "' + filename + '": data for "' + name + '" already exists'
                raise Exception(msg)
            m[name] = array( map(float64, raw_data) )
    f.close()
    return (container_size, tuple(parallelism), m)

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
parser = argparse.ArgumentParser(description='Plot min_row.')
parser.add_argument('--title', dest='title', default='', help='additional title for the graph')
args       = parser.parse_args()
more_title = args.title

# read files and create map of <kernel name>, <data>
fig_data       = collections.defaultdict(lambda: collections.defaultdict(tuple))
out_files      = glob.glob1(os.getcwd(), "*.out")
container_size = []
for filename in out_files:
    data           = parse_file(filename)
    container_size = data[0]
    pes            = num_of_PEs(data[1])
    for key,value in data[2].iteritems():
        fig_data[key][(pes, data[1])] = value

# iterate through each kernel and read data for different number of threads
for title,data in fig_data.iteritems():
    labels   = collections.defaultdict(tuple)
    avg_time = collections.defaultdict(tuple)
    min_time = collections.defaultdict(tuple)
    max_time = collections.defaultdict(tuple)
    conf_int = collections.defaultdict(tuple)
    # iterate through each kernel and read data for different number of threads
    ordered_keys = sorted(data.keys(), key=cmp_to_key(par_cmp))
    for k in ordered_keys:
        v = data[k]
        labels[k[0]]  += (create_label(k[1]),)
        avg_time[k[0]] = concatenate( (avg_time[k[0]], array([v[0]])) )
        min_time[k[0]] = concatenate( (min_time[k[0]], array([v[1]])) )
        max_time[k[0]] = concatenate( (max_time[k[0]], array([v[2]])) )
        conf_int[k[0]] = concatenate( (conf_int[k[0]], array([v[3]])) )
    # plot data
    bar_width  = 0.1
    bar_dist   = bar_width/4
    group_dist = bar_width
    fig = matplotlib.pyplot.figure()
    ax  = fig.add_subplot(111)
    xtick_idx = ()
    xtick_lbl = ()
    last_idx     = 0.0
    ordered_keys = sorted(avg_time.keys())
    for pes in ordered_keys:
        d = avg_time[pes]
        #e = avg_time[pes] - min_time[pes]
        #f = max_time[pes] - avg_time[pes]
        e = conf_int[pes]
        f = conf_int[pes]
        # figure out indices
        N   = len(d)
        idx = numpy.arange(N)
        # plot data
        plot_idx = idx * (bar_width + bar_dist) + last_idx
        last_idx = plot_idx[N-1] + (group_dist + bar_width)
        ax.bar(plot_idx, d, bar_width, color='b')
        # plot errors
        ax.errorbar(plot_idx + bar_width/2, d, fmt='k.', yerr=[e,f])
        # save position for labels
        xtick_idx += tuple(plot_idx + bar_width/2)
        xtick_lbl += labels[pes]
    # create labels
    ax.set_title(title + ' - container size ' + create_label(container_size) + more_title)
    ax.set_xlabel('processes x threads')
    ax.set_ylabel('Time (s)')
    ax.set_xticks(xtick_idx)
    ax.set_xticklabels(xtick_lbl, rotation=45)
    [xmin, xmax, ymin, ymax] = ax.axis()
    ax.axis( (xmin-group_dist, last_idx, ymin, ymax) )
    matplotlib.pyplot.show()
