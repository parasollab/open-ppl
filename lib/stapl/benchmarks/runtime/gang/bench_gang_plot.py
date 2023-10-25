#!/usr/bin/env python

import numpy
import matplotlib.pyplot
import glob
import os,sys
import shlex
import collections
import argparse
import pylab

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
    total_locs = 0
    iterations  = 0
    for l in f:
        data = shlex.split(l)
        if not data:
            continue
        if data[0]=='locations':
            if total_locs==0:
              total_locs = int(data[1])
            elif int(data[1])!=total_locs:
              msg = 'File "' + filename + '": "locations" is not consistent'
              raise Exception(msg)
        elif data[0]=='iterations':
            if iterations==0:
                iterations = int(data[1])
            elif int(data[1])!=iterations:
                msg = ('File "' + filename +
                       '": number of iterations is not consistent')
                raise Exception(msg)
        elif data[0]=='benchmark' or data[0]=='processes' or \
             data[0]=='threads' or \
             data[0]=='Application' or \
             data[0][0]=='#':
            pass
        else:
            # parse raw data
            name, procs, p, n, raw_data = data[0], data[1], data[2], data[3],
                                          data[4:len(data)]
            if n!="1":
              name = name + " " + n + " threads/process";
            else:
              name = name + " 1 thread/process";
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

colors = ['b-o', 'r-x', 'm-*', 'c-+', 'g->']
i = -1
def getCycledColor():
    global i, colors
    if i < len(colors)-1:
        i = i + 1
        return colors[i]
    else:
        i = 0
        return colors[0]

# main program

# configuration
line_width       = 2
marker_size      = 10
font_size        = 11
title_font_size  = 11
axis_font_size   = 14
legend_font_size = 10

# read command line arguments
parser = argparse.ArgumentParser(description='Plot bench_gang.')
parser.add_argument('--title',
                    dest='title',
                    default='',
                    help='additional title for the graph')
parser.add_argument('--file',
                    dest='filename',
                    required=True,
                    help='file to parse')
parser.add_argument('--output',
                    dest='output',
                    default='',
                    help='output file')
args   = parser.parse_args()
title  = args.title
fname  = args.filename
out_fn = args.output

cwd      = os.getcwd()
fname    = os.path.join(cwd, fname)
fig_data = parse_file(fname)
print 'Parsing: ' + fname

matplotlib.rcParams['font.size'] = font_size
fig = matplotlib.pyplot.figure()
ax  = fig.add_subplot(111)
ax.set_title(title, fontsize = title_font_size)
ax.set_xlabel('Gang Size (locations)', fontsize = axis_font_size)
ax.set_ylabel('Time (ms)', fontsize = axis_font_size)
ax.grid(True)
ax.set_xscale('log')
xtick_lbl = ()
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
    line = ax.plot(labels, avg_time, getCycledColor(), label=kernel,
                   linewidth=line_width, markersize=marker_size)
    #line = ax.plot(labels, avg_time, 'o-', label=kernel)
    errorbar(labels, avg_time, yerr=[nmin,nmax], fmt='k')
    xtick_lbl = labels

xtick_idx = map(int, labels)
[xmin, xmax, ymin, ymax] = ax.axis()
ax.axis( (xtick_idx[0]*0.5, xtick_idx[len(xtick_idx)-1] * 1.5, ymin, ymax) )
ax.set_xticks(xtick_idx)
ax.set_xticklabels(xtick_lbl, rotation=45)
ax.legend(loc=0, fontsize = legend_font_size)
pylab.ylim(0)

if out_fn:
  matplotlib.pyplot.savefig(out_fn, bbox_inches='tight')
else:
  matplotlib.pyplot.show()
