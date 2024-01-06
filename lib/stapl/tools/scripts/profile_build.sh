#!/bin/bash
#
# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
# 
# All rights reserved.
# 
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.
#
# ==========================================================================
#
# This script parses the output of a STAPL validation build, scraping source
# filenames as well as their corresponding compilation time and memory
# consumption as reported by /usr/bin/time.  The entries are sorted by memory
# usage in decreasing order.
#
# To generate the appropriate output, set USER_CC appropriately prior to the
# build.  For example, for mpi wrapped builds,
#
# export CC_USER='/usr/bin/time mpic++'
#
# **NOTE** This only works for sequential (i.e., -j 1) invocations of make so
# that the output file is in a parsable form.
#

grep "bin/time\|maxresident" $1|\
sed -e 's/.* \([^ \/]*\.cc\).*/\1/'|\
sed 's/.* -o \([^ ]*\)/\1/'|\
grep -v const_iterator_write|\
awk '{printf"%-40s\n",$0}'|\
sed -e 's/.* \(.*\)elapsed .* \(.*\)maxresident.*/\tTime = \1\t\tMemory = \2/'|\
awk 'ORS=NR%2?" ":"\n"'|\
sort -k 7 -n -r 
