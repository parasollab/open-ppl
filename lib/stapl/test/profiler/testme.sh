#!/bin/sh

# Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
# component of the Texas A&M University System.
#
# All rights reserved.
#
# The information and source code contained herein is the exclusive
# property of TEES and may not be disclosed, examined or reproduced
# in whole or in part without explicit written authorization from TEES.

run_command=$1
echo $run_command

eval $run_command ./test_profiler
if test $? != 0
then
    echo "ERROR:: while testing test_profiler"
fi

# verify output
#ls -al test*   # will return testme.sh at the very least (should never return 1 (fail))
#TS=`date +%Y%m%d`  # just the year/date/month part
#for ext in txt sql
#do
#  # `hostname` isn't reliable on systems that run on back-end nodes
#  # we might have to toss $LOGNAME as well
#  file=`ls test-*-$LOGNAME-$TS*.$ext | tail -1`  # should only be one but just in case
#  if [ "$file" != " " -a -f $file ]
#  then
#    cat BP-$ext-verification.dat | while read num line
#    do
#      let numfound=`grep -c "$line" $file`
#      if [ $numfound -lt $num ]
#      then
#        echo "ERROR:: while checking BaseProfiler output - $numfound < $num - $line - $file"
#      fi
#    done
#    rm $file
#  fi
#done
##EOF
