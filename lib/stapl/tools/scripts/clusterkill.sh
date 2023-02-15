#!/bin/sh
if test -z $1
then
 echo usage: clusterkill.sh program_name
else
echo kill local
killall -9 $1
echo kill on compute-0-0
ssh compute-0-0 killall -9 $1
echo kill on compute-0-1
ssh compute-0-1 killall -9 $1
echo kill on compute-0-2
ssh compute-0-2 killall -9 $1
fi
