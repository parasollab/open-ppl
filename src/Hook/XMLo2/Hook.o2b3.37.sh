#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.o2b3.37 -V
cd ./Hook/
../pmpl -f Hook.o2b3.37.xml >& Hook.o2b3.37.log
