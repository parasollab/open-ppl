#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.o1b1.47 -V
cd ./Hook/
../pmpl -f Hook.o1b1.47.xml >& Hook.o1b1.47.log
