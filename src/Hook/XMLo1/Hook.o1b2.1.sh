#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.o1b2.1 -V
cd ./Hook/
../pmpl -f Hook.o1b2.1.xml >& Hook.o1b2.1.log
