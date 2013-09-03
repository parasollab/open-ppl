#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.o1a4.0 -V
cd ./Hook/
../pmpl -f Hook.o1a4.0.xml >& Hook.o1a4.0.log
