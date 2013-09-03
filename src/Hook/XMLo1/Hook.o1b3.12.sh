#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.o1b3.12 -V
cd ./Hook/
../pmpl -f Hook.o1b3.12.xml >& Hook.o1b3.12.log
