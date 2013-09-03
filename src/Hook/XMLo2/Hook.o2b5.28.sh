#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.o2b5.28 -V
cd ./Hook/
../pmpl -f Hook.o2b5.28.xml >& Hook.o2b5.28.log
