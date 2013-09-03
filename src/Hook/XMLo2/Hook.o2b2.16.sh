#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.o2b2.16 -V
cd ./Hook/
../pmpl -f Hook.o2b2.16.xml >& Hook.o2b2.16.log
