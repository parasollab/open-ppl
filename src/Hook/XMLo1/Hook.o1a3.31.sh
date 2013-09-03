#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.o1a3.31 -V
cd ./Hook/
../pmpl -f Hook.o1a3.31.xml >& Hook.o1a3.31.log
