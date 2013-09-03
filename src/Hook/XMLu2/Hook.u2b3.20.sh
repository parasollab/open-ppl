#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.u2b3.20 -V
cd ./Hook/
../pmpl -f Hook.u2b3.20.xml >& Hook.u2b3.20.log
