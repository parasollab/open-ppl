#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.u2b5.1 -V
cd ./Hook/
../pmpl -f Hook.u2b5.1.xml >& Hook.u2b5.1.log
