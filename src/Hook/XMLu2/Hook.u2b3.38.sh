#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.u2b3.38 -V
cd ./Hook/
../pmpl -f Hook.u2b3.38.xml >& Hook.u2b3.38.log
