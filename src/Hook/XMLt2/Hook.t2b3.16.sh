#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.t2b3.16 -V
cd ./Hook/
../pmpl -f Hook.t2b3.16.xml >& Hook.t2b3.16.log
