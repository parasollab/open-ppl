#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.t2b2.27 -V
cd ./Hook/
../pmpl -f Hook.t2b2.27.xml >& Hook.t2b2.27.log
