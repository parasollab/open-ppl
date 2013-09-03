#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.t1b4.2 -V
cd ./Hook/
../pmpl -f Hook.t1b4.2.xml >& Hook.t1b4.2.log
