#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.t1b4.15 -V
cd ./Hook/
../pmpl -f Hook.t1b4.15.xml >& Hook.t1b4.15.log
