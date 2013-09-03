#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.t1b2.5 -V
cd ./Hook/
../pmpl -f Hook.t1b2.5.xml >& Hook.t1b2.5.log
