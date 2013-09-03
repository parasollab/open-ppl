#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.t1b2.26 -V
cd ./Hook/
../pmpl -f Hook.t1b2.26.xml >& Hook.t1b2.26.log
