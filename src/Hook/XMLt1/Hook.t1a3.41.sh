#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.t1a3.41 -V
cd ./Hook/
../pmpl -f Hook.t1a3.41.xml >& Hook.t1a3.41.log
