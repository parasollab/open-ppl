#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.t2b5.23 -V
cd ./Hook/
../pmpl -f Hook.t2b5.23.xml >& Hook.t2b5.23.log
