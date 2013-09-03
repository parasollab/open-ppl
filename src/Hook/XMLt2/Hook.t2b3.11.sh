#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.t2b3.11 -V
cd ./Hook/
../pmpl -f Hook.t2b3.11.xml >& Hook.t2b3.11.log
