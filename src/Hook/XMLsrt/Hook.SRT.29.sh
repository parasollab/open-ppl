#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.SRT.29 -V
cd ./Hook/
../pmpl -f Hook.SRT.29.xml >& Hook.SRT.29.log
