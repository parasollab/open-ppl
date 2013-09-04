#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.SRT.15 -V
cd ./Hook/
../pmpl -f Hook.SRT.15.xml >& Hook.SRT.15.log
