#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Gauss.40 -V
cd ./Hook/
../pmpl -f Hook.Gauss.40.xml >& Hook.Gauss.40.log
