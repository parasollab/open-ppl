#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Gauss.49 -V
cd ./Hook/
../pmpl -f Hook.Gauss.49.xml >& Hook.Gauss.49.log
