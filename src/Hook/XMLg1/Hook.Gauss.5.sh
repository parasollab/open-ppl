#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Gauss.5 -V
cd ./Hook/
../pmpl -f Hook.Gauss.5.xml >& Hook.Gauss.5.log
