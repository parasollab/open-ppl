#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Gauss.27 -V
cd ./Hook/
../pmpl -f Hook.Gauss.27.xml >& Hook.Gauss.27.log
