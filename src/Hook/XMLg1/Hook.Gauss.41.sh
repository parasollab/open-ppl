#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Gauss.41 -V
cd ./Hook/
../pmpl -f Hook.Gauss.41.xml >& Hook.Gauss.41.log
