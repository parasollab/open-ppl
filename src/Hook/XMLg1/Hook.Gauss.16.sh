#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Gauss.16 -V
cd ./Hook/
../pmpl -f Hook.Gauss.16.xml >& Hook.Gauss.16.log
