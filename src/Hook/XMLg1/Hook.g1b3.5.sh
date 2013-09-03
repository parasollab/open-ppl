#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.g1b3.5 -V
cd ./Hook/
../pmpl -f Hook.g1b3.5.xml >& Hook.g1b3.5.log
