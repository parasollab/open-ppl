#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.g1a1.29 -V
cd ./Hook/
../pmpl -f Hook.g1a1.29.xml >& Hook.g1a1.29.log
