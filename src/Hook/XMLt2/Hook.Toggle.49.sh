#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.Toggle.49 -V
cd ./Hook/
../pmpl -f Hook.Toggle.49.xml >& Hook.Toggle.49.log
