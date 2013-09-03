#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.Toggle.34 -V
cd ./Hook/
../pmpl -f Hook.Toggle.34.xml >& Hook.Toggle.34.log
