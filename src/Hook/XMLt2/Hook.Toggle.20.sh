#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.Toggle.20 -V
cd ./Hook/
../pmpl -f Hook.Toggle.20.xml >& Hook.Toggle.20.log
