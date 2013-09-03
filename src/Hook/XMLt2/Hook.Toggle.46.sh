#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.Toggle.46 -V
cd ./Hook/
../pmpl -f Hook.Toggle.46.xml >& Hook.Toggle.46.log
