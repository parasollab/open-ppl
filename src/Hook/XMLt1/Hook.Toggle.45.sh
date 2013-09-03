#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Toggle.45 -V
cd ./Hook/
../pmpl -f Hook.Toggle.45.xml >& Hook.Toggle.45.log
