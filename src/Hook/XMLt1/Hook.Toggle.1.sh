#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Toggle.1 -V
cd ./Hook/
../pmpl -f Hook.Toggle.1.xml >& Hook.Toggle.1.log
