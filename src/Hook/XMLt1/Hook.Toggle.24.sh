#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Toggle.24 -V
cd ./Hook/
../pmpl -f Hook.Toggle.24.xml >& Hook.Toggle.24.log
