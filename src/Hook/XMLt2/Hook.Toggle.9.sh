#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.Toggle.9 -V
cd ./Hook/
../pmpl -f Hook.Toggle.9.xml >& Hook.Toggle.9.log
