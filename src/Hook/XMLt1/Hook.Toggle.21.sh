#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Toggle.21 -V
cd ./Hook/
../pmpl -f Hook.Toggle.21.xml >& Hook.Toggle.21.log
