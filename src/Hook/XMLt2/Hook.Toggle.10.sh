#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.Toggle.10 -V
cd ./Hook/
../pmpl -f Hook.Toggle.10.xml >& Hook.Toggle.10.log
