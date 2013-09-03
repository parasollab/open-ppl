#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.Toggle.39 -V
cd ./Hook/
../pmpl -f Hook.Toggle.39.xml >& Hook.Toggle.39.log
