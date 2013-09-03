#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.Toggle.44 -V
cd ./Hook/
../pmpl -f Hook.Toggle.44.xml >& Hook.Toggle.44.log
