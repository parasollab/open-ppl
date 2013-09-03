#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.Toggle.17 -V
cd ./Hook/
../pmpl -f Hook.Toggle.17.xml >& Hook.Toggle.17.log
