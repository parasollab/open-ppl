#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Toggle.12 -V
cd ./Hook/
../pmpl -f Hook.Toggle.12.xml >& Hook.Toggle.12.log
