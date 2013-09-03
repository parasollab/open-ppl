#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Toggle.35 -V
cd ./Hook/
../pmpl -f Hook.Toggle.35.xml >& Hook.Toggle.35.log
