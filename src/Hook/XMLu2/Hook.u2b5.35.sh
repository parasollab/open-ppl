#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.u2b5.35 -V
cd ./Hook/
../pmpl -f Hook.u2b5.35.xml >& Hook.u2b5.35.log
