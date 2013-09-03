#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.u2b2.8 -V
cd ./Hook/
../pmpl -f Hook.u2b2.8.xml >& Hook.u2b2.8.log
