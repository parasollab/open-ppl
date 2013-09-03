#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.u1a4.40 -V
cd ./Hook/
../pmpl -f Hook.u1a4.40.xml >& Hook.u1a4.40.log
