#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.u1b2.43 -V
cd ./Hook/
../pmpl -f Hook.u1b2.43.xml >& Hook.u1b2.43.log
