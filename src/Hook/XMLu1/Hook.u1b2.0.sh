#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.u1b2.0 -V
cd ./Hook/
../pmpl -f Hook.u1b2.0.xml >& Hook.u1b2.0.log
