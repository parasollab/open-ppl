#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.u1b3.10 -V
cd ./Hook/
../pmpl -f Hook.u1b3.10.xml >& Hook.u1b3.10.log
