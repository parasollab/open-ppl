#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.u1b4.46 -V
cd ./Hook/
../pmpl -f Hook.u1b4.46.xml >& Hook.u1b4.46.log
