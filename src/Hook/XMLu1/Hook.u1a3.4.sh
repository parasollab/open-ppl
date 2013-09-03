#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.u1a3.4 -V
cd ./Hook/
../pmpl -f Hook.u1a3.4.xml >& Hook.u1a3.4.log
