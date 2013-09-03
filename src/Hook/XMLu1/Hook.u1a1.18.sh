#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.u1a1.18 -V
cd ./Hook/
../pmpl -f Hook.u1a1.18.xml >& Hook.u1a1.18.log
