#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.g1a2.25 -V
cd ./Hook/
../pmpl -f Hook.g1a2.25.xml >& Hook.g1a2.25.log
