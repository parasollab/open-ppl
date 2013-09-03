#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.g1a3.35 -V
cd ./Hook/
../pmpl -f Hook.g1a3.35.xml >& Hook.g1a3.35.log
