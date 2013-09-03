#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.g1a2.23 -V
cd ./Hook/
../pmpl -f Hook.g1a2.23.xml >& Hook.g1a2.23.log
