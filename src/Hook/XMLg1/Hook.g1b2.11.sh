#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.g1b2.11 -V
cd ./Hook/
../pmpl -f Hook.g1b2.11.xml >& Hook.g1b2.11.log
