#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.Uniform.42 -V
cd ./Hook/
../pmpl -f Hook.Uniform.42.xml >& Hook.Uniform.42.log
