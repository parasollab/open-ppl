#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Uniform.1 -V
cd ./Hook/
../pmpl -f Hook.Uniform.1.xml >& Hook.Uniform.1.log
