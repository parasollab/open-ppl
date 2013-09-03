#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Uniform.43 -V
cd ./Hook/
../pmpl -f Hook.Uniform.43.xml >& Hook.Uniform.43.log
