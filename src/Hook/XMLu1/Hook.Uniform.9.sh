#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Uniform.9 -V
cd ./Hook/
../pmpl -f Hook.Uniform.9.xml >& Hook.Uniform.9.log
