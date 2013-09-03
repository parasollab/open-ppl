#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.Uniform.27 -V
cd ./Hook/
../pmpl -f Hook.Uniform.27.xml >& Hook.Uniform.27.log
