#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.Uniform.14 -V
cd ./Hook/
../pmpl -f Hook.Uniform.14.xml >& Hook.Uniform.14.log
