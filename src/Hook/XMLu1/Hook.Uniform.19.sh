#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Uniform.19 -V
cd ./Hook/
../pmpl -f Hook.Uniform.19.xml >& Hook.Uniform.19.log
