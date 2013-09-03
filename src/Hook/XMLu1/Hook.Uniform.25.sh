#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Uniform.25 -V
cd ./Hook/
../pmpl -f Hook.Uniform.25.xml >& Hook.Uniform.25.log
