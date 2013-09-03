#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.Uniform.35 -V
cd ./Hook/
../pmpl -f Hook.Uniform.35.xml >& Hook.Uniform.35.log
