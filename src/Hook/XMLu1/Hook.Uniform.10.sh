#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Uniform.10 -V
cd ./Hook/
../pmpl -f Hook.Uniform.10.xml >& Hook.Uniform.10.log
