#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N Hook.Uniform.18 -V
cd ./Hook/
../pmpl -f Hook.Uniform.18.xml >& Hook.Uniform.18.log
