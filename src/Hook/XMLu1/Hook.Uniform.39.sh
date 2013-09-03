#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=20:00:00 -N Hook.Uniform.39 -V
cd ./Hook/
../pmpl -f Hook.Uniform.39.xml >& Hook.Uniform.39.log
