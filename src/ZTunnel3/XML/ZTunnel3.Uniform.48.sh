#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel3.Uniform.48 -V
cd ./ZTunnel3/
../pmpl -f ZTunnel3.Uniform.48.xml >& ZTunnel3.Uniform.48.log
