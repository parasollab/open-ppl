#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel3.Gauss_RRT.12 -V
cd ./ZTunnel3/
../pmpl -f ZTunnel3.Gauss_RRT.12.xml >& ZTunnel3.Gauss_RRT.12.log
