#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel4.Gauss_RRT.39 -V
cd ./ZTunnel4/
../pmpl -f ZTunnel4.Gauss_RRT.39.xml >& ZTunnel4.Gauss_RRT.39.log
