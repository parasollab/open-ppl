#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel4.Toggle_RRT.46 -V
cd ./ZTunnel4/
../pmpl -f ZTunnel4.Toggle_RRT.46.xml >& ZTunnel4.Toggle_RRT.46.log
