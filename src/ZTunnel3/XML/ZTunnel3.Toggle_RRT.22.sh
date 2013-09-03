#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel3.Toggle_RRT.22 -V
cd ./ZTunnel3/
../pmpl -f ZTunnel3.Toggle_RRT.22.xml >& ZTunnel3.Toggle_RRT.22.log
