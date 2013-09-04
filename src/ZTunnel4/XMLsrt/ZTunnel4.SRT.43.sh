#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel4.SRT.43 -V
cd ./ZTunnel4/
../pmpl -f ZTunnel4.SRT.43.xml >& ZTunnel4.SRT.43.log
