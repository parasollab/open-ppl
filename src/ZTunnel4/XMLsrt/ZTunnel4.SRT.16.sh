#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel4.SRT.16 -V
cd ./ZTunnel4/
../pmpl -f ZTunnel4.SRT.16.xml >& ZTunnel4.SRT.16.log
