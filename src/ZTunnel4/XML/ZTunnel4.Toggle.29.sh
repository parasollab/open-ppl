#!/bin/bash
#PBS -q parallel -l mem=1000mb -l walltime=10:00:00 -N ZTunnel4.Toggle.29 -V
cd ./ZTunnel4/
../pmpl -f ZTunnel4.Toggle.29.xml >& ZTunnel4.Toggle.29.log
