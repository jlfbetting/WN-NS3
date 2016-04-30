#!/bin/bash
# Script for simulating throughput from -100 to 100 m on the x axis, steps of 4 m, by J.L.F. Betting

for ((i = -100; i <= 100; i = i+4))
do
./waf --run "scratch/chapter3 --distance=$i --txPwr=110 --printOnScreen=0";
done
