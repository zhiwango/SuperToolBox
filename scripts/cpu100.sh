#!/bin/bash

N=`cat /proc/cpuinfo | grep "physical id" |wc -l`

List=`seq 1 $N`

for i in $List ; do
    dd if=/dev/zero of=/dev/null &
done


