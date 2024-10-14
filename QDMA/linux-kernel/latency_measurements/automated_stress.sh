#!/bin/bash

for i in {0..36}
do
    stress --io "$i" --timeout 7200 -q &
    stress_pid=$!
    sleep 2 
    start_time=$(date +%s)
    make realtime_irq_mmr ARGS="7 1000000 wq_u_hi_stress_io_$i"
    end_time=$(date +%s)
    elapsed=$(( end_time - start_time ))
    echo "$elapsed"
    kill $stress_pid 2>/dev/null
    wait $stress_pid 2>/dev/null
    echo "stress done"
done
