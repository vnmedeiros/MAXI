#!/bin/bash
#--waveformPower=0.10
for seed in `seq 1 4`;
do
	./waf --run "scratch/testeMCS --simulationTime=50000 --algorithm=RALL --nNodes=30 --nRadios=3 --seed=$seed --repeatId=1 --activeFlows=100.0 --waveformPower=0.0 --nNodesInterfering=0" > /dev/null &
	./waf --run "scratch/testeMCS --simulationTime=50000 --algorithm=RALL --nNodes=30 --nRadios=3 --seed=$seed --repeatId=2 --activeFlows=100.0 --waveformPower=0.0 --nNodesInterfering=0" > /dev/null &
	./waf --run "scratch/testeMCS --simulationTime=50000 --algorithm=RALL --nNodes=30 --nRadios=3 --seed=$seed --repeatId=3 --activeFlows=100.0 --waveformPower=0.0 --nNodesInterfering=0" > /dev/null &
	./waf --run "scratch/testeMCS --simulationTime=50000 --algorithm=RALL --nNodes=30 --nRadios=3 --seed=$seed --repeatId=4 --activeFlows=100.0 --waveformPower=0.0 --nNodesInterfering=0" > /dev/null &
	wait
	
	./waf --run "scratch/testeMCS --simulationTime=50000 --algorithm=RALL --nNodes=30 --nRadios=3 --seed=$seed --repeatId=5 --activeFlows=100.0 --waveformPower=0.0 --nNodesInterfering=0" > /dev/null &
	./waf --run "scratch/testeMCS --simulationTime=50000 --algorithm=RALL --nNodes=30 --nRadios=3 --seed=$seed --repeatId=6 --activeFlows=100.0 --waveformPower=0.0 --nNodesInterfering=0" > /dev/null &
	./waf --run "scratch/testeMCS --simulationTime=50000 --algorithm=RALL --nNodes=30 --nRadios=3 --seed=$seed --repeatId=7 --activeFlows=100.0 --waveformPower=0.0 --nNodesInterfering=0" > /dev/null &
	./waf --run "scratch/testeMCS --simulationTime=50000 --algorithm=RALL --nNodes=30 --nRadios=3 --seed=$seed --repeatId=8 --activeFlows=100.0 --waveformPower=0.0 --nNodesInterfering=0" > /dev/null &
	wait
	
	./waf --run "scratch/testeMCS --simulationTime=50000 --algorithm=RALL --nNodes=30 --nRadios=3 --seed=$seed --repeatId=9 --activeFlows=100.0 --waveformPower=0.0 --nNodesInterfering=0" > /dev/null &
	./waf --run "scratch/testeMCS --simulationTime=50000 --algorithm=RALL --nNodes=30 --nRadios=3 --seed=$seed --repeatId=10 --activeFlows=100.0 --waveformPower=0.0 --nNodesInterfering=0" > /dev/null &
	wait
done

