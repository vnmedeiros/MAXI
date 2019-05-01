#!/bin/bash

if [ $# != 3 ]
then
	echo "Invalid arguments"
	echo "Usage: ./run_teste.sh [ALGORITHM] [N_NODES] [N_RADIOS]"
	echo "Example: ./run_misc.sh RALL 40 3"
	exit
fi

for repeatIR in `seq 1 3`;
do
	./waf --run "scratch/testeOfdm --simulationTime=50000 --algorithm=$1 --nNodes=$2 --nRadios=$3 --seed=9 --repeatId=$repeatIR --activeFlows=100.0 --waveformPower=0.00 --nNodesInterfering=2" > /dev/null &
        ./waf --run "scratch/testeOfdm --simulationTime=50000 --algorithm=$1 --nNodes=$2 --nRadios=$3 --seed=10 --repeatId=$repeatIR --activeFlows=100.0 --waveformPower=0.00 --nNodesInterfering=2" > /dev/null ;
#	for seed in `seq 7 10`;
#	do		
#		./waf --run "scratch/testeOfdm --simulationTime=50000 --algorithm=$1 --nNodes=$2 --nRadios=$3 --seed=$seed --repeatId=$repeatIR --activeFlows=100.0 --waveformPower=0.00 --nNodesInterfering=2"
#	done	
done

