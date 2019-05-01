./waf --run "scratch/testeMCS --simulationTime=50000 --algorithm=RALL --nNodes=35 --nRadios=3 --seed=5 --repeatId=1 --activeFlows=100.0 --waveformPower=0.10 --nNodesInterfering=2" &
./waf --run "scratch/testeMCS --simulationTime=50000 --algorithm=BPR --nNodes=35 --nRadios=3 --seed=5 --repeatId=1 --activeFlows=100.0 --waveformPower=0.10 --nNodesInterfering=2"
