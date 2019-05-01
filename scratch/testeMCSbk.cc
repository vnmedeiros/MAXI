/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 MIRKO BANCHI
 * Copyright (c) 2015 University of Washington
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Authors: Vinicius Nunes <vnicius.nm.ba@gmail.com> <viniciusnunesmedeiros@inf.ufg.br>
 *
 */

#include <sstream>
#include <iomanip>
#include <vector>
#include <map>
#include <utility>      // std::pair, std::make_pair
#include <algorithm>
#include <string>
#include <cmath>        // std::abs
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "ns3/core-module.h"
#include "ns3/config-store-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/internet-module.h"
#include "ns3/lab-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/gnuplot.h"

// This is a routing RALL or BPR example of an IEEE 802.11n Wi-Fi adhoc network.
//
// The main use case is to enable and test SpectrumWifiPhy vs YansWifiPhy
// Users may vary the following command-line arguments in addition to the
// attributes, global values, and default values typically available:
//
//    --simulationTime:  Simulation time in seconds [600]
//    --wifiType:        select ns3::SpectrumWifiPhy or ns3::YansWifiPhy [ns3::SpectrumWifiPhy]
//    --errorModelType:  select ns3::NistErrorRateModel or ns3::YansErrorRateModel [ns3::NistErrorRateModel]
//

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("RALL_BPR");

std::map < std::pair<int,int>,	std::vector<struct MatrixQuality> > matrixQuality;

enum SetupType {
  SETUP_DISABLE_CHANNELS,
  SETUP_CHANNELS,
  SETUP_ALGORITHM,
  SETUP_UPDATE_GRAPH_EDGES,
  SETUP
};

struct MatrixQuality {
	double MMS;
	double MME;
	double SNR;
	double SINR;
	double MOD;
	double PER;
	bool RECEIVED;
};

typedef struct _config {
	double 		simulationTime;
	uint32_t 	channelWidth;
	std::string wifiType;
	std::string errorModelType;
	int 		nNodes;
	int 		nRadios;
	int 		nChannels;
	//Parameters to determine area:
	int			quadrantDivision;
	int 		quadrantI;
	int 		quadrantJ;
	double 		sizeArea;
	double 		minimumDistance;
	double 		maximumDistance;
	char 		algorithmRun[20];
	int 		seed;
	double		percentageActiveFlows;

} CONFIG;

typedef struct _reportResults {
	int m_lostPackets;		//Total packets lost
	int m_rxPackets;		//Total packets received
	double m_throughput;	//Average throughput
	double m_delayAvg;		//Average end-to-end time
	double m_jitterAvg;		//Average end-to-end jitter
	int m_MacTxDrop; 		//A packet has been dropped in the MAC layer before being queued for transmission.
	int m_MacRxDrop; 		//A packet has been dropped in the MAC layer after it has been passed up from the physical layer.
	int m_PhyTxDrop; 		//Trace source indicating a packet has been dropped by the device during transmission.
	int m_PhyRxDrop; 		//Trace source indicating a packet has been dropped by the device during reception.
	int m_ArpDrop;			//Trace source indicating a packet has been dropped by ARP protocol.
	unsigned int m_QuantityHops; //Trace source indicating a quantity hops for packet received.
	int recalculates;
	int qtdLinkUpTrue;
	int qtdLinkUpFalse;
	int totalLowQuality;

} REPORT_RESULTS;

typedef struct _elements {
	NodeContainer wifiNodes;
	Ipv4InterfaceContainer wifiNodeInterface;
	Ipv4JointRoutingHelper ipv4RoutingHelper;
	Ptr<MultiModelSpectrumChannel> spectrumChannel;
	algorithm::LABORARouting * routing;
	element::Graph * graph;
	int * map;
} ELEMENTS;

typedef struct _simulation {
	REPORT_RESULTS reportResultsInstance = {0,0,0.0,0.0,0.0,0,0,0,0,0,0,0,0,0};
	CONFIG config = {/*simulationTime*/5000.0,/*channelWidth*/20,/*wifiType*/"ns3::SpectrumWifiPhy"/*"ns3::YansWifiPhy"*/,/*errorModelType*/"ns3::NistErrorRateModel"
					,/*N_NODES*/20,/*N_RADIOS*/3,/*N_CHANNELS*/8,/*QUADRANT_DIVISION*/3,/*QUADRANT_I*/2,/*QUADRANT_J*/2
					,/*SIZE_AREA*/400,/*MINIMUM_DISTANCE*/20.0,/*MAXIMUM_DISTANCE*/37.0,/*algorithm_run[20]*/"RALL",/*seed*/3,/*active*/100.0};
	ELEMENTS elements;
} SIMULATION;

SIMULATION sm;
int HopsClass[4][2];

void QualityVariationPlotFile ( );

static void UpdateQualityLink (int, int, double, double, double, bool);
static int* setChannels(algorithm::CRAA *, NodeContainer, uint32_t);

static void launchInitializeQuality(uint16_t, double);
static void launchGenerateMap(uint16_t, double);
static void setups(SetupType);
static void FlowGenerator(int, double, double);
static void addInterferingNodes(double, double, int nNodesInterfering);

void   ArpDrop(Ptr<const Packet> p) {sm.reportResultsInstance.m_ArpDrop++;}
void MacTxDrop(Ptr<const Packet> p) {sm.reportResultsInstance.m_MacTxDrop++;}
void MacRxDrop(Ptr<const Packet> p) {sm.reportResultsInstance.m_MacRxDrop++;}
void PhyTxDrop(Ptr<const Packet> p) {sm.reportResultsInstance.m_PhyTxDrop++;}
void PhyRxDrop(Ptr<const Packet> p) {
	InfoTag infoTag;
	if ( p->PeekPacketTag(infoTag) && TypeTag::TAG_DATA == infoTag.GetTypeTag())
		sm.reportResultsInstance.m_PhyRxDrop++;
}

unsigned int countUpdates = 0;
static void callRecalculateRoutes() {
	//if (sm.config.activeFlows == false)
	//	return;
	countUpdates++;
	if (countUpdates == 10) {
		sm.reportResultsInstance.recalculates++;
		algorithm::RALL * rall = (algorithm::RALL *)sm.elements.routing;
		rall->recreatesAllRoutes();

		int totalLowQuality = 0;
		for (unsigned int i = 0; i < sm.elements.graph->listEdges.size(); i++) {
			for(unsigned int j = 0; j < sm.elements.graph->listEdges[i].size(); j++) {
				if (sm.elements.graph->listEdges[i][j].quality <= algorithm::RALL::THRESHOLD_QUALITY ) {
					totalLowQuality++;
				}
			}
		}
		sm.reportResultsInstance.totalLowQuality += totalLowQuality;
		countUpdates = 0;
	}
}

int main(int argc, char *argv[]) {
	unsigned int repeatId = 5;
	double waveformPower = 0.01;
	int nNodesInterfering = 2;
	CommandLine cmd;
	cmd.AddValue("simulationTime", 	"Simulation time in seconds", 						sm.config.simulationTime);
	cmd.AddValue("wifiType", 		"select ns3::SpectrumWifiPhy or ns3::YansWifiPhy", 	sm.config.wifiType);
	cmd.AddValue("errorModelType", 	"select ns3::NistErrorRateModel or ns3::YansErrorRateModel", sm.config.errorModelType);
	cmd.AddValue ("algorithm", 		"Algorithm", 	sm.config.algorithmRun);
	cmd.AddValue ("nNodes", 		"nNodes", 		sm.config.nNodes);
	cmd.AddValue ("seed", 			"Seed", 		sm.config.seed);
	cmd.AddValue ("nRadios", 		"nRadios", 		sm.config.nRadios);
	cmd.AddValue ("activeFlows",	"activeFlows", 	sm.config.percentageActiveFlows);
	cmd.AddValue ("repeatId", 		"repeatId", 	repeatId);
	cmd.AddValue ("waveformPower", 	"waveformPower",waveformPower);
	cmd.AddValue ("nNodesInterfering","nNodesInterfering",nNodesInterfering);

	cmd.Parse(argc, argv);

	ns3::SeedManager::SetSeed(sm.config.seed);
	srand(repeatId);

	std::cout 	<< "# wifiType=" 	<< sm.config.wifiType 	<< " nodes="
				<< sm.config.nNodes << " distance min=" 	<< sm.config.minimumDistance
				<< "m distance max="<< sm.config.maximumDistance << "m"
				<< " seed: " 		<< sm.config.seed 		<< std::endl;

	sm.elements.map = (int *) malloc ( sizeof(int *) * (sm.config.nNodes * sm.config.nNodes));
	for (int i = 0; i < sm.config.nNodes * sm.config.nNodes; i++)
		sm.elements.map[i] = 0;

	//the node 0 is the sink
	sm.elements.wifiNodes.Create(sm.config.nNodes);

	YansWifiPhyHelper phy = YansWifiPhyHelper::Default();
	SpectrumWifiPhyHelper spectrumPhy = SpectrumWifiPhyHelper::Default();
	if (sm.config.wifiType == "ns3::YansWifiPhy") {
		YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
		Ptr<YansWifiChannel> channel = wifiChannel.Create();

		Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
		//Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel>();
		//lossModel->SetPathLossExponent (2.7);
		//lossModel->SetReference (1, 46.6777);
		channel->SetPropagationLossModel(lossModel);

		phy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
		phy.SetChannel(channel);
		phy.Set("ShortGuardEnabled", BooleanValue(false));
		phy.SetErrorRateModel(sm.config.errorModelType);
		//phy.Set ("TxPowerStart",DoubleValue(16));
		//phy.Set ("TxPowerEnd", DoubleValue(16));
		sm.config.channelWidth = 20;

		phy.Set ("TxGain", 		 DoubleValue (0) ); 	// set it to zero; otherwise, gain will be added
		phy.Set ("RxGain", 		 DoubleValue (0)); 		// set it to zero; otherwise, gain will be added
		phy.Set ("TxPowerStart", DoubleValue (10)); 	//changes default TxPower from 16.0206dBm to 10dBm
		phy.Set ("TxPowerEnd", 	 DoubleValue (10));  	//changes default TxPower from 16.0206dBm to 10dBm
		//--------------Energy Threshold and Cca Threshold Configuration------------------------------------------------------------------
		phy.Set ("EnergyDetectionThreshold", DoubleValue (-76.36232)); 		//dmax=150m
		phy.Set ("CcaMode1Threshold", 		 DoubleValue (-81.35987)); 		//dmax=200m

	} else if (sm.config.wifiType == "ns3::SpectrumWifiPhy") {
		sm.elements.spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

		//Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
		Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel>();
		lossModel->SetPathLossExponent (2.7);
		lossModel->SetReference (1, 46.6777);
		sm.elements.spectrumChannel->AddPropagationLossModel(lossModel);
		Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
		sm.elements.spectrumChannel->SetPropagationDelayModel(delayModel);

		spectrumPhy.SetPcapDataLinkType(SpectrumWifiPhyHelper::DLT_IEEE802_11_RADIO);
		spectrumPhy.SetChannel(sm.elements.spectrumChannel);
		spectrumPhy.Set ("ShortGuardEnabled", BooleanValue(true));
		//spectrumPhy.Set ("GuardInterval", TimeValue(NanoSeconds (1600)));
		spectrumPhy.SetErrorRateModel(sm.config.errorModelType);
		spectrumPhy.Set ("Frequency", UintegerValue(/*2412*/5180));
        spectrumPhy.Set ("ChannelNumber", UintegerValue (36));

		sm.config.channelWidth = 40;

		spectrumPhy.Set ("TxGain", 		 	DoubleValue (0) ); 	// set it to zero; otherwise, gain will be added
		spectrumPhy.Set ("RxGain", 			DoubleValue (0) ); 	// set it to zero; otherwise, gain will be added
		spectrumPhy.Set ("TxPowerStart", 	DoubleValue (22)); 	//changes default TxPower from 16.0206dBm to 10dBm //22 36mb - 20nós
		spectrumPhy.Set ("TxPowerEnd", 		DoubleValue (22));  //changes default TxPower from 16.0206dBm to 10dBm //22 36mb - 20nós
		//--------------Energy Threshold and Cca Threshold Configuration------------------------------------------------------------------
		spectrumPhy.Set ("EnergyDetectionThreshold", DoubleValue(-78.0)); //dmax = 150m
		spectrumPhy.Set ("CcaMode1Threshold", 		 DoubleValue(-62.0)); //dmax=200m
		// Set MIMO capabilities
        spectrumPhy.Set ("TxAntennas", UintegerValue (2));
        spectrumPhy.Set ("RxAntennas", UintegerValue (2));

		
	} else {
		NS_FATAL_ERROR("Unsupported WiFi type " << sm.config.wifiType);
	}

	WifiHelper wifi;
	wifi.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);
	//wifi.SetStandard(WIFI_PHY_STANDARD_80211n_2_4GHZ);
	//wifi.SetStandard(WIFI_PHY_STANDARD_80211g);
	//wifi.SetStandard(WIFI_PHY_STANDARD_80211a);
	//wifi.SetStandard(WIFI_PHY_STANDARD_80211ac);


	Ssid ssid = Ssid("NS3-80211n");

	StringValue DataRate;
	//DataRate = StringValue("HtMcs1");
	//DataRate = StringValue("DsssRate11Mbps");
	//DataRate = StringValue("ErpOfdmRate54Mbps");
	//DataRate = StringValue("ErpOfdmRate36Mbps");
	//DataRate = StringValue("OfdmRate36Mbps");
	DataRate = StringValue("HtMcs4");

	wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
			"DataMode", 		DataRate, 				//
			"ControlMode", 		DataRate, 				//
			"NonUnicastMode", 	DataRate, 				//Wifi mode used for non-unicast transmissions.
			"MaxSsrc", 			UintegerValue(0), 		//The maximum number of retransmission attempts for an RTS.
			"MaxSlrc", 			UintegerValue(7), 		//The maximum number of retransmission attempts for a DATA packet.
			"RtsCtsThreshold", 	UintegerValue(65535), 	//If the size of the data packet + LLC header + MAC header + FCS trailer is bigger than this value, we use an RTS/CTS handshake before sending the data.
			"FragmentationThreshold", StringValue("2346"));//2346 Disable fragmentation for frames below 10000 bytes.

	Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (DataRate));

	Config::SetDefault("ns3::WifiMacQueue::MaxDelay", TimeValue(MilliSeconds(200)));
	//Config::SetDefault("ns3::WifiMacQueue::MaxPacketNumber", UintegerValue(1000));
	Config::SetDefault("ns3::ArpCache::PendingQueueSize", UintegerValue (800));

	//Create WifiMac for each wifi node:
	HtWifiMacHelper mac = HtWifiMacHelper::Default();
	//NqosWifiMacHelper mac = NqosWifiMacHelper::Default();
	mac.SetType ("ns3::AdhocWifiMac", "Ssid", SsidValue(ssid));
	

	//mobility.
	Ptr<RandomRectanglePositionAllocatorMinMax> pa = new RandomRectanglePositionAllocatorMinMax(sm.config.sizeArea, sm.config.quadrantDivision);
	pa->SetQuadrants(sm.config.quadrantI, sm.config.quadrantJ);
	MyMobilityHelper mobility;
	mobility.SetPositionAllocator(pa);
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.SetNNodes(sm.config.nNodes);
	mobility.SetMinDist(sm.config.minimumDistance);
	mobility.SetMaxDist(sm.config.maximumDistance);
	mobility.Install(sm.elements.wifiNodes);
	if (!pa->verifyAllocation(mobility, sm.config.nNodes, sm.config.minimumDistance, sm.config.maximumDistance)) {
		std::cout << "verify Allocation not correct!";
		exit(1);
	}

	//install radios in nodes:
	NetDeviceContainer wifiDevice[sm.config.nRadios];
	for (int i = 0; i < sm.config.nRadios; i++) {
		if (sm.config.wifiType == "ns3::YansWifiPhy")
			wifiDevice[i] = wifi.Install(phy, mac, sm.elements.wifiNodes);
		else if (sm.config.wifiType == "ns3::SpectrumWifiPhy")
			wifiDevice[i] = wifi.Install(spectrumPhy, mac, sm.elements.wifiNodes);
	}
	// Channel width must be set *after* installation because the attribute is overwritten by the ConfigureStandard method ()
	Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue(sm.config.channelWidth));

	//Create stack protocols:
	InternetStackHelper stack;
	Ipv4ListRoutingHelper list;
	list.Add(sm.elements.ipv4RoutingHelper, 0);
	stack.SetRoutingHelper(list);
	stack.Install(sm.elements.wifiNodes);

	//address:
	Ipv4AddressHelper address;
	address.SetBase("10.1.1.0", "255.255.255.255");
	for (int i = 0; i < sm.config.nRadios; i++) { // \FAZER TESTAR A QUANTIDADE DE RADIOS!
		if (i == 0) {
			sm.elements.wifiNodeInterface = address.Assign(wifiDevice[i]);
		} else {
			address.Assign(wifiDevice[i]);
		}
	}

	//Traces
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop", MakeCallback(&MacTxDrop));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxDrop", MakeCallback(&MacRxDrop));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&PhyRxDrop));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&PhyTxDrop));
	Config::ConnectWithoutContext("/NodeList/*/$ns3::ArpL3Protocol/Drop", MakeCallback(&ArpDrop));

	uint16_t quantityPackages = 10;
	uint16_t quantityPackagesQuality = 100;
	double timeInterval = 0.5;
	double timeFinishGenerateMap = sm.config.nNodes * (sm.config.nRadios * quantityPackages * timeInterval);
	double timeFinishInitQuality = timeFinishGenerateMap + (quantityPackagesQuality * (0.5 + timeInterval));

	//waveformPower = 0.60;
	if (waveformPower > 0)
		addInterferingNodes(waveformPower,timeFinishGenerateMap, nNodesInterfering);

	Simulator::Schedule(Seconds(0.0), &launchGenerateMap, quantityPackages, timeInterval);
	Simulator::Schedule(Seconds(timeFinishGenerateMap + 1), &setups, SetupType::SETUP_ALGORITHM);
	Simulator::Schedule(Seconds(timeFinishGenerateMap + 2), &setups, SetupType::SETUP_DISABLE_CHANNELS);
	Simulator::Schedule(Seconds(timeFinishGenerateMap + 3), &setups, SetupType::SETUP_CHANNELS);
	Simulator::Schedule(Seconds(timeFinishGenerateMap + 4), &launchInitializeQuality, quantityPackagesQuality, timeInterval);
	Simulator::Schedule(Seconds(timeFinishInitQuality + 5), &setups, SetupType::SETUP_UPDATE_GRAPH_EDGES);
	Simulator::Schedule(Seconds(timeFinishInitQuality + 50), &FlowGenerator, 0, 1.0, sm.config.percentageActiveFlows);
	//FlowGenerator( 0, t);

	//Install FlowMonitor on all nodes
	FlowMonitorHelper flowmon;
	Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
	monitor->SetAttribute("DelayBinWidth", DoubleValue (0.100));
	monitor->SerializeToXmlFile("FlowMonitor.xml", true, true);

	//Install Animation
	//AnimationInterface anim ("animationTESTE.xml"); // Mandatory
	//anim.EnableIpv4RouteTracking ("routingtable-wireless-RALL.xml", Seconds (0), Seconds (5), Seconds (0.25)); //Optional
	//anim.EnablePacketMetadata (); // Optional

	int packetsClass[4][2];
	int delayClass[4][2];
	int jitterClass[4][2];
	double throughputClass[4][2];
	for(int i = 0; i < 4; i++) {
		packetsClass[i][0] = 0;
		packetsClass[i][1] = 0;
		delayClass[i][0] = 0;
		delayClass[i][1] = 0;
		jitterClass[i][0] = 0;
		jitterClass[i][1] = 0;
		throughputClass[i][0] = 0;
		throughputClass[i][1] = 0;
		HopsClass[i][0] = 0;
		HopsClass[i][1] = 0;
	}

	std::cout << "Start...\n";
	Simulator::Stop(Seconds(sm.config.simulationTime + 1));
	Simulator::Run();

	//Print per flow statistics
	int count = 0;
	Histogram delayHistogram(0.100);
	monitor->CheckForLostPackets();
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
	FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i) {

		Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
		/*std::cout << "\nFlow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << "["<< t.sourcePort << "," << t.destinationPort <<"])\n";
		std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
		std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
		std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024 << " Mbps\n";
		std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
		std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
		std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024 << " Mbps\n";
		std::cout << "  lost Packets: " << i->second.lostPackets << "\n";*/

		if (t.sourcePort == 41) {
			packetsClass[0][0] += i->second.rxPackets;
			packetsClass[0][1] += i->second.lostPackets;
			throughputClass[0][0] += (i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) /*/ 1024 / 1024*/);
			throughputClass[0][1]++;
			if(i->second.rxPackets > 0) {
				delayClass[0][0] += i->second.delaySum.GetMilliSeconds() / i->second.rxPackets;
				delayClass[0][1]++;
				if (i->second.rxPackets > 1) {
					jitterClass[0][0] += i->second.jitterSum.GetMilliSeconds() / (i->second.rxPackets-1);
					jitterClass[0][1]++;
				}
			}
		} if (t.sourcePort == 42) {
			packetsClass[1][0] += i->second.rxPackets;
			packetsClass[1][1] += i->second.lostPackets;
			throughputClass[1][0] += (i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) /*/ 1024 / 1024*/);
			throughputClass[1][1]++;
			if(i->second.rxPackets > 0) {
				delayClass[1][0] += i->second.delaySum.GetMilliSeconds() / i->second.rxPackets;
				delayClass[1][1]++;
				if (i->second.rxPackets > 1) {
					jitterClass[1][0] += i->second.jitterSum.GetMilliSeconds() / (i->second.rxPackets-1);
					jitterClass[1][1]++;
				}
			}
		} if (t.sourcePort == 43) {
			packetsClass[2][0] += i->second.rxPackets;
			packetsClass[2][1] += i->second.lostPackets;
			throughputClass[2][0] += (i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) /*/ 1024 / 1024*/);
			throughputClass[2][1]++;
			if(i->second.rxPackets > 0) {
				delayClass[2][0] += i->second.delaySum.GetMilliSeconds() / i->second.rxPackets;
				delayClass[2][1]++;
				if (i->second.rxPackets > 1) {
					jitterClass[2][0] += i->second.jitterSum.GetMilliSeconds() / (i->second.rxPackets-1);
					jitterClass[2][1]++;
				}
			}
		} if (t.sourcePort == 44) {
			packetsClass[3][0] += i->second.rxPackets;
			packetsClass[3][1] += i->second.lostPackets;
			throughputClass[3][0] += (i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) /*/ 1024 / 1024*/);
			throughputClass[3][1]++;
			if(i->second.rxPackets > 0) {
				delayClass[3][0] += i->second.delaySum.GetMilliSeconds() / i->second.rxPackets;
				delayClass[3][1]++;
				if (i->second.rxPackets > 1) {
					jitterClass[3][0] += i->second.jitterSum.GetMilliSeconds() / (i->second.rxPackets-1);
					jitterClass[3][1]++;
				}
			}
		}

		if(i->second.rxPackets > 0) {
			sm.reportResultsInstance.m_delayAvg += i->second.delaySum.GetMilliSeconds() / i->second.rxPackets;
			if (i->second.rxPackets > 1) {
				sm.reportResultsInstance.m_jitterAvg += i->second.jitterSum.GetMilliSeconds() / (i->second.rxPackets -1);
			}
			count++;
		}


		sm.reportResultsInstance.m_lostPackets += i->second.lostPackets;
		sm.reportResultsInstance.m_rxPackets += i->second.rxPackets;
		sm.reportResultsInstance.m_throughput += (i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) /*/ 1024 / 1024*/);

		Histogram dh = i->second.delayHistogram;
		for (unsigned int t = 0; t < dh.GetNBins(); t++) {
			//double value = delayHistogram.GetBinWidth(0) * dh.GetBinStart(t);
			double value = dh.GetBinStart(t);
			for (unsigned int u = 0; u < dh.GetBinCount(t); u++)
				delayHistogram.AddValue(value);
		}
	}

	char dataFileName[50];
		sprintf(dataFileName,"plot/%s-%d-R%d_%d.dat", sm.config.algorithmRun ,sm.config.nNodes, sm.config.nRadios, nNodesInterfering);
		struct stat st;
		int hd = stat(dataFileName, &st);
		std::ofstream dataFile (dataFileName, std::ofstream::app);

		if (hd == -1) {
			dataFile << "SEED;\t"
					 << "     RX Packets "
					 << "   Lost Packets "
					 << " % Lost Packets "
					 << " % Lost PcktsC1 "
					 << " % Lost PcktsC2 "
					 << " % Lost PcktsC3 "
					 << " % Lost PcktsC4 "
					 << "throughput(Mbps)"
					 << "thghputC1(Mbps) "
					 << "thghputC2(Mbps) "
					 << "thghputC3(Mbps) "
					 << "thghputC4(Mbps) "
					 << "  AVG Delay(ms) "
					 << " AVGDelay(ms)C1 "
					 << " AVGDelay(ms)C2 "
					 << " AVGDelay(ms)C3 "
					 << " AVGDelay(ms)C4 "
					 << " AVG Jitter(ms) "
					 << " AVGJiter(ms)C1 "
					 << " AVGJiter(ms)C2 "
					 << " AVGJiter(ms)C3 "
					 << " AVGJiter(ms)C4 "
					 << "       AVG HOPS "
					 << "    AVG HOPS_C1 "
					 << "    AVG HOPS_C2 "
					 << "    AVG HOPS_C3 "
					 << "    AVG HOPS_C4 "
					 << "      MacRxDrop "
					 << "      MacTxDrop "
					 << "      PhyRxDrop "
					 << "      PhyTxDrop "
					 << "        ArpDrop "
					 << "   recalculates "
					 << "  qtdLinkUpTrue "
					 << " qtdLinkUpFalse "
					 << "     LowQuality           ";

				for (unsigned int t = 0; t < delayHistogram.GetNBins(); t++) {
					dataFile << "[" << delayHistogram.GetBinStart(t) * 1000 << "-" <<  delayHistogram.GetBinEnd(t) * 1000 << "];";
				}
			dataFile << std::endl;
		}

		dataFile << sm.config.seed << "\t"
				 << std::setfill (' ') << std::setw (15) << sm.reportResultsInstance.m_rxPackets << " "
				 << std::setfill (' ') << std::setw (15) << sm.reportResultsInstance.m_lostPackets << " "
				 << std::setfill (' ') << std::setw (15) << (double)sm.reportResultsInstance.m_lostPackets/(sm.reportResultsInstance.m_lostPackets+sm.reportResultsInstance.m_rxPackets) << " "

				 << std::setfill (' ') << std::setw (15) << (double)packetsClass[0][1]/(packetsClass[0][0] + packetsClass[0][1]) << " "
				 << std::setfill (' ') << std::setw (15) << (double)packetsClass[1][1]/(packetsClass[1][0] + packetsClass[1][1]) << " "
				 << std::setfill (' ') << std::setw (15) << (double)packetsClass[2][1]/(packetsClass[2][0] + packetsClass[2][1]) << " "
				 << std::setfill (' ') << std::setw (15) << (double)packetsClass[3][1]/(packetsClass[3][0] + packetsClass[3][1]) << " "

				 << std::setfill (' ') << std::setw (15) << sm.reportResultsInstance.m_throughput/(sm.config.nNodes * sm.config.percentageActiveFlows / 100) / 1024 << " "

				 << std::setfill (' ') << std::setw (15) << throughputClass[0][0]/throughputClass[0][1]/1024 << " "
				 << std::setfill (' ') << std::setw (15) << throughputClass[1][0]/throughputClass[1][1]/1024 << " "
				 << std::setfill (' ') << std::setw (15) << throughputClass[2][0]/throughputClass[2][1]/1024 << " "
				 << std::setfill (' ') << std::setw (15) << throughputClass[3][0]/throughputClass[3][1]/1024 << " "

				 << std::setfill (' ') << std::setw (15) << sm.reportResultsInstance.m_delayAvg/count << " "

				 << std::setfill (' ') << std::setw (15) << (double)delayClass[0][0]/delayClass[0][1] << " "
				 << std::setfill (' ') << std::setw (15) << (double)delayClass[1][0]/delayClass[1][1] << " "
				 << std::setfill (' ') << std::setw (15) << (double)delayClass[2][0]/delayClass[2][1] << " "
				 << std::setfill (' ') << std::setw (15) << (double)delayClass[3][0]/delayClass[3][1] << " "

				 << std::setfill (' ') << std::setw (15) << sm.reportResultsInstance.m_jitterAvg/count << " "

				 << std::setfill (' ') << std::setw (15) << (double)jitterClass[0][0]/jitterClass[0][1] << " "
				 << std::setfill (' ') << std::setw (15) << (double)jitterClass[1][0]/jitterClass[1][1] << " "
				 << std::setfill (' ') << std::setw (15) << (double)jitterClass[2][0]/jitterClass[2][1] << " "
				 << std::setfill (' ') << std::setw (15) << (double)jitterClass[3][0]/jitterClass[3][1] << " "

				 << std::setfill (' ') << std::setw (15) << (double)sm.reportResultsInstance.m_QuantityHops/sm.reportResultsInstance.m_rxPackets << " "

				 << std::setfill (' ') << std::setw (15) << (double)HopsClass[0][0]/HopsClass[0][1]  << " "
				 << std::setfill (' ') << std::setw (15) << (double)HopsClass[1][0]/HopsClass[1][1]  << " "
				 << std::setfill (' ') << std::setw (15) << (double)HopsClass[2][0]/HopsClass[2][1]  << " "
				 << std::setfill (' ') << std::setw (15) << (double)HopsClass[3][0]/HopsClass[3][1]  << " "

				 << std::setfill (' ') << std::setw (15) << sm.reportResultsInstance.m_MacRxDrop << " "
				 << std::setfill (' ') << std::setw (15) << sm.reportResultsInstance.m_MacTxDrop << " "
				 << std::setfill (' ') << std::setw (15) << sm.reportResultsInstance.m_PhyRxDrop << " "
				 << std::setfill (' ') << std::setw (15) << sm.reportResultsInstance.m_PhyTxDrop << " "
				 << std::setfill (' ') << std::setw (15) << sm.reportResultsInstance.m_ArpDrop << " "
				 << std::setfill (' ') << std::setw (15) << sm.reportResultsInstance.recalculates << " "
				 << std::setfill (' ') << std::setw (15) << sm.reportResultsInstance.qtdLinkUpTrue << " "
				 << std::setfill (' ') << std::setw (15) << sm.reportResultsInstance.qtdLinkUpFalse << " "
				 << std::setfill (' ') << std::setw (15) << sm.reportResultsInstance.totalLowQuality << " "
				 << std::setfill (' ') << std::setw (15);

		for (unsigned int t = 0; t < delayHistogram.GetNBins(); t++) {
			dataFile << delayHistogram.GetBinCount(t) << ";";
		}
		//dataFile << " #" << std::setfill (' ') << std::setw (15) << waveformPower;

		dataFile << std::endl;
		dataFile.close();

	//QualityVariationPlotFile();

	Simulator::Destroy();
	free(sm.elements.map);

	return 0;
}

/*
 *\\***************************************************\\
 *\\			Implementation functions:			   \\
 *\\***************************************************\\
 */

static void GenerateTrafficForSocket(Ptr<Socket> socket, uint32_t sizePackages, uint32_t packagesCount, Time interval, TypeTag typeTag) {
	if (packagesCount > 0) {
		Ptr<Packet> pkt = Create<Packet>(sizePackages);
		InfoTag infoTag;
		infoTag.SetSrc(socket->GetNode()->GetId());
		infoTag.SetNodeId(socket->GetNode()->GetId());
		infoTag.SetId(packagesCount);
		infoTag.SetTypeTag(typeTag);
		pkt->AddPacketTag(infoTag);
		socket->Send(pkt);
		Simulator::Schedule(interval, &GenerateTrafficForSocket, socket, sizePackages, packagesCount - 1, interval, typeTag);
	} else {
		socket->Close();
	}
}

static void ReceiveTrafficForSocket(Ptr<Socket> socket) {
	Ptr<Packet> packet;
	while ((packet = socket->Recv())) {
		InfoTag infoTag;
		if ( packet->PeekPacketTag(infoTag) ) {
			packet->RemovePacketTag(infoTag);
			if (infoTag.GetSrc() != socket->GetNode()->GetId()) {
				sm.elements.map[infoTag.GetSrc() * sm.config.nNodes + socket->GetNode()->GetId()]++;
			}
		}
	}
}

static void launchGenerateMap(uint16_t quantityPackages, double timeInterval) {
	uint16_t sizePackages = 1024;
	double totalTime = timeInterval * quantityPackages;
	Time packetInterval = Seconds(timeInterval);
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

	Address BroadcastAddress (InetSocketAddress(Ipv4Address::GetBroadcast(), 80));
	InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);

	std::vector<Ptr<Socket>> sendSockets;
	std::vector<Ptr<Socket>> recvSockets;
	int t=0;
	for (int i=0; i < sm.config.nNodes; i++) {
		for (int r=0; r < sm.config.nRadios; r++, t++) {

			sendSockets.push_back(Socket::CreateSocket(sm.elements.wifiNodes.Get(i), tid));
			recvSockets.push_back(Socket::CreateSocket(sm.elements.wifiNodes.Get(i), tid));
			sendSockets[t]->Bind();
			sendSockets[t]->BindToNetDevice(sm.elements.wifiNodes.Get(i)->GetDevice(r));
			sendSockets[t]->Connect(BroadcastAddress);
			sendSockets[t]->SetAllowBroadcast(true);
			sendSockets[t]->ShutdownRecv();

			recvSockets[t]->Bind(local);
			recvSockets[t]->ShutdownSend();
			recvSockets[t]->SetRecvCallback(MakeCallback (&ReceiveTrafficForSocket));

			double time = (double)  i * (sm.config.nRadios * totalTime) + r * totalTime ;
			Simulator::ScheduleWithContext(sendSockets[t]->GetNode()->GetId(),
											Seconds(time), &GenerateTrafficForSocket,
											sendSockets[t], sizePackages, quantityPackages,
											packetInterval, TypeTag::TAG_PRELOAD);
		}
	}
	std::cout << "[Generate Map: \033[1;34mok\033[0m ] \n";
}

static void launchInitializeQuality(uint16_t quantityPackages, double timeInterval) {
	for (int i = 0; i < sm.config.nNodes * sm.config.nNodes; i++)
		sm.elements.map[i] = 0;

	uint16_t sizePackages = 1024;
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

	Address BroadcastAddress (InetSocketAddress(Ipv4Address::GetBroadcast(), 80));
	InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);

	std::vector<Ptr<Socket>> sendSockets;
	std::vector<Ptr<Socket>> recvSockets;
	int t=0;
	for (int i=0; i < sm.config.nNodes; i++) {
		for (int r=0; r < sm.config.nRadios; r++) {
			uint16_t ch = sm.elements.wifiNodes.Get(i)->GetDevice(r)->GetObject<WifiNetDevice>()->GetPhy()->GetObject<WifiPhy>()->GetChannelNumber();
			if (ch == uint16_t(0))
				continue;
			sendSockets.push_back(Socket::CreateSocket(sm.elements.wifiNodes.Get(i), tid));
			recvSockets.push_back(Socket::CreateSocket(sm.elements.wifiNodes.Get(i), tid));

			sendSockets[t]->Bind();
			sendSockets[t]->BindToNetDevice(sm.elements.wifiNodes.Get(i)->GetDevice(r));
			sendSockets[t]->Connect(BroadcastAddress);
			sendSockets[t]->SetAllowBroadcast(true);
			sendSockets[t]->ShutdownRecv();

			recvSockets[t]->Bind(local);
			recvSockets[t]->ShutdownSend();
			recvSockets[t]->SetRecvCallback(MakeCallback (&ReceiveTrafficForSocket));

			double time = (double)(rand() % 200)/200;
			Time packetInterval = Seconds(timeInterval+time/2);
			Simulator::ScheduleWithContext(sendSockets[t]->GetNode()->GetId(),
								Seconds(time), &GenerateTrafficForSocket,
								sendSockets[t], sizePackages, quantityPackages,
								packetInterval, TypeTag::TAG_PRELOAD);
			t++;
		}
	}
	std::cout << "[Initialize Quality: \033[1;34mok\033[0m] \n";
}

static void setups(SetupType st) {
	switch (st) {
		case SetupType::SETUP_ALGORITHM: {
			sm.elements.graph = new element::Graph(sm.elements.map, sm.config.nNodes);
			//sm.elements.graph->showAdjacencyList();
			//Create Routing:
			if (strcmp (sm.config.algorithmRun, "BPR") == 0) {
				algorithm::BPR * bpr = new algorithm::BPR(sm.config.nNodes, 2.5, sm.elements.graph);
				bpr->CalculateCandidatesPaths();
				sm.elements.routing = dynamic_cast<algorithm::LABORARouting * >(bpr);
			} else if (strcmp (sm.config.algorithmRun, "RALL") == 0) {
				sm.elements.routing = new algorithm::RALL(0.5, 0.5, sm.elements.graph);
				sm.elements.graph->m_callbackUpdateLink = &callRecalculateRoutes;
			} else {
				std::cout << "Routing algorithm	not correct!";
				exit(1);
			}
			std::cout << "[setup algorithm: \033[1;34mok\033[0m] \n";
			return;
		}
		break;

		case SetupType::SETUP_DISABLE_CHANNELS: {
			//setting an unused channel to all radios
			for (int i = 0; i < sm.config.nNodes; i++) {
				Ptr<ns3::Node> node = sm.elements.wifiNodes.Get(i);
				for (int j = 0; j < sm.config.nRadios; j++) {
					Ptr<WifiNetDevice> netDevice = node->GetDevice(j)->GetObject<WifiNetDevice>();
					Ptr<WifiPhy> wifiPhy = netDevice->GetPhy()->GetObject<WifiPhy>();
					wifiPhy->SetChannelNumber(uint16_t(0));
				}
			}
			std::cout << "[setup disable channels: \033[1;34mok\033[0m] \n";
			return;
		}
		break;

		case SetupType::SETUP_CHANNELS: {
			Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/UpdateLinkQuality", MakeCallback(&UpdateQualityLink));

			//Create CRAA:
			int channelDefault = 36;
			//int myChannels[]= {1,6,11,2,7,12,3,8,13,4,9,5,10};
			int myChannels[]= {36,60,112,132,40,64,116,136,44,100,120,140,48,104,124,144,52,108,128};
			std::set<CRAAChannel> channels(myChannels, myChannels+sm.config.nChannels);

			std::map< element::Edge, std::map< element::Edge, double> > per;
			algorithm::CRAA * craa = new algorithm::CRAA(sm.elements.routing, sm.elements.graph->U.size(), 0.0, channels, sm.config.nRadios, 2, per, channelDefault);
			craa->RunInitial();

			//Set initial channel:
			int *channelsToRadios = setChannels(craa, sm.elements.wifiNodes, sm.config.nNodes);
			craa->SetChannelsToRadios(channelsToRadios);
			//Create map between address and channel:
			std::map< std::pair<int, int>, Ipv4Address > nodesAddress;
			for (int i = 0; i < sm.config.nNodes; i++) {
				Ptr<ns3::Node> node = sm.elements.wifiNodes.Get(i);
				for (int j = 0; j < sm.config.nRadios; j++) {
					Ptr<WifiPhy> nodeWifiPhy = node->GetDevice(j)->GetObject<WifiNetDevice>()->GetPhy()->GetObject<WifiPhy>();
					int channel = nodeWifiPhy->GetChannelNumber();
					if(channel == 0) continue;
					std::pair<int, int> ic =  std::make_pair (i,channel);
					nodesAddress[ic] = node->GetObject<Ipv4>()->GetAddress(j+1,0).GetLocal();
				}
			}
			//Set Ipv4JointRouting:
			int nodeId = 0;
			for (int i = 0; i < sm.config.nNodes; i++, nodeId++) {
				Ptr<JointRouting> jointRouting = sm.elements.ipv4RoutingHelper.GetJointRouting(sm.elements.wifiNodes.Get(i)->GetObject<Ipv4>());
				jointRouting->setLRouting(sm.elements.routing);
				jointRouting->setRcaa(craa);
				jointRouting->setNodesAddress(nodesAddress);
				jointRouting->setNodeId(nodeId);
			}
			std::cout << "[setup channels: \033[1;34mok\033[0m] \n";
			return;
		}
		break;

		case SetupType::SETUP_UPDATE_GRAPH_EDGES: {
			sm.elements.graph->updateEdgesMatrix(sm.elements.map, sm.config.nNodes, 50);
			std::cout << "[setup update graphs edges: \033[1;34mok\033[0m] \n";
			return;
		}
		break;

		case SetupType::SETUP: {
			std::cout << "ORas...\n";
		}
		break;
	}

}

static void UpdateQualityLink (int sourceVertice, int destinationVertice, double snr, double sinr, double per, bool received) {
	bool updated = sm.elements.graph->qualityLinkUpdate(sourceVertice, destinationVertice, sinr/snr); //SINR
	//bool updated = graph->qualityLinkUpdate(sourceVertice, destinationVertice, 1-per); //PER
	element::Edge item(sourceVertice, destinationVertice, 0);
	std::vector<element::Edge>::iterator ed = std::find(sm.elements.graph->listEdges[sourceVertice].begin(),
														sm.elements.graph->listEdges[sourceVertice].end(),
														item);
	struct MatrixQuality mq;
	if ( ed != sm.elements.graph->listEdges[sourceVertice].end()) {
		mq.MME = (*ed).quality_MME;
		mq.MMS = (*ed).quality_MMS;
		mq.SNR = snr;
		mq.SINR = sinr;
		mq.PER = per;
		mq.RECEIVED = received;
		if (updated) {
			//traceValuesCount_instance.qtdLinkUpTrue++;
			mq.MOD = (*ed).quality;
		}
		else {
			//traceValuesCount_instance.qtdLinkUpFalse++;
			mq.MOD = matrixQuality[std::make_pair(sourceVertice,destinationVertice)].back().MOD;
		}
		matrixQuality[std::make_pair(sourceVertice,destinationVertice)].push_back(mq);
	}
}

static int* setChannels(algorithm::CRAA *craa, NodeContainer wifiNodes, uint32_t nNodes) {
	// creating channelsToRadios used in Ipv4JointRouting::LookupStatic2
	int *channelsToRadios = new int[nNodes * craa->m_nRadios];
	for (unsigned int i = 0; i < nNodes * craa->m_nRadios; i++) {
		channelsToRadios[i] = 0;
	}

	// setting channels to radios
	//std::cout << "Nodes - m_allocatedChannels:" << std::endl;
	for (unsigned int i = 0; i < nNodes; i++) {
		//std::cout << "Node " << i << " -- ";
		std::set<CRAAChannel> usedChannels = craa->m_channelsToNodes[i];
		//std::cout << "usedChannels = " << usedChannels.size() << ": ";
		if ((int) usedChannels.size() > craa->m_nRadios) {
			std::cout << "ERROR - usedChannels > m_nRadios" << std::endl;
			exit(1);
		}

		int radio = 0;
		for (std::set<CRAAChannel>::iterator it = usedChannels.begin(); it != usedChannels.end(); ++it, radio++) {
			CRAAChannel channel = *it;
			//std::cout << channel << " ";
			Ptr<ns3::Node> node;
			node = wifiNodes.Get(i);
			Ptr<WifiNetDevice> netDevice = node->GetDevice(radio)->GetObject<WifiNetDevice>();
			if (sm.config.wifiType == "ns3::SpectrumWifiPhy") {
				Ptr<SpectrumWifiPhy> spectrumWifiPhy = netDevice->GetPhy()->GetObject<SpectrumWifiPhy>();
				spectrumWifiPhy->SetChannelNumber(uint32_t(channel));
				sm.elements.spectrumChannel->AddRx(spectrumWifiPhy->GetSpectrumPhy());
			} else if (sm.config.wifiType == "ns3::YansWifiPhy") {
				Ptr<WifiPhy> wifiPhy = netDevice->GetPhy()->GetObject<WifiPhy>();
				wifiPhy->SetChannelNumber(uint32_t(channel));
			}

			channelsToRadios[i * craa->m_nRadios + radio] = channel;
		}
		//std::cout << std::endl;
	}
	//std::cout << std::endl;
//
//	std::cout << "channelsToRadios:" << std::endl;
//	for (unsigned int i = 0; i < nNodes; i++) {
//		std::cout << "Node " << i << ":" << std::endl;
//		Ptr<ns3::Node> node;
//		node = wifiNodes.Get(i);
//		for (int j = 0; j < craa->m_nRadios; j++) {
//			if (channelsToRadios[i * craa->m_nRadios + j] != 0) {
//				std::cout << "interface = " << j << " channel = " << channelsToRadios[i * craa->m_nRadios + j] << " Address: "
//						<< node->GetObject<Ipv4>()->GetAddress(j+1,0).GetLocal() << std::endl;
//			}
//		}
//	}
//	std::cout << "set_channels end." << std::endl;
//	std::cout << std::endl;

	return channelsToRadios;
}

static void ReceiveFlows(const Ipv4Header &header, Ptr<const Packet> p, uint32_t interface) {
	Ptr<Packet> packet = p->Copy(); // Make a copy of the packet
	InfoTag infoTag;
	if ( packet->PeekPacketTag(infoTag) )
		if (infoTag.GetTypeTag() == TypeTag::TAG_DATA) {
		Ipv4Header iph;
		UdpHeader udph;
		RoutingHeader jointRouteHeader;
		packet->RemoveHeader (iph);
		packet->RemoveHeader (udph);
		packet->PeekHeader(jointRouteHeader);
		sm.reportResultsInstance.m_QuantityHops += jointRouteHeader.GetHops().size();
		HopsClass[infoTag.GetClassApp()][0] += jointRouteHeader.GetHops().size();
		HopsClass[infoTag.GetClassApp()][1]++;
	}
}

static void FlowGenerator(int idSink, double startAt, double percentage) {
	NS_ASSERT (percentage > 0 && percentage <= 100);
	//sm.elements.graph->showAdjacencyList();
	std::vector<bool> activeFlows (sm.config.nNodes, false);
	int totalActive = sm.config.nNodes * percentage / 100;
	int i=0, iActive = 0;
	for (int j = 0; iActive < totalActive; j++) {
		i = j % sm.config.nNodes;
		if (activeFlows[i] == true)
			continue;
		activeFlows[i] = rand() % 2 == 0 ? ++iActive : false;
	}

	algorithm::RALL::TX_TOTAL_BALANCED = (totalActive-1) * (264 * 1000) + (1024 * 1000);

	PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 80));
	ApplicationContainer packetSink = packetSinkHelper.Install(sm.elements.wifiNodes.Get(idSink));
	packetSink.Start(Seconds(startAt));
	packetSink.Stop(Seconds(sm.config.simulationTime));

	Address sinkAddress (InetSocketAddress (sm.elements.wifiNodeInterface.GetAddress(idSink), 80));

	LaboraAppHelper laboraAppHelper_c2_1("ns3::UdpSocketFactory", sinkAddress, LaboraAppHelper::Class_2);
	LaboraAppHelper laboraAppHelper_c3_1("ns3::UdpSocketFactory", sinkAddress, LaboraAppHelper::Class_3);

	LaboraAppHelper laboraAppHelper_c2_2("ns3::UdpSocketFactory", sinkAddress, LaboraAppHelper::Class_2);
	LaboraAppHelper laboraAppHelper_c3_2("ns3::UdpSocketFactory", sinkAddress, LaboraAppHelper::Class_3);


	laboraAppHelper_c2_1.SetAttribute ("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
	laboraAppHelper_c2_1.SetAttribute ("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

	laboraAppHelper_c2_2.SetAttribute ("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
	laboraAppHelper_c2_2.SetAttribute ("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

	laboraAppHelper_c3_1.SetAttribute ("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
	laboraAppHelper_c3_1.SetAttribute ("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

	laboraAppHelper_c3_2.SetAttribute ("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
	laboraAppHelper_c3_2.SetAttribute ("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));


	ApplicationContainer source;
	for (int i = 1; i < sm.config.nNodes; i++) {
		if (activeFlows[i] ==  true) {
			source.Add(laboraAppHelper_c2_1.Install(sm.elements.wifiNodes.Get(i)));
			source.Add(laboraAppHelper_c2_2.Install(sm.elements.wifiNodes.Get(i)));
			source.Add(laboraAppHelper_c3_1.Install(sm.elements.wifiNodes.Get(i)));
			source.Add(laboraAppHelper_c3_2.Install(sm.elements.wifiNodes.Get(i)));
		}
	}

	source.Start(Seconds(startAt+0.1));
	source.Stop(Seconds(sm.config.simulationTime));

	Config::ConnectWithoutContext("/NodeList/*/$ns3::Ipv4L3Protocol/LocalDeliver", MakeCallback(&ReceiveFlows)); // \FAZER: Endenter a forma como os pacotes são passados para camada de cima, a aplicação do SINk não tá recebendo os pacotes.
	std::cout << "[FlowGenerator: \033[1;34mok\033[0m] \n\n";
}

Ptr<SpectrumModel> SpectrumModelWifi_MHz;

class static_SpectrumModelWifi_MHz_initializer {
public:
	static_SpectrumModelWifi_MHz_initializer(double bandMHz) {
		//see values the Frequency/ChannelWidth in wifi-phy.cc line 68
		BandInfo bandInfo;
		bandInfo.fc = bandMHz;
		bandInfo.fl = bandMHz - 10e6;
		bandInfo.fh = bandMHz + 10e6;

		Bands bands;
		bands.push_back(bandInfo);

		SpectrumModelWifi_MHz = Create<SpectrumModel>(bands);
	}
} static_SpectrumModelWifi_MHz_initializer_instance(/*2412e6*/5180e6);

static void addInterferingNodes(double waveformPower, double startAt, int nNodesInterfering) {
	// Configure waveform generator
	NodeContainer interferingNode;
	interferingNode.Create(nNodesInterfering);

	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
	//positionAlloc->Add(Vector(200.0, 400.0, 0.0));		//it is the starting position
	//positionAlloc->Add(Vector(600.0, 400.0, 0.0));		//it is the starting position
	//positionAlloc->Add(Vector(400.0, 200.0, 0.0));		//it is the starting position
	//positionAlloc->Add(Vector(400.0, 600.0, 0.0));		//it is the starting position
	positionAlloc->Add(Vector(10.0, 10.0, 0.0));			//it is the starting position
	positionAlloc->Add(Vector(390.0, 390.0, 0.0));		//it is the starting position
	positionAlloc->Add(Vector(10.0, 390.0, 0.0));		//it is the starting position
	positionAlloc->Add(Vector(390.0, 10.0, 0.0));		//it is the starting position
	MobilityHelper nodeMobilityHelper;
	nodeMobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	nodeMobilityHelper.SetPositionAllocator(positionAlloc);
	nodeMobilityHelper.Install(interferingNode);

	//double waveformPower = 0.0398;
	Ptr<SpectrumValue> wgPsd = Create<SpectrumValue>(SpectrumModelWifi_MHz);
	*wgPsd = waveformPower / (100 * 180000);
	std::cout << "wgPsd : " << *wgPsd << " integrated power: " << Integral (*(GetPointer (wgPsd))) << std::endl;

	WaveformGeneratorHelper waveformGeneratorHelper;
	waveformGeneratorHelper.SetChannel(sm.elements.spectrumChannel);
	waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd);
	//waveformGeneratorHelper.SetPhyAttribute("Period", TimeValue(Seconds(1.0)));
	waveformGeneratorHelper.SetPhyAttribute("DutyCycle", DoubleValue(1.0));
	//waveformGeneratorHelper.SetPhyAttribute("DutyCycle", DoubleValue(0.5));
	NetDeviceContainer waveformGeneratorDevices = waveformGeneratorHelper.Install(interferingNode);

	for(int i = 0; i < nNodesInterfering; i++)
		Simulator::Schedule(Seconds(startAt), &WaveformGenerator::Start, waveformGeneratorDevices.Get(i)->GetObject<NonCommunicatingNetDevice>()->GetPhy()->GetObject<WaveformGenerator>());
}






void QualityVariationPlotFile ( ) {
	struct stat st = {0};
	if (stat("plot", &st) == -1) {
		mkdir("plot", 0700);
	}
	if (stat("plot/qualityVariation", &st) == -1) {
		mkdir("plot/qualityVariation", 0700);
	}
	char prefixFile[10];
	sprintf(prefixFile,"%s-%d-seed:%d", sm.config.algorithmRun , sm.config.nNodes, sm.config.seed);

	std::string pre(prefixFile);
	std::string fileNameWithNoExtension = "qualityVariation-2d";
	std::string graphicsFileName        = fileNameWithNoExtension + ".png";
	std::string plotFileName            = "plot/qualityVariation/" + pre  + "_" + fileNameWithNoExtension + ".plt";
	int qtdGraph = matrixQuality.size();
	char style[250];

	sprintf(style,"COR_GRAY	= '#cbcbcb'\n"
				  "COR_RED 	= '#F15854'\n"
				  "COR_GREEN 	= '#60BD68'\n"
				  "COR_BLUE 	= '#5DA5DA'\n"
				  "COR_PURPLE 	= '#B276B2'\n"

				  "set style line 1 lc rgb COR_GRAY lt 1 lw 1 \n"
				  "set style line 2 lc rgb COR_GREEN lt 1 lw 1 \n"
				  "set style line 3 lc rgb COR_BLUE lt 1 lw 1 \n"
				  "set style line 4 lc rgb COR_PURPLE lt 1 lw 3 \n"

				  "unset key\n"
				  "set multiplot layout %d, 1\n"
				  "#set yrange [-0.1:1.1]\n"
				  "#set grid ytics mytics\n"
				  "#set mytics 4\n"
				  "#set grid\n", qtdGraph);

	// Instantiate the plot and set its title.
	Gnuplot plot (graphicsFileName);
	plot.AppendExtra(style);

	// Make the graphics file, which the plot file will create when it
	// is used with Gnuplot, be a PNG file.
	char term[20];
	sprintf(term, "png size 1500,%d", 333 * qtdGraph);
	plot.SetTerminal (term);

	// Set the labels for each axis.
	plot.SetLegend ("", "%");


	std::map<std::string, std::pair< int, int > > histogramQualityIR;
	std::map<std::string, std::pair< int, int > > histogramQualitySNR;
	std::map<std::string, std::pair< int, int > > histogramQualitySINR;

	// Open the plot file.
	std::ofstream plotFile (plotFileName.c_str());
	unsigned int len = 1;
	for(auto elem : matrixQuality) {
	  // Instantiate the dataset, set its title, and make the points be
	  // plotted along with connecting lines.
	  Gnuplot2dDataset dataset_SNR;
	  Gnuplot2dDataset dataset_SINR;
	  Gnuplot2dDataset dataset_IR;
	  Gnuplot2dDataset dataset_PER;
	  Gnuplot2dDataset dataset_MMS;
	  Gnuplot2dDataset dataset_MME;
	  Gnuplot2dDataset dataset_MOD;
	  Gnuplot2dDataset dataset_REC;
	  char title[20], titleSNR[] = "SNR", titleSINR[] = "SINR", titleMMS[] = "MMS",titleMME[] = "MME", titleMOD[] = "MOD", titleREC[] = "RECEIVED", titleIR[] = "IR", titlePER[] = "PER";

	  sprintf (title, "(%d - %d)", elem.first.first, elem.first.second);

	  dataset_SNR.SetTitle(std::string(titleSNR));
	  dataset_SINR.SetTitle(std::string(titleSINR));
	  dataset_IR.SetTitle(std::string(titleIR));
	  dataset_PER.SetTitle(std::string(titlePER));
	  dataset_MMS.SetTitle(std::string(titleMMS));
	  dataset_MME.SetTitle(std::string(titleMME));
	  dataset_MOD.SetTitle(std::string(titleMOD));
	  dataset_REC.SetTitle(std::string(titleREC));

	  dataset_SNR.SetStyle(Gnuplot2dDataset::STEPS);
	  dataset_SINR.SetStyle(Gnuplot2dDataset::STEPS);
	  dataset_IR.SetStyle(Gnuplot2dDataset::STEPS);
	  dataset_PER.SetStyle(Gnuplot2dDataset::LINES);
	  dataset_MMS.SetStyle(Gnuplot2dDataset::LINES);
	  dataset_MME.SetStyle(Gnuplot2dDataset::LINES);
	  dataset_MOD.SetStyle(Gnuplot2dDataset::STEPS);
	  dataset_REC.SetStyle(Gnuplot2dDataset::POINTS);


	  int id = 0;
	  for (auto elemq : elem.second) {
		  dataset_SNR.Add(id, elemq.SNR);
		  dataset_SINR.Add(id, elemq.SINR);
		  dataset_IR.Add(id, elemq.SINR/elemq.SNR);
		  dataset_PER.Add(id, elemq.PER);
		  dataset_MMS.Add(id, elemq.MMS);
		  dataset_MME.Add(id, elemq.MME);
		  dataset_MOD.Add(id, elemq.MOD);
		  //dataset_REC.Add(id, RECEIVED/(RECEIVED+NON_RECEIVED));
		  if (elemq.RECEIVED) {
			  dataset_REC.Add(id, 1);
		  } else {
			  //dataset_REC.Add(id, elemq.SINR/elemq.SNR);
			  dataset_REC.Add(id, elemq.SINR/elemq.SNR);
		  }
		  id++;

		  //histogram:
		  std::string ir_s = std::to_string( std::abs (elemq.SINR/elemq.SNR));
		  std::string sinr_s = std::to_string( std::abs (elemq.SINR));

		  std::map <std::string, std::pair< int, int > >::iterator it_ir = histogramQualityIR.find(ir_s);
		  std::map <std::string, std::pair< int, int > >::iterator it_sinr = histogramQualitySINR.find(sinr_s);
			if (elemq.RECEIVED) {
				if (it_ir != histogramQualityIR.end())
					histogramQualityIR[ir_s].first++;
				else {
					histogramQualityIR.insert(std::pair<std::string, std::pair<int, int> >(ir_s,std::pair<int, int>(1, 0)));
				}

				if (it_sinr != histogramQualitySINR.end())
					histogramQualitySINR[sinr_s].first++;
				else {
					histogramQualitySINR.insert(std::pair<std::string, std::pair<int, int> >(sinr_s,std::pair<int, int>(1, 0)));
				}

			} else {
				if (it_ir != histogramQualityIR.end())
					histogramQualityIR[ir_s].second++;
				else {
					histogramQualityIR.insert(std::pair<std::string, std::pair<int, int> >(ir_s,std::pair<int, int>(0, 1)));
				}

				if (it_sinr != histogramQualitySINR.end())
					histogramQualitySINR[sinr_s].second++;
				else {
					histogramQualitySINR.insert(std::pair<std::string, std::pair<int, int> >(sinr_s, std::pair<int, int>(0, 1)));
				}
			}
	  }

/*
	  int i = 1;
	  int window = 10;
	  double SNR=0, IR=0, PER=0, RECEIVED=0, NON_RECEIVED=0;
	  for (auto elemq : elem.second) {
		  if (i++ % window == 0) {
			  int id = i/window;
			  dataset_SNR.Add(id, SNR/window);
			  dataset_IR.Add(id, IR/window);
			  dataset_PER.Add(id, PER/window);
			  dataset_MMS.Add(id, elemq.MMS/window);
			  dataset_MME.Add(id, elemq.MME/window);
			  dataset_MOD.Add(id, elemq.MOD/window);
			  dataset_REC.Add(id, RECEIVED/(RECEIVED+NON_RECEIVED));
			  SNR=0; IR=0; RECEIVED=0; NON_RECEIVED=0; PER = 0;
		  } else {
			  if (elemq.RECEIVED)
				  RECEIVED ++;
			  else
				  NON_RECEIVED ++;
			  SNR += elemq.SNR;
			  IR += elemq.SINR/elemq.SNR;
			  PER += elemq.PER;
		  }
	  }
*/
	  // Add the dataset to the plot.
	  //plot.AddDataset (dataset_SINR);
	  //plot.AddDataset (dataset_SNR);
	  plot.AddDataset (dataset_IR);
	  //plot.AddDataset (dataset_PER);
	  //plot.AddDataset (dataset_MMS);
	  //plot.AddDataset (dataset_MOD);
	  plot.AddDataset (dataset_MME);
	  plot.AddDataset (dataset_REC);

	  // Write the plot file.
	  plot.SetTitle(title);
	  if (len++ == matrixQuality.size()) {
		  plotFile << "\nset key outside bottom center maxrows 1 \n";
	  }
	  plot.GenerateOutput (plotFile, true);
	  plot.ClearAllDataset();
	}
	// Close the plot file.
	plotFile.close ();

	char dataFileName[50] = "plot/histo_ir.dat";
	std::ofstream dataFile (dataFileName, std::ofstream::app);
	for(std::map<std::string, std::pair<int, int> >::const_iterator it = histogramQualityIR.begin(); it != histogramQualityIR.end(); ++it) {
		dataFile << it->first << " " << it->second.first << " " << it->second.second << "\n";
	}
	dataFile.close();

	char dataFileName_sinr[50] = "plot/histo_sinr.dat";
	std::ofstream dataFile_sinr (dataFileName_sinr, std::ofstream::app);
	for(std::map<std::string, std::pair<int, int> >::const_iterator it = histogramQualitySINR.begin(); it != histogramQualitySINR.end(); ++it) {
		dataFile_sinr << it->first << " " << it->second.first << " " << it->second.second << "\n";
	}
	dataFile_sinr.close();
}

