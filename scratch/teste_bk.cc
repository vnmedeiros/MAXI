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


struct MatrixQuality {
	double MMS;
	double MME;
	double SNR;
	double SINR;
	double MOD;
	double PER;
	bool RECEIVED;
};

algorithm::LABORARouting * routing;
element::Graph 			 * graph;
std::map <
	std::pair<int,int>,
	std::vector<struct MatrixQuality>
> matrixQuality;
int * map;

class ReportResults {
public:
	int m_lostPackets;		//Total packets lost
	int m_rxPackets;		//Total packets received
	double m_throughput;	//Average throughput
	double m_delayAvg;		//Average end-to-end time
	int m_MacTxDrop; 		//A packet has been dropped in the MAC layer before being queued for transmission.
	int m_MacRxDrop; 		//A packet has been dropped in the MAC layer after it has been passed up from the physical layer.
	int m_PhyTxDrop; 		//Trace source indicating a packet has been dropped by the device during transmission.
	int m_PhyRxDrop; 		//Trace source indicating a packet has been dropped by the device during reception.
	int m_ArpDrop;			//Trace source indicating a packet has been dropped by ARP protocol.
	unsigned int m_QuantityHops; //Trace source indicating a quantity hops for packet received.

	int recalculates;
	int qtdLinkUpTrue;
	int qtdLinkUpFalse;
	ReportResults():m_lostPackets(0), m_rxPackets(0),
					   m_throughput(0),	m_delayAvg(0.0),
					   m_MacTxDrop(0), m_MacRxDrop(0),
					   m_PhyTxDrop(0), m_PhyRxDrop(0),
					   m_ArpDrop(0), m_QuantityHops(0),
					   recalculates(0), qtdLinkUpTrue(0),
					   qtdLinkUpFalse(0){}
} reportResultsInstance;

void ArpDrop(Ptr<const Packet> p) {reportResultsInstance.m_ArpDrop++;}
void MacTxDrop(Ptr<const Packet> p) {reportResultsInstance.m_MacTxDrop++;}
void MacRxDrop(Ptr<const Packet> p) {reportResultsInstance.m_MacRxDrop++;}
void PhyTxDrop(Ptr<const Packet> p) {reportResultsInstance.m_PhyTxDrop++;}

std::map <std::pair<int,int>, int> cDropRx;
void PhyRxDrop(Ptr<const Packet> p) {
	//reportResultsInstance.m_PhyRxDrop++;
	//return;
	InfoTag infoTag;
	if ( p->PeekPacketTag(infoTag) ) {
		if(TypeTag::TAG_DATA == infoTag.GetTypeTag() ) {
			if (graph->existEdge(infoTag.GetPreviousNodeID(), infoTag.GetNodeID())) {
				reportResultsInstance.m_PhyRxDrop++;
				cDropRx[std::make_pair(infoTag.GetPreviousNodeID(),infoTag.GetNodeID())]++;
				return;

				algorithm::RALL * b = dynamic_cast<algorithm::RALL *>(routing);
				if (b == NULL) {
					reportResultsInstance.m_PhyRxDrop++;
					return;
				}
				path * pt = b->getRoute(element::Flow(infoTag.GetSrc(), 0, 0, 0));
				if(pt) {
					for (unsigned int i = 0; i < pt->size()-1; i++) {
						if ( (unsigned int)(*pt)[i] == infoTag.GetPreviousNodeID() && (unsigned int)(*pt)[i+1] == infoTag.GetNodeID() ) {
							//std::cout << infoTag.GetSrc() << " id=" << infoTag.GetId()  <<":\n";
							//((algorithm::RALL * )routing)->printExistingRoutes();
							cDropRx[std::make_pair(infoTag.GetPreviousNodeID(),infoTag.GetNodeID())]++;
							reportResultsInstance.m_PhyRxDrop++;
							break;
						}
					}
				}
			}
		}
	}
}

static void UpdateQualityLink (int, int, double, double, double, bool);
static int* setChannels(algorithm::CRAA *, NodeContainer, uint32_t);
static void PRELOAD(int, int, int, double, NodeContainer);
static void PRELOAD_ReceiveTraffic(Ptr<Socket>);
static void PRELOAD_GenerateTraffic(Ptr<Socket>, uint32_t, uint32_t, Time);
static void FLOWS_Generate(int, int, NodeContainer, Ipv4InterfaceContainer, double, double);
//static void SETUP_CONFIG(NodeContainer wifiNodes, Ipv4JointRoutingHelper ipv4RoutingHelper, Ipv4InterfaceContainer wifiNodeInterface);
static void QualityVariationPlotFile (std::map< std::pair<int,int>, std::vector<struct MatrixQuality> >, char * );

unsigned int countUpdates = 0;
static void callRecalculateRoutes() {
	countUpdates++;
	if (countUpdates == 10) {
		reportResultsInstance.recalculates++;
		algorithm::RALL * rall = (algorithm::RALL *)routing;
		rall->recreatesAllRoutes();
		countUpdates = 0;
	}
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
		reportResultsInstance.m_QuantityHops += jointRouteHeader.GetHops().size();
	}
}

void print(int N_NODES) {
	std::cout << "PRINT:" << Simulator::Now() << std::endl;

	for (int i = 0; i < N_NODES; i++) {
		//std::cout << "V:" << i << " -> ";
		for (int j = 0; j < N_NODES; j++) {
			if (map[i * N_NODES + j] >= 50 && map[j * N_NODES + i] >= 50 && i != j) {
				std::cout << "map[" << i << " * 20 + " << j << "] = " << map[i * N_NODES + j] << ";";
			}
		}
		std::cout << std::endl;
	}
	std::cout << std::endl << std::endl;
	graph->showAdjacencyList();
}


double 		simulationTime = 5000; //seconds
uint32_t 	channelWidth = 0;
std::string wifiType = "ns3::SpectrumWifiPhy";
std::string errorModelType = "ns3::NistErrorRateModel";
bool 		enablePcap = false;

int N_NODES 	= 20;	//Number of Nodes
int N_RADIOS 	= 3; 	//Number of Radios
int N_CHANNELS 	= 3;	//Number of Channels

//Parameters to determine area:
int QUADRANT_DIVISION	= 3; 	//Quadrant division
int QUADRANT_I 			= 2; 	//Quadrant position
int QUADRANT_J 			= 2;	//Quadrant position

/*double SIZE_AREA 		= 800; 	//Size area
double MINIMUM_DISTANCE = 80.0; //Minimum distance
double MAXIMUM_DISTANCE = 130.0;//Maximum distance*/

double SIZE_AREA 		= 800; 	//Size area
double MINIMUM_DISTANCE = 80.0; //Minimum distance
double MAXIMUM_DISTANCE = 130.0;//200//Maximum distance

char algorithm_run[20] = "RALL";	//Algorithm strings
//char algorithm[20] = "BPR";	//Algorithm strings
int seed = 1;
int main(int argc, char *argv[]) {

	CommandLine cmd;
	cmd.AddValue("simulationTime", 	"Simulation time in seconds", 						simulationTime);
	cmd.AddValue("wifiType", 		"select ns3::SpectrumWifiPhy or ns3::YansWifiPhy", 	wifiType);
	cmd.AddValue("errorModelType", 	"select ns3::NistErrorRateModel or ns3::YansErrorRateModel", errorModelType);
	cmd.AddValue("enablePcap", 		"enable pcap output", 								enablePcap);
	cmd.AddValue ("algorithm", 		"Algorithm", 	algorithm_run);
	cmd.AddValue ("N_NODES", 		"N_NODES", 		N_NODES);
	cmd.AddValue ("seed", 			"Seed", 		seed);
	cmd.AddValue ("N_RADIOS", 		"N_RADIOS", 	N_RADIOS);
	cmd.Parse(argc, argv);


	//Set simulator seed:
	ns3::SeedManager::SetSeed(seed);

	map = (int *) malloc ( sizeof(int *) * (N_NODES*N_NODES));
	for (int i = 0; i<N_NODES*N_NODES; i++)
		map[i] = 0;

	std::cout << "# wifiType: " << wifiType
			  << " distance min: " << MINIMUM_DISTANCE << "m distance max: " << MAXIMUM_DISTANCE << "m"
			  << " seed: " << seed << std::endl;

	//the node 0 is the sink
	NodeContainer wifiNodes;
	wifiNodes.Create(N_NODES);

	YansWifiPhyHelper phy = YansWifiPhyHelper::Default();
	SpectrumWifiPhyCustomHelper spectrumPhy = SpectrumWifiPhyCustomHelper::Default();
	if (wifiType == "ns3::YansWifiPhy") {
		YansWifiChannelHelper channel;
		channel.AddPropagationLoss("ns3::FriisPropagationLossModel");
		channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
		phy.SetChannel(channel.Create());

		phy.Set("ShortGuardEnabled", BooleanValue(false));
		channelWidth = 20;
		Config::SetDefault("ns3::WifiPhy::EnergyDetectionThreshold", DoubleValue (-79.0)); 	//The energy of a received signal should be higher than this threshold (dbm) to allow the PHY layer to detect the signal.
		Config::SetDefault("ns3::WifiPhy::CcaMode1Threshold", DoubleValue (-62.0) ); 		//The energy of a received signal should be higher than this threshold (dbm) to allow the PHY layer to declare CCA BUSY state.
		Config::SetDefault("ns3::WifiPhy::TxGain", DoubleValue (0) ); 					//Transmission gain (dB).
		Config::SetDefault("ns3::WifiPhy::RxGain", DoubleValue (0) ); 					//Reception gain (dB).
		Config::SetDefault("ns3::WifiPhy::TxPowerLevels", UintegerValue (1) ); 			//Number of transmission power levels available between TxPowerStart and TxPowerEnd included.
		Config::SetDefault("ns3::WifiPhy::TxPowerEnd", DoubleValue (22.0) );  			//Maximum available transmission level (dbm).
		Config::SetDefault("ns3::WifiPhy::TxPowerStart", DoubleValue (22.0) ); 			//Minimum available transmission level (dbm).

	} else if (wifiType == "ns3::SpectrumWifiPhy") {
		Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
		Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
		spectrumChannel->AddPropagationLossModel(lossModel);

		Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
		spectrumChannel->SetPropagationDelayModel(delayModel);

		spectrumPhy.SetChannel(spectrumChannel);
		spectrumPhy.SetErrorRateModel(errorModelType);
		//spectrumPhy.Set("Frequency", UintegerValue(5180)); // channel 36 at 20 MHz
		spectrumPhy.Set("Frequency", UintegerValue(2412)); // channel 1 at 20 MHz

		spectrumPhy.Set("ShortGuardEnabled", BooleanValue(false));
		channelWidth = 20;
		Config::SetDefault("ns3::WifiPhy::EnergyDetectionThreshold", DoubleValue (-79.0)); 	//The energy of a received signal should be higher than this threshold (dbm) to allow the PHY layer to detect the signal.
		Config::SetDefault("ns3::WifiPhy::CcaMode1Threshold", DoubleValue (-62.0) ); 		//The energy of a received signal should be higher than this threshold (dbm) to allow the PHY layer to declare CCA BUSY state.
		Config::SetDefault("ns3::WifiPhy::TxGain", DoubleValue (0) ); 					//Transmission gain (dB).
		Config::SetDefault("ns3::WifiPhy::RxGain", DoubleValue (0) ); 					//Reception gain (dB).
		Config::SetDefault("ns3::WifiPhy::TxPowerLevels", UintegerValue (1) ); 			//Number of transmission power levels available between TxPowerStart and TxPowerEnd included.
		Config::SetDefault("ns3::WifiPhy::TxPowerEnd", DoubleValue (16.0) );  			//Maximum available transmission level (dbm).
		Config::SetDefault("ns3::WifiPhy::TxPowerStart", DoubleValue (16.0) ); 			//Minimum available transmission level (dbm).
	} else {
		NS_FATAL_ERROR("Unsupported WiFi type " << wifiType);
	}


	WifiHelper wifi;
	wifi.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);
	//wifi.SetStandard(WIFI_PHY_STANDARD_80211n_2_4GHZ);
	Ssid ssid = Ssid("NS3-80211n");

	StringValue DataRate;
	DataRate = StringValue("HtMcs1");

	wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
			"DataMode", 		DataRate, 				//
			"ControlMode", 		DataRate, 				//
			"NonUnicastMode", 	DataRate, 				//Wifi mode used for non-unicast transmissions.
			"MaxSsrc", 			UintegerValue(1), 		//The maximum number of retransmission attempts for an RTS.
			"MaxSlrc", 			UintegerValue(1), 		//The maximum number of retransmission attempts for a DATA packet.
			"RtsCtsThreshold", 	UintegerValue(10000), 	//If the size of the data packet + LLC header + MAC header + FCS trailer is bigger than this value, we use an RTS/CTS handshake before sending the data.
			"FragmentationThreshold", StringValue("10000"));//Disable fragmentation for frames below 10000 bytes.


	//Create WifiMac for each wifi node:
	NqosWifiMacHelper mac;
	mac = NqosWifiMacHelper::Default();
	mac.SetType ("ns3::AdhocWifiMac", "Ssid", SsidValue(ssid));


	//install radios in nodes:
	NetDeviceContainer wifiDevice[N_RADIOS];
	for (int i = 0; i < N_RADIOS; i++) {
		if (wifiType == "ns3::YansWifiPhy")
			wifiDevice[i] = wifi.Install(phy, mac, wifiNodes);
		else if (wifiType == "ns3::SpectrumWifiPhy")
			wifiDevice[i] = wifi.Install(spectrumPhy, mac, wifiNodes);
	}
	// Channel width must be set *after* installation because the attribute
	// is overwritten by the ConfigureStandard method ()
	Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue(channelWidth));

	//mobility.
	Ptr<RandomRectanglePositionAllocatorMinMax> pa = new RandomRectanglePositionAllocatorMinMax(SIZE_AREA, QUADRANT_DIVISION);
	pa->SetQuadrants(QUADRANT_I, QUADRANT_J);
	MyMobilityHelper mobility;
	mobility.SetPositionAllocator(pa);
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.SetNNodes(N_NODES);
	mobility.SetMinDist(MINIMUM_DISTANCE);
	mobility.SetMaxDist(MAXIMUM_DISTANCE);
	mobility.Install(wifiNodes);
	if (!pa->verifyAllocation(mobility, N_NODES, MINIMUM_DISTANCE, MAXIMUM_DISTANCE)) {
		std::cout << "verify Allocation not correct!";
		exit(1);
	}

	//Create stack protocols:
	InternetStackHelper stack;
	Ipv4JointRoutingHelper ipv4RoutingHelper;
	Ipv4ListRoutingHelper list;
	list.Add(ipv4RoutingHelper, 0);
	stack.SetRoutingHelper(list);
	stack.Install(wifiNodes);

	//address:
	Ipv4InterfaceContainer wifiNodeInterface;
	Ipv4AddressHelper address;
	address.SetBase("10.1.1.0", "255.255.255.0");
	for (int i = 0; i < N_RADIOS; i++) {
		if (i == 0) {
			wifiNodeInterface = address.Assign(wifiDevice[i]);
		} else {
			address.Assign(wifiDevice[i]);
		}
	}

	/*map[0 * 20 + 1] = 300;map[0 * 20 + 2] = 300;map[0 * 20 + 3] = 300;map[0 * 20 + 4] = 300;map[0 * 20 + 5] = 300;map[0 * 20 + 6] = 300;map[0 * 20 + 7] = 300;map[0 * 20 + 8] = 300;map[0 * 20 + 11] = 300;map[0 * 20 + 13] = 300;map[0 * 20 + 17] = 300;
	map[1 * 20 + 0] = 445;map[1 * 20 + 2] = 6;map[1 * 20 + 5] = 6;map[1 * 20 + 6] = 303;map[1 * 20 + 7] = 6;map[1 * 20 + 8] = 160;map[1 * 20 + 12] = 302;map[1 * 20 + 13] = 303;map[1 * 20 + 17] = 303;
	map[2 * 20 + 0] = 300;map[2 * 20 + 1] = 300;map[2 * 20 + 3] = 300;map[2 * 20 + 4] = 301;map[2 * 20 + 5] = 300;map[2 * 20 + 7] = 300;map[2 * 20 + 8] = 300;map[2 * 20 + 10] = 300;map[2 * 20 + 15] = 300;
	map[3 * 20 + 0] = 254;map[3 * 20 + 2] = 289;map[3 * 20 + 4] = 163;map[3 * 20 + 5] = 153;map[3 * 20 + 8] = 300;map[3 * 20 + 10] = 153;map[3 * 20 + 11] = 300;
	map[4 * 20 + 0] = 450;map[4 * 20 + 2] = 450;map[4 * 20 + 3] = 450;map[4 * 20 + 5] = 450;map[4 * 20 + 8] = 450;map[4 * 20 + 10] = 450;map[4 * 20 + 15] = 450;
	map[5 * 20 + 0] = 300;map[5 * 20 + 1] = 300;map[5 * 20 + 2] = 450;map[5 * 20 + 3] = 300;map[5 * 20 + 4] = 409;map[5 * 20 + 7] = 450;map[5 * 20 + 9] = 450;map[5 * 20 + 10] = 448;map[5 * 20 + 14] = 448;map[5 * 20 + 15] = 450;
	map[6 * 20 + 0] = 306;map[6 * 20 + 1] = 306;map[6 * 20 + 8] = 306;map[6 * 20 + 11] = 306;map[6 * 20 + 12] = 306;map[6 * 20 + 13] = 306;map[6 * 20 + 17] = 306;
	map[7 * 20 + 0] = 150;map[7 * 20 + 1] = 150;map[7 * 20 + 2] = 150;map[7 * 20 + 5] = 190;map[7 * 20 + 9] = 272;map[7 * 20 + 14] = 159;map[7 * 20 + 15] = 159;
	map[8 * 20 + 0] = 300;map[8 * 20 + 1] = 300;map[8 * 20 + 2] = 300;map[8 * 20 + 3] = 300;map[8 * 20 + 4] = 300;map[8 * 20 + 6] = 300;map[8 * 20 + 11] = 300;map[8 * 20 + 12] = 300;map[8 * 20 + 18] = 300;
	map[9 * 20 + 5] = 156;map[9 * 20 + 7] = 156;map[9 * 20 + 14] = 306;map[9 * 20 + 15] = 156;map[9 * 20 + 16] = 303;
	map[10 * 20 + 2] = 303;map[10 * 20 + 3] = 303;map[10 * 20 + 4] = 450;map[10 * 20 + 5] = 303;map[10 * 20 + 14] = 303;map[10 * 20 + 15] = 303;map[10 * 20 + 16] = 303;
	map[11 * 20 + 0] = 300;map[11 * 20 + 3] = 300;map[11 * 20 + 6] = 300;map[11 * 20 + 8] = 340;map[11 * 20 + 12] = 300;map[11 * 20 + 18] = 450;map[11 * 20 + 19] = 306;
	map[12 * 20 + 1] = 15;map[12 * 20 + 6] = 159;map[12 * 20 + 8] = 15;map[12 * 20 + 11] = 15;map[12 * 20 + 13] = 159;map[12 * 20 + 17] = 159;
	map[13 * 20 + 0] = 150;map[13 * 20 + 1] = 300;map[13 * 20 + 6] = 447;map[13 * 20 + 12] = 450;map[13 * 20 + 17] = 450;
	map[14 * 20 + 5] = 303;map[14 * 20 + 7] = 303;map[14 * 20 + 9] = 450;map[14 * 20 + 10] = 303;map[14 * 20 + 15] = 338;map[14 * 20 + 16] = 450;
	map[15 * 20 + 2] = 450;map[15 * 20 + 4] = 450;map[15 * 20 + 5] = 450;map[15 * 20 + 7] = 450;map[15 * 20 + 9] = 450;map[15 * 20 + 10] = 450;map[15 * 20 + 14] = 450;map[15 * 20 + 16] = 450;
	map[16 * 20 + 9] = 303;map[16 * 20 + 10] = 156;map[16 * 20 + 14] = 450;map[16 * 20 + 15] = 303;
	map[17 * 20 + 0] = 300;map[17 * 20 + 1] = 300;map[17 * 20 + 6] = 300;map[17 * 20 + 12] = 442;map[17 * 20 + 13] = 450;
	map[18 * 20 + 8] = 6;map[18 * 20 + 11] = 152;map[18 * 20 + 19] = 300;
	map[19 * 20 + 11] = 450;map[19 * 20 + 18] = 450;
	*/
	map[0 * 20 + 1] = 300;map[0 * 20 + 2] = 300;map[0 * 20 + 3] = 300;map[0 * 20 + 4] = 300;map[0 * 20 + 5] = 300;map[0 * 20 + 6] = 300;map[0 * 20 + 7] = 300;map[0 * 20 + 8] = 300;map[0 * 20 + 11] = 300;map[0 * 20 + 13] = 300;map[0 * 20 + 17] = 300;
	map[1 * 20 + 0] = 445;map[1 * 20 + 6] = 303;map[1 * 20 + 8] = 160;map[1 * 20 + 13] = 303;map[1 * 20 + 17] = 303;
	map[2 * 20 + 0] = 300;map[2 * 20 + 3] = 300;map[2 * 20 + 4] = 301;map[2 * 20 + 5] = 300;map[2 * 20 + 7] = 300;map[2 * 20 + 8] = 300;map[2 * 20 + 10] = 300;map[2 * 20 + 15] = 300;
	map[3 * 20 + 0] = 254;map[3 * 20 + 2] = 289;map[3 * 20 + 4] = 163;map[3 * 20 + 5] = 153;map[3 * 20 + 8] = 300;map[3 * 20 + 10] = 153;map[3 * 20 + 11] = 300;
	map[4 * 20 + 0] = 450;map[4 * 20 + 2] = 450;map[4 * 20 + 3] = 450;map[4 * 20 + 5] = 450;map[4 * 20 + 8] = 450;map[4 * 20 + 10] = 450;map[4 * 20 + 15] = 450;
	map[5 * 20 + 0] = 300;map[5 * 20 + 2] = 450;map[5 * 20 + 3] = 300;map[5 * 20 + 4] = 409;map[5 * 20 + 7] = 450;map[5 * 20 + 9] = 450;map[5 * 20 + 10] = 448;map[5 * 20 + 14] = 448;map[5 * 20 + 15] = 450;
	map[6 * 20 + 0] = 306;map[6 * 20 + 1] = 306;map[6 * 20 + 8] = 306;map[6 * 20 + 11] = 306;map[6 * 20 + 12] = 306;map[6 * 20 + 13] = 306;map[6 * 20 + 17] = 306;
	map[7 * 20 + 0] = 150;map[7 * 20 + 2] = 150;map[7 * 20 + 5] = 191;map[7 * 20 + 9] = 273;map[7 * 20 + 14] = 159;map[7 * 20 + 15] = 159;
	map[8 * 20 + 0] = 300;map[8 * 20 + 1] = 300;map[8 * 20 + 2] = 300;map[8 * 20 + 3] = 300;map[8 * 20 + 4] = 300;map[8 * 20 + 6] = 300;map[8 * 20 + 11] = 300;
	map[9 * 20 + 5] = 156;map[9 * 20 + 7] = 156;map[9 * 20 + 14] = 306;map[9 * 20 + 15] = 156;map[9 * 20 + 16] = 303;
	map[10 * 20 + 2] = 303;map[10 * 20 + 3] = 303;map[10 * 20 + 4] = 450;map[10 * 20 + 5] = 303;map[10 * 20 + 14] = 303;map[10 * 20 + 15] = 303;map[10 * 20 + 16] = 303;
	map[11 * 20 + 0] = 300;map[11 * 20 + 3] = 300;map[11 * 20 + 6] = 300;map[11 * 20 + 8] = 340;map[11 * 20 + 18] = 450;map[11 * 20 + 19] = 306;
	map[12 * 20 + 6] = 159;map[12 * 20 + 13] = 159;map[12 * 20 + 17] = 159;
	map[13 * 20 + 0] = 150;map[13 * 20 + 1] = 300;map[13 * 20 + 6] = 447;map[13 * 20 + 12] = 450;map[13 * 20 + 17] = 450;
	map[14 * 20 + 5] = 303;map[14 * 20 + 7] = 303;map[14 * 20 + 9] = 450;map[14 * 20 + 10] = 303;map[14 * 20 + 15] = 338;map[14 * 20 + 16] = 450;
	map[15 * 20 + 2] = 450;map[15 * 20 + 4] = 450;map[15 * 20 + 5] = 450;map[15 * 20 + 7] = 450;map[15 * 20 + 9] = 450;map[15 * 20 + 10] = 450;map[15 * 20 + 14] = 450;map[15 * 20 + 16] = 450;
	map[16 * 20 + 9] = 303;map[16 * 20 + 10] = 156;map[16 * 20 + 14] = 450;map[16 * 20 + 15] = 303;
	map[17 * 20 + 0] = 300;map[17 * 20 + 1] = 300;map[17 * 20 + 6] = 300;map[17 * 20 + 12] = 442;map[17 * 20 + 13] = 450;
	map[18 * 20 + 11] = 152;map[18 * 20 + 19] = 300;
	map[19 * 20 + 11] = 450;map[19 * 20 + 18] = 450;


	graph = new element::Graph(map, N_NODES);
	//graph = new element::Graph(mobility.GetMatrix(), N_NODES);
	for (int i = 0; i<N_NODES*N_NODES; i++)
		map[i] = 0;
	//memset(map, 0, (N_NODES*N_NODES));
	//graph->showAdjacencyList();


	//Create Routing:
	if (strcmp (algorithm_run, "BPR") == 0) {
		algorithm::BPR * bpr = new algorithm::BPR(N_NODES, 2.5, graph);
		bpr->CalculateCandidatesPaths();
		routing = dynamic_cast<algorithm::LABORARouting * >(bpr);
	} else if (strcmp (algorithm_run, "RALL") == 0) {
		routing = new algorithm::RALL(0.5, 0.5, graph);
		graph->m_callbackUpdateLink = &callRecalculateRoutes;
	} else {
		std::cout << "Routing algorithm	not correct!";
		exit(1);
	}

	//Create CRAA:
	int myChannels[]= {1,6,11,2,7,12,3,8,13,4,9,5,10};
	//int myChannels[]= {36,60,112,132,40,64,116,136,44,100,120,140,48,104,124,144,52,108,128};
	std::set<CRAAChannel> channels(myChannels, myChannels+N_CHANNELS);

	std::map< element::Edge, std::map< element::Edge, double> > per;
	algorithm::CRAA * craa = new algorithm::CRAA(routing, graph->U.size(), 0.0, channels, N_RADIOS, 2, per, 1);
	craa->RunInitial();

	//Set initial channel:
	int *channelsToRadios = setChannels(craa, wifiNodes, N_NODES);
	craa->SetChannelsToRadios(channelsToRadios);

	//Create map between address and channel:
	std::map< std::pair<int, int>, Ipv4Address > nodesAddress;
	for (int i = 0; i < N_NODES; i++) {
		Ptr<ns3::Node> node = wifiNodes.Get(i);
		for (int j = 0; j < N_RADIOS; j++) {
			Ptr<WifiPhy> nodeWifiPhy = node->GetDevice(j)->GetObject<WifiNetDevice>()->GetPhy()->GetObject<WifiPhy>();
			int channel = nodeWifiPhy->GetChannelNumber();
			std::pair<int, int> ic =  std::make_pair (i,channel);
			nodesAddress[ic] = node->GetObject<Ipv4>()->GetAddress(j+1,0).GetLocal();
		}
	}

	//Set Ipv4JointRouting:
	int nodeId = 0;
	for (int i = 0; i < N_NODES; i++, nodeId++) {
		Ptr<JointRouting> jointRouting = ipv4RoutingHelper.GetJointRouting(wifiNodes.Get(i)->GetObject<Ipv4>());
		jointRouting->setLRouting(routing);
		jointRouting->setRcaa(craa);
		jointRouting->setNodesAddress(nodesAddress);
		jointRouting->setNodeId(nodeId);
		//jointRouting->m_callbackUpdateLinkQuality = MakeCallback(&UpdateQualityLink);
	}

	//Setting Applications
	PRELOAD(50, N_NODES, N_RADIOS, 0.5, wifiNodes);
	FLOWS_Generate(0, N_NODES, wifiNodes, wifiNodeInterface, simulationTime, N_NODES*50 + 100);
	Simulator::Schedule(Seconds(N_NODES*50 + 100), &print, N_NODES);


	//Traces
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop", MakeCallback(&MacTxDrop));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxDrop", MakeCallback(&MacRxDrop));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&PhyRxDrop));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&PhyTxDrop));
	Config::ConnectWithoutContext("/NodeList/*/$ns3::ArpL3Protocol/Drop", MakeCallback(&ArpDrop));
	Config::ConnectWithoutContext("/NodeList/*/$ns3::Ipv4L3Protocol/LocalDeliver", MakeCallback(&ReceiveFlows)); // \FAZER: Endenter a forma como os pacotes são passados para camada de cima, a aplicação do SINk não tá recebendo os pacotes.
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/UpdateLinkQuality", MakeCallback(&UpdateQualityLink));
	//Config::ConnectWithoutContext("/NodeList/*/$ns3::Ipv4L3Protocol/LocalDeliver", MakeCallback(&ReceiveFlows)); // \FAZER: Endenter a forma como os pacotes são passados para camada de cima, a aplicação do SINk não tá recebendo os pacotes.

	//Install FlowMonitor on all nodes
	FlowMonitorHelper flowmon;
	Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
	monitor->SetAttribute("DelayBinWidth", DoubleValue (0.100));
	monitor->SerializeToXmlFile("FlowMonitor.xml", true, true);

	//Install Animation
	AnimationInterface anim ("animationTESTE.xml"); // Mandatory
	//anim.EnableIpv4RouteTracking ("routingtable-wireless-RALL.xml", Seconds (0), Seconds (5), Seconds (0.25)); //Optional
	anim.EnablePacketMetadata (); // Optional


	std::cout << "Start...\n";
	Simulator::Stop(Seconds(simulationTime + 1));
	Simulator::Run();

	//Print per flow statistics
	int count = 0;
	Histogram delayHistogram(0.100);
	monitor->CheckForLostPackets();
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
	FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i) {
		/*
		Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
		std::cout << "\nFlow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << "["<< t.sourcePort << "," << t.destinationPort <<"])\n";
		std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
		std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
		std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024 << " Mbps\n";
		std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
		std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
		std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024 << " Mbps\n";
		std::cout << "  lost Packets: " << i->second.lostPackets << "\n";
		*/
		if(i->second.rxPackets > 0) {
			reportResultsInstance.m_delayAvg += i->second.delaySum.GetMilliSeconds() / i->second.rxPackets;
			count++;
		}
		reportResultsInstance.m_lostPackets += i->second.lostPackets;
		reportResultsInstance.m_rxPackets += i->second.rxPackets;
		reportResultsInstance.m_throughput += (i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024);

		Histogram dh = i->second.delayHistogram;
		for (unsigned int t = 0; t < dh.GetNBins(); t++) {
			double value = delayHistogram.GetBinWidth(0) * dh.GetBinStart(t);
			for (unsigned int u = 0; u < dh.GetBinCount(t); u++)
				delayHistogram.AddValue(value);
		}
	}

	char dataFileName[50];
		sprintf(dataFileName,"plot/%s-%d-R%d.dat", algorithm_run ,N_NODES, N_RADIOS);
		struct stat st;
		int hd = stat(dataFileName, &st);
		std::ofstream dataFile (dataFileName, std::ofstream::app);

		if (hd == -1) {
			dataFile << "SEED;\t"
					 << "     RX Packets "
					 << "   Lost Packets "
					 << " % Lost Packets "
					 << "throughpu(Mbps) "
					 << " AVG Delay(ms)t "
					 << "       AVG HOPS "
					 << "      MacRxDrop "
					 << "      MacTxDrop "
					 << "      PhyRxDrop "
					 << "      PhyTxDrop "
					 << "        ArpDrop "
					 << "   recalculates "
					 << "  qtdLinkUpTrue "
					 << " qtdLinkUpFalse            ";
				for (unsigned int t = 0; t < delayHistogram.GetNBins(); t++) {
					dataFile << "[" << delayHistogram.GetBinStart(t) * 1000 << "-" <<  delayHistogram.GetBinEnd(t) * 1000 << "];";
				}
			dataFile << std::endl;
		}

		dataFile << seed << "\t"
				 << std::setfill (' ') << std::setw (15) << reportResultsInstance.m_rxPackets << " "
				 << std::setfill (' ') << std::setw (15) << reportResultsInstance.m_lostPackets << " "
				 << std::setfill (' ') << std::setw (15) << (double)reportResultsInstance.m_lostPackets/(reportResultsInstance.m_lostPackets+reportResultsInstance.m_rxPackets) << " "
				 << std::setfill (' ') << std::setw (15) << reportResultsInstance.m_throughput/(N_NODES -1) << " "
				 << std::setfill (' ') << std::setw (15) << reportResultsInstance.m_delayAvg/count << " "
				 << std::setfill (' ') << std::setw (15) << (double)reportResultsInstance.m_QuantityHops/reportResultsInstance.m_rxPackets << " "
				 << std::setfill (' ') << std::setw (15) << reportResultsInstance.m_MacRxDrop << " "
				 << std::setfill (' ') << std::setw (15) << reportResultsInstance.m_MacTxDrop << " "
				 << std::setfill (' ') << std::setw (15) << reportResultsInstance.m_PhyRxDrop << " "
				 << std::setfill (' ') << std::setw (15) << reportResultsInstance.m_PhyTxDrop << " "
				 << std::setfill (' ') << std::setw (15) << reportResultsInstance.m_ArpDrop << " "
				 << std::setfill (' ') << std::setw (15) << reportResultsInstance.recalculates << " "
				 << std::setfill (' ') << std::setw (15) << reportResultsInstance.qtdLinkUpTrue << " "
				 << std::setfill (' ') << std::setw (15) << reportResultsInstance.qtdLinkUpFalse << " "
				 << std::setfill (' ') << std::setw (15);

		for (unsigned int t = 0; t < delayHistogram.GetNBins(); t++) {
			dataFile << delayHistogram.GetBinCount(t) << ";";
		}
		dataFile << std::endl;
		dataFile.close();


	char prefixFileName[10];
	sprintf(prefixFileName,"%s-%d-seed:%d", algorithm_run ,N_NODES, seed);
	QualityVariationPlotFile(matrixQuality, prefixFileName);

	for (auto el : cDropRx) {
		std::cout << el.first.first << "-" << el.first.second << " : " << el.second << std::endl;
	}
	Simulator::Destroy();

	return 0;
}

/*
 *\\***************************************************\\
 *\\			Implementation functions:			   \\
 *\\***************************************************\\
 */

static void PRELOAD(int qtdPkts, int nNodes, int nRadios, double interval, NodeContainer wifiNodes) {
	Time interPacketInterval = Seconds(interval);
	Address BroadcastAddress (InetSocketAddress (Ipv4Address::GetBroadcast(), 80));
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);

	//Ptr<Socket> recvSink = Socket::CreateSocket(wifiNodes.Get(0), tid);
	//recvSink->Bind(local);
	//recvSink->SetRecvCallback(MakeCallback (&PRELOAD_ReceiveTraffic));

	std::vector<Ptr<Socket>> sendSockets;
	std::vector<Ptr<Socket>> recvSockets;
	int t=0;
	for (int i=0; i <= nNodes-1; i++) {
		for (int r=0; r < nRadios; r++, t++) {
			double time = (double)(rand() % 300)/200;
			sendSockets.push_back(Socket::CreateSocket(wifiNodes.Get(i), tid));
			recvSockets.push_back(Socket::CreateSocket(wifiNodes.Get(i), tid));
			sendSockets[t]->Bind();
			sendSockets[t]->BindToNetDevice(wifiNodes.Get(i)->GetDevice(r));
			sendSockets[t]->Connect(BroadcastAddress);
			sendSockets[t]->SetAllowBroadcast(true);
			sendSockets[t]->ShutdownRecv();

			recvSockets[t]->Bind(local);
			recvSockets[t]->ShutdownSend();
			recvSockets[t]->SetRecvCallback(MakeCallback (&PRELOAD_ReceiveTraffic));

			Simulator::ScheduleWithContext(sendSockets[t]->GetNode()->GetId(),
										   Seconds(time), &PRELOAD_GenerateTraffic,
										   //Seconds((i-1) * 50 + 10), &PRELOAD_GenerateTraffic,
										   sendSockets[t], 1024, qtdPkts, interPacketInterval);
		}
	}
}

static void PRELOAD_GenerateTraffic(Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval) {
	if (pktCount > 0) {
		Ptr<Packet> pkt = Create<Packet>(pktSize);
		InfoTag infoTag;
		infoTag.SetSrc(socket->GetNode()->GetId());
		infoTag.SetNodeId(socket->GetNode()->GetId());
		infoTag.SetId(pktCount);
		infoTag.SetTypeTag(TypeTag::TAG_PRELOAD);
		pkt->AddPacketTag(infoTag);
		socket->Send(pkt);
		Simulator::Schedule(pktInterval, &PRELOAD_GenerateTraffic, socket, pktSize, pktCount - 1, pktInterval);
	} else {
		socket->Close();
	}
}

static void PRELOAD_ReceiveTraffic(Ptr<Socket> socket) {
	Address addr;
	Ptr<Packet> packet;
	while ((packet = socket->Recv())) {
		socket->GetSockName(addr);
		InfoTag infoTag;
		if ( packet->PeekPacketTag(infoTag) ) {
			packet->RemovePacketTag(infoTag);
			//map[infoTag.GetSrc()][socket->GetNode()->GetId()]++;
			map[infoTag.GetSrc() * N_NODES + socket->GetNode()->GetId()]++;
		}
	}
}

static void FLOWS_Generate(int idSink, int N_NODES, NodeContainer wifiNodes, Ipv4InterfaceContainer wifiNodeInterface, double simulationTime, double startAfter) {
	//return;
	Address sinkAddress (InetSocketAddress (wifiNodeInterface.GetAddress(0), 80));

	PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 80));
	ApplicationContainer packetSink = packetSinkHelper.Install(wifiNodes.Get(0));
	packetSink.Start(Seconds(startAfter));
	packetSink.Stop(Seconds(simulationTime));


	LaboraAppHelper laboraAppHelper("ns3::UdpSocketFactory", sinkAddress, LaboraAppHelper::Class_3);
	laboraAppHelper.SetAttribute("MaxPackets", UintegerValue(256));
	//laboraAppHelper.SetAttribute("DataRate", StringValue("1Mbps"));
	laboraAppHelper.SetAttribute("DataRate", StringValue("512kb/s"));
	//laboraAppHelper.SetAttribute("DataRate", StringValue("320kbps"));
	laboraAppHelper.SetAttribute("PacketSize", UintegerValue(1024));
	laboraAppHelper.SetAttribute ("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
	laboraAppHelper.SetAttribute ("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

	ApplicationContainer source;
	for (int i = 1; i < N_NODES; i++) {
		source.Add(laboraAppHelper.Install(wifiNodes.Get(i)));

	}
	source.Add(laboraAppHelper.Install(wifiNodes.Get(11)));

	source.Start (Seconds (startAfter+0.1));
	source.Stop (Seconds (simulationTime));
}

static void UpdateQualityLink (int sourceVertice, int destinationVertice, double snr, double sinr, double per, bool received) {
	bool updated = graph->qualityLinkUpdate(sourceVertice, destinationVertice, sinr); //SINR
	//bool updated = graph->qualityLinkUpdate(sourceVertice, destinationVertice, 1-per); //PER
	element::Edge item(sourceVertice, destinationVertice, 0);
	std::vector<element::Edge>::iterator ed = std::find(graph->listEdges[sourceVertice].begin(), graph->listEdges[sourceVertice].end(), item);
	struct MatrixQuality mq;

	if ( ed != graph->listEdges[sourceVertice].end()) {
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
	//return 0;
	//setting an unused channel to all radios
	for (uint32_t i = 0; i < nNodes; i++) {
		Ptr<ns3::Node> node = wifiNodes.Get(i);
		for (int j = 0; j < craa->m_nRadios; j++) {
			Ptr<WifiNetDevice> netDevice = node->GetDevice(j)->GetObject<WifiNetDevice>();
			Ptr<WifiPhy> wifiPhy = netDevice->GetPhy()->GetObject<WifiPhy>();
			wifiPhy->SetChannelNumber(uint16_t(110));
		}
	}

	// creating channelsToRadios used in Ipv4JointRouting::LookupStatic2
	int *channelsToRadios = new int[nNodes * craa->m_nRadios];
	for (unsigned int i = 0; i < nNodes * craa->m_nRadios; i++) {
		channelsToRadios[i] = 0;
	}

	// setting channels to radios
	std::cout << "Nodes - m_allocatedChannels:" << std::endl;
	for (unsigned int i = 0; i < nNodes; i++) {
		std::cout << "Node " << i << " -- ";
		std::set<CRAAChannel> usedChannels = craa->m_channelsToNodes[i];
		std::cout << "usedChannels = " << usedChannels.size() << ": ";
		if ((int) usedChannels.size() > craa->m_nRadios) {
			std::cout << "ERROR - usedChannels > m_nRadios" << std::endl;
			exit(1);
		}

		int radio = 0;
		for (std::set<CRAAChannel>::iterator it = usedChannels.begin(); it != usedChannels.end(); ++it, radio++) {
			CRAAChannel channel = *it;
			std::cout << channel << " ";
			Ptr<ns3::Node> node;
			node = wifiNodes.Get(i);
			Ptr<WifiNetDevice> netDevice = node->GetDevice(radio)->GetObject<WifiNetDevice>();
			Ptr<WifiPhy> wifiPhy = netDevice->GetPhy()->GetObject<WifiPhy>();
			wifiPhy->SetChannelNumber(uint32_t(channel));
			channelsToRadios[i * craa->m_nRadios + radio] = channel;
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	std::cout << "channelsToRadios:" << std::endl;
	for (unsigned int i = 0; i < nNodes; i++) {
		std::cout << "Node " << i << ":" << std::endl;
		Ptr<ns3::Node> node;
		node = wifiNodes.Get(i);
		for (int j = 0; j < craa->m_nRadios; j++) {
			if (channelsToRadios[i * craa->m_nRadios + j] != 0) {
				std::cout << "interface = " << j << " channel = " << channelsToRadios[i * craa->m_nRadios + j] << " Address: "
						<< node->GetObject<Ipv4>()->GetAddress(j+1,0).GetLocal() << std::endl;
			}
		}
	}
	std::cout << "set_channels end." << std::endl;
	std::cout << std::endl;

	return channelsToRadios;
}


/*
 *\\***************************************************\\
 *\\		Implementation Plots e Files Results	   \\
 *\\***************************************************\\
 */
void configurePlotDir() {
	struct stat st = {0};
	if (stat("plot", &st) == -1) {
		mkdir("plot", 0700);
	}
	if (stat("plot/qualityVariation", &st) == -1) {
		mkdir("plot/qualityVariation", 0700);
	}
}

void QualityVariationPlotFile (std::map< std::pair<int,int>, std::vector<struct MatrixQuality> > matrixQuality, char * prefixFile ) {
	configurePlotDir();
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
			  //dataset_REC.Add(id, 1);
		  } else {
			  //dataset_REC.Add(id, elemq.SINR/elemq.SNR);
			  dataset_REC.Add(id, elemq.SINR);
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
	  plot.AddDataset (dataset_SINR);
	  //plot.AddDataset (dataset_IR);
	  //plot.AddDataset (dataset_PER);
	  //plot.AddDataset (dataset_MMS);
	  //plot.AddDataset (dataset_MOD);
	  //plot.AddDataset (dataset_MME);
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
