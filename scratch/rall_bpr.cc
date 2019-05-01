/*
 * rall.cc
 *
 *  Created on: Oct 20, 2016
 *      Author: Vinícius Nunes
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
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/internet-module.h"
#include "ns3/propagation-module.h"
#include "ns3/lab-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/gnuplot.h"

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("RALL_TEST");

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
} static_SpectrumModelWifi_MHz_initializer_instance(2412e6/*5180e6*/);

class TraceValuesCount {
public:
	int m_MacTxDrop; //A packet has been dropped in the MAC layer before being queued for transmission.
	int m_MacRxDrop; //A packet has been dropped in the MAC layer after it has been passed up from the physical layer.
	int m_PhyTxDrop; //Trace source indicating a packet has been dropped by the device during transmission.
	int m_PhyRxDrop; //Trace source indicating a packet has been dropped by the device during reception.
	int m_ArpDrop;	//Trace source indicating a packet has been dropped by ARP protocol.
	unsigned int m_QuantityHops; //Trace source indicating a quantity hops for packet received.

	int recalculates;
	int qtdLinkUpTrue;
	int qtdLinkUpFalse;
	TraceValuesCount():m_MacTxDrop(0), m_MacRxDrop(0),
						 m_PhyTxDrop(0), m_PhyRxDrop(0),
						 m_ArpDrop(0), m_QuantityHops(0),
						 recalculates(0), qtdLinkUpTrue(0),
						 qtdLinkUpFalse(0){}
} reportResultsInstance;

struct MatrixQuality {
	double MMS;
	double MME;
	double SNR;
	double SINR;
	double MOD;
	double PER;
	bool RECEIVED;
};


std::map< std::pair<int,int>, std::vector<struct MatrixQuality> > matrixQuality;
static void UpdateQualityLink (int, int, double, double, double, bool);
static void addInterferingNodes(Ptr<MultiModelSpectrumChannel>);
static int* set_channels(algorithm::CRAA *, NodeContainer, uint32_t);
static void ReceiveFlows(const Ipv4Header &, Ptr<const Packet>, uint32_t);
//static void ReceiveFlows(Ptr< const Packet > packet, const Address &address);
static void GenerateFlows(int, int, NodeContainer, Ipv4InterfaceContainer, double, double);
static void PRELOAD(int, int, int, double, NodeContainer);
static void PRELOAD_ReceiveTraffic(Ptr<Socket>);
static void PRELOAD_GenerateTraffic(Ptr<Socket>, uint32_t, uint32_t, Time);

//function pots
void QualityVariationPlotFile (std::map< std::pair<int,int>, std::vector<struct MatrixQuality> >, char * );

element::Graph * graph;
algorithm::LABORARouting * routing;

void print( ) {
	std::cout << "print\n";
	((algorithm::RALL * )routing)->recreatesAllRoutes();
	graph->showAdjacencyList();
}

void ArpDrop(Ptr<const Packet> p) {reportResultsInstance.m_ArpDrop++;}
void MacTxDrop(Ptr<const Packet> p) {reportResultsInstance.m_MacTxDrop++;}
void MacRxDrop(Ptr<const Packet> p) {reportResultsInstance.m_MacRxDrop++;}
void PhyTxDrop(Ptr<const Packet> p) {reportResultsInstance.m_PhyTxDrop++;}
void PhyRxDrop(Ptr<const Packet> p) {
	reportResultsInstance.m_PhyRxDrop++;
	return;
	InfoTag infoTag;
	if ( p->PeekPacketTag(infoTag) ) {
		if(TypeTag::TAG_DATA == infoTag.GetTypeTag() ) {
			if (graph->existEdge(infoTag.GetPreviousNodeID(), infoTag.GetNodeID())) {
				path * pt = ((algorithm::RALL * )routing)->getRoute(element::Flow(infoTag.GetSrc(), 0, 0, 0));
				if(pt) {
					for (unsigned int i = 0; i < pt->size()-1; i++) {
						if ( (unsigned int)(*pt)[i] == infoTag.GetPreviousNodeID() && (unsigned int)(*pt)[i+1] == infoTag.GetNodeID() ) {
							//std::cout << infoTag.GetSrc() << " id=" << infoTag.GetId()  <<":\n";
							//((algorithm::RALL * )routing)->printExistingRoutes();
							reportResultsInstance.m_PhyRxDrop++;
							break;
						}
					}
				}
			}
		}
	}
}


unsigned int countUpdates = 0;
void callRecalculateRoutes() {
	countUpdates++;
	//if (countUpdates == graph->listEdges.size()) {
	if (countUpdates == 10) {
		reportResultsInstance.recalculates++;
		algorithm::RALL * rall = (algorithm::RALL *)routing;
		rall->recreatesAllRoutes();
		countUpdates = 0;
	}
}


int main(int argc, char *argv[]) {

	//LogComponentEnable("SpectrumWifiPhyCustom", LOG_FUNCTION);

	double SIMULATION_TIME = 8000; //seconds
	int N_NODES 	= 10;	//Number of Nodes
	int N_RADIOS 	= 1; 	//Number of Radios
	int N_CHANNELS 	= 11;	//Number of Channels
	int N_FFLOWS 	= 2; 	//Number of Flow

	//Parameters to determine area:
	int QUADRANT_DIVISION	= 3; 	//Quadrant division
	int QUADRANT_I 			= 2; 	//Quadrant position
	int QUADRANT_J 			= 2;	//Quadrant position
	//40 nodes:
	double SIZE_AREA 		= 800; 	//Size area
	double MINIMUM_DISTANCE = 80.0; //Minimum distance
	double MAXIMUM_DISTANCE = 130.0;//Maximum distance

	//double SIZE_AREA 		= 800;	 	//Size area
	//double MINIMUM_DISTANCE = 80.0; 	//Minimum distance
	//double MAXIMUM_DISTANCE = 130.0;	//Maximum distance

	char algorithm[20] = "RALL";	//Algorithm strings
	//char algorithm[20] = "BPR";	//Algorithm strings
	int seed = 7;					//SEED


	//Command line arguments
	CommandLine cmd;
	cmd.AddValue ("algorithm", "Algorithm", algorithm);
	cmd.AddValue ("N_NODES", "N_NODES", N_NODES);
	cmd.AddValue ("seed", "Seed", seed);
	cmd.AddValue ("N_RADIOS", "N_RADIOS", N_RADIOS);
	cmd.Parse (argc, argv);


	std::cout << "Configuration" << std::endl <<
			"SIMULATION_TIME: " << SIMULATION_TIME << std::endl <<
			"N_NODES: " << N_NODES  << std::endl <<
			"N_RADIOS: " << N_RADIOS << std::endl <<
			"N_CHANNELS: " << N_CHANNELS << std::endl <<
			"N_FFLOWS: " << N_FFLOWS << std::endl <<
			"ALGORITHM: " << algorithm << std::endl;

	char ssid_str[20]; 				//SSID string

	sprintf(ssid_str, "%s", "data");

	//Set simulator seed:
	ns3::SeedManager::SetSeed(seed);

	//std::string phyMode ("DsssRate11Mbps");
	//std::string phyMode ("OfdmRate36Mbps");

	//the node 0 is the sink
	NodeContainer wifiNodes;
	wifiNodes.Create(N_NODES);


	/*
	Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
	Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
	//Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel>();
	//lossModel->SetPathLossExponent(2.7);
	//lossModel->SetReference(1, 46.6777);
	Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
	spectrumChannel->AddPropagationLossModel(lossModel);
	spectrumChannel->SetPropagationDelayModel(delayModel);

	SpectrumWifiPhyCustomHelper wifiPhy = SpectrumWifiPhyCustomHelper::Default();
	wifiPhy.SetChannel(spectrumChannel);
	wifiPhy.SetErrorRateModel("ns3::NistErrorRateModel");
	wifiPhy.Set("Frequency", UintegerValue(5180));
	//wifiPhy.Set("TxPowerStart", DoubleValue(5)); // dBm  (1.26 mW)
	//wifiPhy.Set("TxPowerEnd", DoubleValue(5));
	wifiPhy.Set("TxPowerStart", DoubleValue(5)); // dBm  (1.26 mW)
	wifiPhy.Set("TxPowerEnd", DoubleValue(5));
	wifiPhy.Set("ShortGuardEnabled", BooleanValue(false));
	wifiPhy.Set("ChannelWidth", UintegerValue(20));
	 */


	/*
	 * Create WifiChannel (comunication channel):
	 * Create a channel helper in a default working state. By default, we create
	 * a channel model with a propagation delay equal to a constant, the speed
	 * of light, and a propagation loss based on a log distance model with a
	 * reference loss of 46.6777 dB at reference distance of 1m.
	 *
	 * Create WifiPhy:
	 * Create a phy helper without any parameter set. The user must set them all
	 * to be able to call Install later.
	 */

	YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
	Ptr<YansWifiChannel> channel = wifiChannel.Create ();
	Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel>();;
	lossModel->SetPathLossExponent (2.7);
	lossModel->SetReference (1, 46.6777);
	//Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
	channel->SetPropagationLossModel(lossModel);
	YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
	wifiPhy.SetChannel(channel);
	//wifiPhy.Set("TxPowerStart", DoubleValue(5)); // dBm  (1.26 mW)
	//wifiPhy.Set("TxPowerEnd", DoubleValue(5));
	wifiPhy.Set ("TxPowerStart",DoubleValue(20));
	wifiPhy.Set ("TxPowerEnd", DoubleValue(20));
	//wifiPhy.SetErrorRateModel("ns3::NistErrorRateModel");
	wifiPhy.SetErrorRateModel("ns3::YansErrorRateModel");


	WifiHelper wifi;
	wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

	Config::SetDefault("ns3::WifiMacQueue::MaxDelay", TimeValue(MilliSeconds(2000)));
	Config::SetDefault("ns3::WifiMacQueue::MaxPacketNumber", UintegerValue(1000));
	Config::SetDefault("ns3::ArpCache::PendingQueueSize", UintegerValue (800));
	Config::SetDefault ("ns3::WifiRemoteStationManager::MaxSlrc", UintegerValue (0));
	wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
				/*
				 * Default power level to be used for transmissions. This is the
				 * power level that is used by all those WifiManagers that do not
				 * implement TX power control.
				 */
				//"DefaultTxPowerLevel", UintegerValue(20),
				/*
				 * The maximum number of retransmission attempts for an RTS. This
				 * value will not have any effect on some rate control algorithms.
				 * Set with class: ns3::UintegerValue
				 * Underlying type: uint32_t 0:4294967295
				 * Initial value: 7
				 */
				"MaxSsrc", UintegerValue (0),
				/*
				 * The maximum number of retransmission attempts for a DATA packet.
				 * This value will not have any effect on some rate control
				 * algorithms.
				 * Set with class: ns3::UintegerValue
				 * Underlying type: uint32_t 0:4294967295
				 * Initial value: 7
				 */
				"MaxSlrc", UintegerValue (0),
				/*
				 * Turn off RTS/CTS for frames below 2049 bytes.
				 * If the size of the data packet + LLC header + MAC header + FCS
				 * trailer is bigger than this value, we use an RTS/CTS handshake
				 * before sending the data, as per IEEE Std. 802.11-2007,
				 * Section 9.2.6. This value will not have any effect on some rate
				 * control algorithms.
				 * Set with class: ns3::UintegerValue
				 * Underlying type: uint32_t 0:4294967295
				 * Initial value: 2346
				 */
				"RtsCtsThreshold", UintegerValue (10000),
				/*
				 * Disable fragmentation for frames below 2049 bytes.
				 */
				"FragmentationThreshold", StringValue ("10000"),
				/*
				 * DataMode: The transmission mode to use for every data packet
				 * transmission.
				 * Set with class: WifiModeValue
				 * Underlying type: WifiMode
				 */
				"DataMode", StringValue ("DsssRate11Mbps"),
				//"DataMode", StringValue ("OfdmRate36Mbps"),
				/*
				 * ControlMode: The transmission mode to use for every control
				 * packet transmission.
				 * Set with class: WifiModeValue
				 * Underlying type: WifiMode
				 */
				"ControlMode", StringValue ("DsssRate11Mbps"),
				//"ControlMode", StringValue ("OfdmRate36Mbps"),
				/*
				 * NonUnicastMode: Wifi mode used for non-unicast transmissions.
				 * Set with class: WifiModeValue
				 * Underlying type: WifiMode
				 */
				"NonUnicastMode",StringValue ("DsssRate11Mbps"));
				//"NonUnicastMode",StringValue ("OfdmRate36Mbps"));


	//Create WifiMac for each wifi node:
	NqosWifiMacHelper mac;
	mac = NqosWifiMacHelper::Default();
	mac.SetType ("ns3::AdhocWifiMac", "Ssid", SsidValue(Ssid(ssid_str)));

	//install radios in nodes:
	NetDeviceContainer wifiDevice[N_RADIOS];
	for (int i = 0; i < N_RADIOS; i++) {
		wifiDevice[i] = wifi.Install(wifiPhy, mac, wifiNodes);
	}

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
	if (!pa->verifyAllocation(mobility, N_NODES,MINIMUM_DISTANCE,MAXIMUM_DISTANCE)) {
		std::cout << "verify Allocation not correct!";
		exit(1);
	}

	graph = new element::Graph(mobility.GetMatrix(), N_NODES);

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

	//Create Routing:
	if (strcmp (algorithm, "BPR") == 0) {
		algorithm::BPR * bpr = new algorithm::BPR(N_NODES, 2.5, graph);
		bpr->CalculateCandidatesPaths();
		routing = dynamic_cast<algorithm::LABORARouting * >(bpr);
	} else if (strcmp (algorithm, "RALL") == 0) {
		routing = new algorithm::RALL(0.5, 0.5, graph);
		graph->m_callbackUpdateLink = &callRecalculateRoutes;
	} else {
		std::cout << "Routing algorithm	not correct!";
		exit(1);
	}

	//Create CRAA:
	std::set<CRAAChannel> channels;
	for (int i = 1; i <= N_CHANNELS; i++) {
		channels.insert(i);
	}
	std::map< element::Edge, std::map< element::Edge, double> > per;
	algorithm::CRAA * craa = new algorithm::CRAA(routing, graph->U.size(), 0.0, channels, N_RADIOS, N_FFLOWS, per, 1);
	craa->RunInitial();

	//Set initial channel:
	int *channelsToRadios = set_channels(craa, wifiNodes, N_NODES);
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

	Simulator::Schedule(Seconds(90.0), &print);
	//addInterferingNodes(spectrumChannel);
	addInterferingNodes(NULL);
	PRELOAD(50, N_NODES, N_RADIOS, 1.0, wifiNodes);
	//GenerateFlows(0, N_NODES, wifiNodes, wifiNodeInterface, SIMULATION_TIME, 100.0);
	GenerateFlows(0, N_NODES, wifiNodes, wifiNodeInterface, SIMULATION_TIME, N_NODES*50 + 100);

	//Traces
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop", MakeCallback(&MacTxDrop));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxDrop", MakeCallback(&MacRxDrop));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop", MakeCallback(&PhyRxDrop));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop", MakeCallback(&PhyTxDrop));
	Config::ConnectWithoutContext("/NodeList/*/$ns3::ArpL3Protocol/Drop", MakeCallback(&ArpDrop));
	Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/UpdateLinkQuality", MakeCallback(&UpdateQualityLink));

	//Install FlowMonitor on all nodes
	FlowMonitorHelper flowmon;
	Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
	monitor->SetAttribute("DelayBinWidth", DoubleValue (0.100));
	monitor->SerializeToXmlFile("FlowMonitorRALL.xml", true, true);

	//Install Animation
	AnimationInterface anim ("animationRALL.xml"); // Mandatory
	//anim.EnableIpv4RouteTracking ("routingtable-wireless-RALL.xml", Seconds (0), Seconds (5), Seconds (0.25)); //Optional
	anim.EnablePacketMetadata (); // Optional

	std::cout << "Start...\n";
	Simulator::Stop(Seconds(SIMULATION_TIME + 1));
	Simulator::Run();

	//Print per flow statistics
	monitor->CheckForLostPackets();
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
	FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
	int count = 0;
	int lostPackets = 0;
	int rxPackets = 0;
	double throughput = 0;
	double delayAvg = 0.0;
	Histogram delayHistogram(0.100);
	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i) {
		Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
		std::cout << "\nFlow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << "["<< t.sourcePort << "," << t.destinationPort <<"])\n";
		std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
		std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
		std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024 << " Mbps\n";
		std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
		std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
		std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024 << " Mbps\n";
		std::cout << "  lost Packets: " << i->second.lostPackets << "\n";
		if(i->second.rxPackets > 0)
			std::cout << "  AVG Delay: " << i->second.delaySum.GetMilliSeconds() / i->second.rxPackets << " ms\n\n";
		else
			std::cout << "  AVG Delay: -- ms\n\n";


		if(i->second.rxPackets > 0) {
			delayAvg += i->second.delaySum.GetMilliSeconds() / i->second.rxPackets;
			count++;
		}
		lostPackets += i->second.lostPackets;
		rxPackets += i->second.rxPackets;
		throughput += (i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024);

		Histogram dh = i->second.delayHistogram;
		for (unsigned int t = 0; t < dh.GetNBins(); t++) {
			double value = delayHistogram.GetBinWidth(0) * dh.GetBinStart(t);
			for (unsigned int u = 0; u < dh.GetBinCount(t); u++)
				delayHistogram.AddValue(value);
		}

	}

	char dataFileName[50];
	sprintf(dataFileName,"plot/%s-%d-R%d.dat", algorithm ,N_NODES, N_RADIOS);
	struct stat st;
	int hd = stat(dataFileName, &st);
	std::ofstream dataFile (dataFileName, std::ofstream::app);

	if (hd == -1)
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
				 << " qtdLinkUpFalse "
				 << std::endl;

	dataFile << seed << "\t"
			 << std::setfill (' ') << std::setw (15) << rxPackets << " "
			 << std::setfill (' ') << std::setw (15) << lostPackets << " "
			 << std::setfill (' ') << std::setw (15) << (double)lostPackets/(lostPackets+rxPackets) << " "
			 << std::setfill (' ') << std::setw (15) << throughput/(N_NODES -1) << " "
			 << std::setfill (' ') << std::setw (15) << delayAvg/count << " "
			 << std::setfill (' ') << std::setw (15) << (double)reportResultsInstance.m_QuantityHops/rxPackets << " "
			 << std::setfill (' ') << std::setw (15) << reportResultsInstance.m_MacRxDrop << " "
			 << std::setfill (' ') << std::setw (15) << reportResultsInstance.m_MacTxDrop << " "
			 << std::setfill (' ') << std::setw (15) << reportResultsInstance.m_PhyRxDrop << " "
			 << std::setfill (' ') << std::setw (15) << reportResultsInstance.m_PhyTxDrop << " "
			 << std::setfill (' ') << std::setw (15) << reportResultsInstance.m_ArpDrop << " "
			 << std::setfill (' ') << std::setw (15) << reportResultsInstance.recalculates << " "
			 << std::setfill (' ') << std::setw (15) << reportResultsInstance.qtdLinkUpTrue << " "
			 << std::setfill (' ') << std::setw (15) << reportResultsInstance.qtdLinkUpFalse << " "
			 << std::setfill (' ') << std::setw (15) << std::endl;

	for (unsigned int t = 0; t < delayHistogram.GetNBins(); t++) {
		std::cout << "[" << delayHistogram.GetBinStart(t) * 1000 << "-" <<  delayHistogram.GetBinEnd(t) * 1000 << "];";
	}
	std::cout << std::endl;
	for (unsigned int t = 0; t < delayHistogram.GetNBins(); t++) {
		std::cout << delayHistogram.GetBinCount(t) << ";";
	}

	dataFile.close();

	Simulator::Destroy();

	char prefixFileName[10];
	sprintf(prefixFileName,"%s-%d-seed:%d", algorithm ,N_NODES, seed);
	QualityVariationPlotFile(matrixQuality, prefixFileName);

	std::cout << "\n\n\nfim!\n";
	return 0;
}



/*
 *\\***************************************************\\
 *\\			Implementation functions:			   \\
 *\\***************************************************\\
 */

static void PRELOAD(int qtdPkts, int N_NODES, int N_RADIOS, double interval, NodeContainer wifiNodes) {
	Time interPacketInterval = Seconds(interval);
	Address BroadcastAddress (InetSocketAddress (Ipv4Address::GetBroadcast(), 80));
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

	Ptr<Socket> recvSink = Socket::CreateSocket(wifiNodes.Get(0), tid);
	InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);
	recvSink->Bind(local);
	recvSink->SetRecvCallback(MakeCallback (&PRELOAD_ReceiveTraffic));

	std::vector<Ptr<Socket>> sendSockets;
	std::vector<Ptr<Socket>> recvSockets;
	int t=0;
	for (int i=1; i <= N_NODES-1; i++) {
		for (int r=0; r < N_RADIOS; r++, t++) {
			//double time = (double)(rand() % 300)/100;

			sendSockets.push_back(Socket::CreateSocket(wifiNodes.Get(i), tid));
			recvSockets.push_back(Socket::CreateSocket(wifiNodes.Get(i), tid));

			sendSockets[t]->Bind();
			if (i==6) {
				std::cout << "Address: " << wifiNodes.Get(i)->GetDevice(r+1)->GetAddress()
						  << "         " << wifiNodes.Get(i)->GetObject<Ipv4>()->GetAddress(r+1,0).GetLocal()
						  << std::endl;
			}
			sendSockets[t]->BindToNetDevice(wifiNodes.Get(i)->GetDevice(r+1));
			sendSockets[t]->Connect(BroadcastAddress);
			sendSockets[t]->SetAllowBroadcast(true);
			sendSockets[t]->ShutdownRecv();

			recvSockets[t]->Bind(local);
			recvSockets[t]->ShutdownSend();
			recvSockets[t]->SetRecvCallback(MakeCallback (&PRELOAD_ReceiveTraffic));

			Simulator::ScheduleWithContext(sendSockets[t]->GetNode()->GetId(),
										   //Seconds(5+time), &PRELOAD_GenerateTraffic,
										   Seconds((i-1) * 50 + 10), &PRELOAD_GenerateTraffic,
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
		}
	}
}

static void ReceiveFlows(const Ipv4Header &header, Ptr<const Packet> p, uint32_t interface) {
//static void ReceiveFlows(Ptr< const Packet > p, const Address &address) {
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


static void GenerateFlows(int idSink, int N_NODES, NodeContainer wifiNodes, Ipv4InterfaceContainer wifiNodeInterface, double simulationTime, double startAfter) {
	Address sinkAddress (InetSocketAddress (wifiNodeInterface.GetAddress(0), 80));

	PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 80));
	ApplicationContainer packetSink = packetSinkHelper.Install(wifiNodes.Get(0));
	packetSink.Start(Seconds(startAfter));
	packetSink.Stop(Seconds(simulationTime));

	LaboraAppHelper laboraAppHelper("ns3::UdpSocketFactory", sinkAddress, LaboraAppHelper::Class_3);
	laboraAppHelper.SetAttribute("MaxPackets", UintegerValue(100));
	laboraAppHelper.SetAttribute("DataRate", StringValue("1Mbps"));
	//laboraAppHelper.SetAttribute("DataRate", StringValue("512kb/s"));
	laboraAppHelper.SetAttribute("PacketSize", UintegerValue(1024));
	laboraAppHelper.SetAttribute ("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
	laboraAppHelper.SetAttribute ("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

	ApplicationContainer source;
	for (int i = 1; i < N_NODES; i++) {
		source.Add(laboraAppHelper.Install(wifiNodes.Get(i)));
	}
	//source.Add(laboraAppHelper.Install(wifiNodes.Get(1)));

	source.Start (Seconds (startAfter+1));
	source.Stop (Seconds (simulationTime));

	//Config::ConnectWithoutContext("/NodeList/*/ns3::PacketSink/Rx", MakeCallback(&ReceiveFlows));
	Config::ConnectWithoutContext("/NodeList/*/$ns3::Ipv4L3Protocol/LocalDeliver", MakeCallback(&ReceiveFlows)); // \FAZER: Endenter a forma como os pacotes são passados para camada de cima, a aplicação do SINk não tá recebendo os pacotes.
}

static void addInterferingNodes(Ptr<MultiModelSpectrumChannel> spectrumChannel) {
	return;
	// Configure waveform generator
	NodeContainer interferingNode;
	interferingNode.Create(1);

	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
	positionAlloc->Add(Vector(100.0, 100.0, 0.0));		//it is the starting position
	positionAlloc->Add(Vector(490, 490.0, 0.0));		//it is the starting position
	//stationPositionAlloc->Add(Vector(375.0, 125.0, 0.0));		//it is the starting position
	//stationPositionAlloc->Add(Vector(125.0, 375.0, 0.0));		//it is the starting position
	//stationPositionAlloc->Add(Vector(375.0, 375.0, 0.0));		//it is the starting position
	MobilityHelper nodeMobilityHelper;
	nodeMobilityHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	//nodeMobilityHelper.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
	//nodeMobilityHelper.SetMobilityModel ("ns3::RandomWalk2dMobilityModel", "Bounds", RectangleValue (Rectangle (75, 625, 75, 625)), "Distance", DoubleValue(180.0));
	//nodeMobilityHelper.SetMobilityModel ("ns3::RandomWalk2dMobilityModel", "Bounds", RectangleValue (Rectangle (220, 250, 200, 230)), "Distance", DoubleValue(500.0));
	nodeMobilityHelper.SetPositionAllocator(positionAlloc);
	nodeMobilityHelper.Install(interferingNode);
	//(interferingNode.Get(0)->GetObject<ConstantVelocityMobilityModel>())->SetVelocity(Vector(0.0, 2.5, 0.0));

	//double waveformPower = 0.0398;
	double waveformPower = 0.0298;
	//Ptr<SpectrumValue> wgPsd = Create<SpectrumValue>(SpectrumModelWifi5180MHz);
	Ptr<SpectrumValue> wgPsd = Create<SpectrumValue>(SpectrumModelWifi_MHz);
	*wgPsd = waveformPower / (100 * 180000);

	WaveformGeneratorHelper waveformGeneratorHelper;
	waveformGeneratorHelper.SetChannel(spectrumChannel);
	waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd);
	waveformGeneratorHelper.SetPhyAttribute("Period", TimeValue(Seconds(0.1)));
	waveformGeneratorHelper.SetPhyAttribute("DutyCycle", DoubleValue(0.5));
	NetDeviceContainer waveformGeneratorDevices = waveformGeneratorHelper.Install(interferingNode);

	Simulator::Schedule(Seconds(50.002), &WaveformGenerator::Start, waveformGeneratorDevices.Get(0)->GetObject<NonCommunicatingNetDevice>()->GetPhy()->GetObject<WaveformGenerator>());
}

static void UpdateQualityLink (int sourceVertice, int destinationVertice, double snr, double sinr, double per, bool received) {
	//double ir = sinr/snr;
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
			reportResultsInstance.qtdLinkUpTrue++;
			mq.MOD = (*ed).quality;
		}
		else {
			reportResultsInstance.qtdLinkUpFalse++;
			mq.MOD = matrixQuality[std::make_pair(sourceVertice,destinationVertice)].back().MOD;
		}
		matrixQuality[std::make_pair(sourceVertice,destinationVertice)].push_back(mq);
	}
}

static int* set_channels(algorithm::CRAA *craa, NodeContainer wifiNodes, uint32_t nNodes) {

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
 *\\		Implementation Plots functions:			   \\
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

