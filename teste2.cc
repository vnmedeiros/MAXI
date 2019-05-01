/*
 * teste.cc
 *
 *  Created on: Oct 20, 2016
 *      Author: vinicius
 */

#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>

#include "ns3/core-module.h"
#include "ns3/config-store-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/internet-module.h"
#include "ns3/lab-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/applications-module.h"

using namespace ns3;

element::Graph * graph;

Ptr<SpectrumModel> SpectrumModelWifi5180MHz;

class static_SpectrumModelWifi5180MHz_initializer {
public:
	static_SpectrumModelWifi5180MHz_initializer() {
		BandInfo bandInfo;
		bandInfo.fc = 5180e6;
		bandInfo.fl = 5180e6 - 10e6;
		bandInfo.fh = 5180e6 + 10e6;

		Bands bands;
		bands.push_back(bandInfo);

		SpectrumModelWifi5180MHz = Create<SpectrumModel>(bands);
	}

} static_SpectrumModelWifi5180MHz_initializer_instance;

/*static void ReceivePacket(Ptr<Socket> socket) {
	Address addr;
	std::ostringstream oss;
	Ptr<Packet> packet;
	while ((packet = socket->Recv())) {
		socket->GetSockName(addr);
		InfoTag infoTag;
		packet->RemovePacketTag(infoTag);
		std::cout << std::setfill('0') << std::setw(10) << Simulator::Now().GetSeconds()
				<< " [node=" << socket->GetNode()->GetId() << "]         Received packet id:"
				<< infoTag.GetId() << " of node:" << infoTag.GetSrc()
				<< " snr:" << infoTag.GetSnr() << ", signal:" << infoTag.GetSignal() <<  std::endl;
	}
}*/

static void GenerateTraffic(Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval) {
	if (pktCount > 0) {
		//std::cout << std::setfill('0') << std::setw(10) << Simulator::Now().GetSeconds() << " [node=" << socket->GetNode()->GetId() << "]         create and send packet id:" << pktCount << std::endl;
		//socket->GetNode()->GetDevice(0)->GetAddress();
		Ptr<Packet> pkt = Create<Packet>(pktSize);
		InfoTag infoTag;
		infoTag.SetSrc(socket->GetNode()->GetId());
		infoTag.SetId(pktCount);
		pkt->AddPacketTag(infoTag);
		socket->Send(pkt);
		Simulator::Schedule(pktInterval, &GenerateTraffic, socket, pktSize, pktCount - 1, pktInterval);
	} else {
		socket->Close();
	}
}

static void ReceiveTrafficGrubbiness(Ptr<Socket> socket) {
	Address addr;
	Ptr<Packet> packet;
	while ((packet = socket->Recv())) {
		socket->GetSockName(addr);
		InfoTag infoTag;
		if ( packet->PeekPacketTag(infoTag) ) {
			packet->RemovePacketTag(infoTag);
			int vs = infoTag.GetSrc();
			int vd = socket->GetNode()->GetId();
			element::Edge item(vs, vd, infoTag.GetSnr());
			std::vector<element::Edge>::iterator ed = std::find(graph->listEdges[vs].begin(), graph->listEdges[vs].end(), item);
			if (ed != graph->listEdges[vs].end()) {
				(*ed).quality = infoTag.GetSnr();
			}
		}
	}
}

int* set_channels(algorithm::CRAA *craa, NodeContainer wifiNodes, uint32_t nNodes) {
	std::cout << "set_channels:" << std::endl;

	/*
	 * setting an unused channel to all radios
	 */
	for (uint32_t i = 0; i < nNodes; i++) {
		Ptr<ns3::Node> node = wifiNodes.Get(i);
		for (int j = 0; j < craa->m_nRadios; j++) {
			Ptr<WifiNetDevice> netDevice = node->GetDevice(j)->GetObject<WifiNetDevice>();
			Ptr<WifiPhy> wifiPhy = netDevice->GetPhy()->GetObject<WifiPhy>();
			wifiPhy->SetChannelNumber(uint16_t(110));
		}
	}
	/*
	 * creating channelsToRadios used in Ipv4JointRouting::LookupStatic2
	 */

	int *channelsToRadios = new int[nNodes * craa->m_nRadios];
	for (unsigned int i = 0; i < nNodes * craa->m_nRadios; i++) {
		channelsToRadios[i] = 0;
	}

	/*
	 * setting channels to radios
	 */
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
		for (int j = 0; j < craa->m_nRadios; j++) {
			if (channelsToRadios[i * craa->m_nRadios + j] != 0) {
				std::cout << "interface = " << j << " channel = " << channelsToRadios[i * craa->m_nRadios + j] << std::endl;
			}
		}
	}
	std::cout << "set_channels end." << std::endl;
	std::cout << std::endl;

	return channelsToRadios;
}

int* change_channels(algorithm::CRAA *craa, NodeContainer wifiNodes) {
	/*
	 * creating channelsToRadios used in Ipv4JointRouting::LookupStatic2
	 */
	//int nNodes = craa->m_routing->getGraph()->listEdges.size();
	int nNodes = 0;
	int *channelsToRadios = new int[nNodes * craa->m_nRadios];
	for (int i = 0; i < nNodes * craa->m_nRadios; i++) {
		channelsToRadios[i] = 0;
	}

	/*
	 * setting channels to radios
	 */
	//std::cout << "Nodes - m_allocatedChannels:" << std::endl;
	for (int i = 0; i < nNodes; i++) {
		//std::cout << "Node " << i << ":" << std::endl;

		std::set<CRAAChannel> usedChannels = craa->m_channelsToNodes[i];
		//std::cout << "nUsedChannels = " << usedChannels.size () << ": ";
		if ((int) usedChannels.size() > craa->m_nRadios) {
			//std::cout << "ERROR - nUsedChannels > m_nRadios" << std::endl;
			exit(1);
		}

		int radio = 0;
		for (std::set<CRAAChannel>::iterator it = usedChannels.begin();	it != usedChannels.end(); ++it, radio++) {
			CRAAChannel channel = *it;
			//std::cout << channel << " ";

			Ptr<ns3::Node> node;
			if (i == 0) {
				node = wifiNodes.Get(i);
			}

			Ptr<WifiNetDevice> netDevice = node->GetDevice(radio)->GetObject<WifiNetDevice>();
			Ptr<SpectrumWifiPhy> wifiPhy = netDevice->GetPhy()->GetObject<SpectrumWifiPhy>();
			if (channel != (CRAAChannel) wifiPhy->GetChannelNumber()) {
				wifiPhy->SetChannelNumber(uint32_t(channel));
			}

			channelsToRadios[i * craa->m_nRadios + radio] = channel;
		}
		//std::cout << std::endl;
	}
	//std::cout << std::endl;

	return channelsToRadios;
}

void addInterferingNodes(Ptr<MultiModelSpectrumChannel> spectrumChannel) {
	// Configure waveform generator
	NodeContainer interferingNode;
	interferingNode.Create(1);

	Ptr<ListPositionAllocator> stationPositionAlloc2 = CreateObject<ListPositionAllocator>();
	stationPositionAlloc2->Add(Vector(0.0, 0.0, 0.0));		//it is the starting position
	MobilityHelper nodeMobilityHelper2;
	nodeMobilityHelper2.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
	nodeMobilityHelper2.SetPositionAllocator(stationPositionAlloc2);
	nodeMobilityHelper2.Install(interferingNode);
	(interferingNode.Get(0)->GetObject<ConstantVelocityMobilityModel>())->SetVelocity(Vector(10.0, 0.0, 0.0));

	double waveformPower = 0.0098;
	//double waveformPower = 0.1;
	Ptr<SpectrumValue> wgPsd = Create<SpectrumValue>(SpectrumModelWifi5180MHz);
	*wgPsd = waveformPower / (100 * 180000);

	WaveformGeneratorHelper waveformGeneratorHelper;
	waveformGeneratorHelper.SetChannel(spectrumChannel);
	waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd);
	waveformGeneratorHelper.SetPhyAttribute("Period", TimeValue(Seconds(0.03)));
	waveformGeneratorHelper.SetPhyAttribute("DutyCycle", DoubleValue(1));
	NetDeviceContainer waveformGeneratorDevices = waveformGeneratorHelper.Install(interferingNode);

	Simulator::Schedule(Seconds(0.002), &WaveformGenerator::Start, waveformGeneratorDevices.Get(0)->GetObject<NonCommunicatingNetDevice>()->GetPhy()->GetObject<WaveformGenerator>());
}

NS_LOG_COMPONENT_DEFINE("Teste");

int main(int argc, char *argv[]) {
	double distance = 50;
	double simulationTime = 500; 	//seconds
	double interval = 1.0; 			// seconds
	Time interPacketInterval = Seconds(interval);
	int nNodes = 10;
	int nRadios = 2; 	//Number of Radios
	int nChannels = 11;	//Number of Channels
	int nFlows = 2; 	//Number of Flow
	int seed = 6;

	//////////////////////////////////////////////////
	double sizeArea = 500; 		//Size area
	int quadrantDivision = 3; 	//Quadrant division
	int quadrantI = 2; 			//Quadrant position
	int quadrantJ = 2; 			//Quadrant position
	double minimumDistance = 100.0; //Minimum distance
	double maximumDistance = 150.0; //Maximum distance
	char ssid_str[20]; 			//SSID string
	sprintf(ssid_str, "%s", "data");

	std::string wifiType = "ns3::SpectrumWifiPhy";
	std::string errorModelType = "ns3::NistErrorRateModel";
	std::string phyMode ("DsssRate2Mbps");

	std::cout << "wifiType: " << wifiType << " distance: " << distance	<< "m; sent: 1000 TxPower: 1 dBm (1.3 mW)" << std::endl;

	//the node 0 is the sink
	NodeContainer wifiNodes;
	wifiNodes.Create(nNodes);

	//Bug 2460: CcaMode1Threshold default should be set to -62 dBm when using Spectrum
	Config::SetDefault("ns3::WifiPhy::CcaMode1Threshold", DoubleValue(-62.0));
	// disable fragmentation for frames below 2200 bytes
	Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
	// turn off RTS/CTS for frames below 2200 bytes
	Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
	// Fix non-unicast data rate to be the same as that of unicast
	Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));

	Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
	Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
	Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
	spectrumChannel->AddPropagationLossModel(lossModel);
	spectrumChannel->SetPropagationDelayModel(delayModel);

	SpectrumWifiPhyCustomHelper spectrumPhy = SpectrumWifiPhyCustomHelper::Default();
	spectrumPhy.SetChannel(spectrumChannel);
	spectrumPhy.SetErrorRateModel(errorModelType);
	spectrumPhy.Set("Frequency", UintegerValue(5180));
	spectrumPhy.Set("TxPowerStart", DoubleValue(22)); // dBm  (1.26 mW)
	spectrumPhy.Set("TxPowerEnd", DoubleValue(22));
	spectrumPhy.Set("ShortGuardEnabled", BooleanValue(false));
	spectrumPhy.Set("ChannelWidth", UintegerValue(20));

	WifiHelper wifi;
	wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
	wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue (phyMode), "ControlMode", StringValue (phyMode));

	/*
	 * Create WifiMac for each wifi node:
	 * Create a mac helper in a default working state. i.e., this is an adhoc
	 * mac by default
	 */
	NqosWifiMacHelper mac;
	mac = NqosWifiMacHelper::Default();
	mac.SetType ("ns3::AdhocWifiMac", "Ssid", SsidValue(Ssid(ssid_str)));
	//WifiMacHelper mac;
	//mac.SetType ("ns3::AdhocWifiMac");

	//number of radios
	NetDeviceContainer wifiDevice[nRadios];
	for (int i = 0; i < nRadios; i++) {
		wifiDevice[i] = wifi.Install(spectrumPhy, mac, wifiNodes);
	}

	Ptr<RandomRectanglePositionAllocatorMinMax> pa = new RandomRectanglePositionAllocatorMinMax(sizeArea, quadrantDivision);
	pa->SetQuadrants(quadrantI, quadrantJ);
	MyMobilityHelper mobility;
	mobility.SetPositionAllocator(pa);
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.SetNNodes(nNodes);
	mobility.SetMinDist(minimumDistance);
	mobility.SetMaxDist(maximumDistance);
	mobility.Install(wifiNodes);

	if (!pa->verifyAllocation(mobility,nNodes,minimumDistance,maximumDistance)) {
		std::cout << "verify Allocation not correct!";
		exit(1);
	}

	/*
	* Create stack protocols:
	*/
	InternetStackHelper stack;
	Ipv4JointRoutingHelper ipv4RoutingHelper;
	Ipv4ListRoutingHelper list;
	list.Add(ipv4RoutingHelper, 0);
	stack.SetRoutingHelper(list);
	stack.Install(wifiNodes);


	Ipv4InterfaceContainer wifiNodeInterface;
	Ipv4AddressHelper address;
	address.SetBase("10.1.1.0", "255.255.255.0");
	for (int i = 0; i < nRadios; i++) {
		if (i == 0) {
			wifiNodeInterface = address.Assign(wifiDevice[i]);
		} else {
			address.Assign(wifiDevice[i]);
		}
	}

	graph = new element::Graph(mobility.GetMatrix(), nNodes);
	graph->showAdjacencyList();
	/*
	 * Create RALL
	 */
	//algorithm::Routing * rall = new algorithm::RALL(0.5, 0.5, graph);
	//std::cout << "RALL" << std::endl;

	/*
	* Create LBR
	*/
	/*algorithm::Routing * lbr = new algorithm::BPR(nNodes, graph->E, 2.5, graph->listEdges);
	lbr->setGraph(graph);
	lbr->getGraph()->U = graph->U;
	std::cout << "LBR-BPR" << std::endl;*/

	/*
	 * Create CRAA
	 */
	std::set<CRAAChannel> channels;
	for (int i = 1; i <= nChannels; i++) {
		channels.insert(i);
	}
	std::map< element::Edge, std::map< element::Edge, double> > per;
	//int nChannelChanges = graph->U.size();
	//int nChannelChanges = nRadios;
	//algorithm::CRAA * craa = new algorithm::CRAA(rall, nChannelChanges, 0.0, channels, nRadios, nFlows, per);
	//craa->RunInitial();
	std::cout << "CRAA" << std::endl;


	/*
	 * SET INITIAL CHANNEL
	 * */
	//int *channelsToRadios = set_channels(craa, wifiNodes, nNodes);
	//craa->SetChannelsToRadios(channelsToRadios);
	std::cout << "CHANNEL TO RADIOS" << std::endl;

	/*
	* Create Ipv4JointRouting
	*/
	std::map< uint32_t, Ipv4Address > nodesAddress;
	for (int i = 0; i < nNodes; i++) {
		Ptr<ns3::Node> node = wifiNodes.Get(i);
		nodesAddress[node->GetId()] = wifiNodeInterface.GetAddress(i);
	}

	int nodeId = 0;
	for (int i = 0; i < nNodes; i++, nodeId++) {
		Ptr<JointRouting> jointRouting = ipv4RoutingHelper.GetJointRouting(wifiNodes.Get(i)->GetObject<Ipv4>());
		//jointRouting->setRouting(rall);
		//jointRouting->setRcaa(craa);
		jointRouting->setNodesAddress(nodesAddress);
		jointRouting->setNodeId(nodeId);
	}


	/*
	 * Generate random flows
	 */
	std::cout << "Flows:" << std::endl;
	int I = 0;
	std::vector<element::Flow> flows;
	srand(seed);
	for (int i = 0; i < nFlows; i++) {
		int n = (rand() % nNodes) + 1;
		flows.push_back(element::Flow(n, I, 49153 + i, i + 1));
		std::cout << "src " << flows[i].m_sourceSink.first << " sink "
				<< flows[i].m_sourceSink.second << " src-port "
				<< flows[i].m_sourcePort << " sink-port " << flows[i].m_sinkPort
				<< std::endl;
	}
	std::cout << std::endl;


	/*
	* APP.
	*/
	std::cout << "Apps:" << std::endl;


	//calibra o algoritmo RALL com os valore da qualidade para o enlaces:

	Address sinkAddress (InetSocketAddress (wifiNodeInterface.GetAddress(0), 80));
	Address BroadcastAddress (InetSocketAddress (Ipv4Address::GetBroadcast(), 80));
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	int qtdPkts = 50;

	Ptr<Socket> recvSink = Socket::CreateSocket(wifiNodes.Get(0), tid);
	InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);
	recvSink->Bind(local);
	recvSink->SetRecvCallback(MakeCallback (&ReceiveTrafficGrubbiness));

	std::vector<Ptr<Socket>> sendSockets;
	std::vector<Ptr<Socket>> recvSockets;
	for (int t=0; t < nNodes-1; t++) {
		double time = (rand() % 26) / 10.0;

		sendSockets.push_back(Socket::CreateSocket(wifiNodes.Get(t+1), tid));
		recvSockets.push_back(Socket::CreateSocket(wifiNodes.Get(t+1), tid));

		sendSockets[t]->Bind();
		sendSockets[t]->Connect(BroadcastAddress);
		sendSockets[t]->SetAllowBroadcast(true);
		sendSockets[t]->ShutdownRecv();
		Simulator::ScheduleWithContext(sendSockets[t]->GetNode()->GetId(),
										Seconds(5+time), &GenerateTraffic,
										sendSockets[t], 1024, qtdPkts, interPacketInterval);

		recvSockets[t]->Bind(local);
		recvSockets[t]->SetRecvCallback(MakeCallback (&ReceiveTrafficGrubbiness));
		recvSockets[t]->ShutdownSend();
	}

	//addInterferingNodes(spectrumChannel);

	PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 80));
	ApplicationContainer packetSink = packetSinkHelper.Install(wifiNodes.Get(0));
	packetSink.Start(Seconds(120.0));
	packetSink.Stop(Seconds(simulationTime));

	PacketSinkHelper packetSinkHelper_8080("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 8080));
	ApplicationContainer packetSink_8080 = packetSinkHelper_8080.Install(wifiNodes.Get(0));
	packetSink_8080.Start(Seconds(120.0));
	packetSink_8080.Stop(Seconds(simulationTime));
	Address sinkAddress_8080 (InetSocketAddress (wifiNodeInterface.GetAddress(0), 8080));

	LaboraAppHelper laboraAppHelper("ns3::UdpSocketFactory", sinkAddress, LaboraAppHelper::Class_3);
	laboraAppHelper.SetAttribute("MaxPackets", UintegerValue(100));
	laboraAppHelper.SetAttribute("DataRate", StringValue("1Mbps"));
	//laboraAppHelper.SetAttribute("DataRate", StringValue("200kb/s"));
	//laboraAppHelper.SetAttribute("PacketSize", UintegerValue(1024));

	LaboraAppHelper laboraAppHelper_8080("ns3::UdpSocketFactory", sinkAddress_8080, LaboraAppHelper::Class_1);
	laboraAppHelper_8080.SetAttribute("MaxPackets", UintegerValue(100));


	ApplicationContainer source;
	for (int i = 1; i < nNodes; i++) {
		source.Add(laboraAppHelper.Install(wifiNodes.Get(i)));
	}
	source.Add(laboraAppHelper_8080.Install(wifiNodes.Get(2)));

	source.Start (Seconds (130.1));
	source.Stop (Seconds (simulationTime));

	// 8. Install FlowMonitor on all nodes
	FlowMonitorHelper flowmon;
	Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

	Simulator::Stop(Seconds(simulationTime + 1));
	Simulator::Run();


	// 10. Print per flow statistics
	//double Throughput=0.0;
	monitor->CheckForLostPackets();
	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
	FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();
	for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin(); i != stats.end(); ++i) {
		Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
		std::cout << "\nFlow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << "["<< t.sourcePort << "," << t.destinationPort <<"])\n";
		std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
		std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
		std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / 9.0 / 1000 / 1000 << " Mbps\n";
		std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
		std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
		std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / 9.0 / 1000 / 1000 << " Mbps\n";
		if(i->second.rxPackets > 0)
			std::cout << "  AVG Delay: " << i->second.delaySum.GetMilliSeconds() / i->second.rxPackets << " ms\n";
		else
			std::cout << "  AVG Delay: -- ms\n";
	}



	Simulator::Destroy();
	std::cout << "\n\n\nfim!\n";
	return 0;
}
