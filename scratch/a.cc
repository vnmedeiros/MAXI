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
#include "ns3/stats-module.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("A");

static void changeChannel(Ptr<Node> node, uint32_t ch, Ptr<MultiModelSpectrumChannel> mmsc) {
	Ptr<WifiNetDevice> wnd = node->GetDevice(0)->GetObject<WifiNetDevice>();
	Ptr<SpectrumWifiPhy> wp = wnd->GetPhy()->GetObject<SpectrumWifiPhy>();
	wp->SetChannelNumber(ch);
	mmsc->AddRx(wp->GetSpectrumPhy());
}

static void GenerateTrafficForSocket(Ptr<Socket> socket, uint32_t sizePackages, uint32_t packagesCount, Time interval, TypeTag typeTag) {
	if (packagesCount > 0) {

		std::cout << "N:" << socket->GetBoundNetDevice()->GetObject<WifiNetDevice>()->GetPhy()->GetObject<WifiPhy>()->GetChannelNumber() <<std::endl;
		std::cout << "->ID: " << socket->GetNode()->GetId() << ", GenerateTrafficForSocket" << std::endl;
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

static void LaunchGenerateMapReceiveTraffic(Ptr<Socket> socket) {
	Ptr<Packet> packet;
	while ((packet = socket->Recv())) {
		InfoTag infoTag;
		if ( packet->PeekPacketTag(infoTag) ) {
			packet->RemovePacketTag(infoTag);
			std::cout << "<-ID: " << socket->GetNode()->GetId() << ", LaunchGenerateMapReceiveTraffic for ID: " << infoTag.GetSrc() << std::endl;

		}
	}
}

int main(int argc, char *argv[]) {

	int channelWidth = 20;
	double simulationTime = 300.0;

	NodeContainer wifiNodes;
	wifiNodes.Create(3);

	/*
	YansWifiPhyHelper phy = YansWifiPhyHelper::Default();

	YansWifiChannelHelper channel;
	channel.AddPropagationLoss("ns3::FriisPropagationLossModel");
	channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

	Ptr<YansWifiChannel> ch = channel.Create();
	phy.SetChannel(ch);
	phy.Set("ShortGuardEnabled", BooleanValue(false));

	Config::SetDefault("ns3::WifiPhy::EnergyDetectionThreshold", DoubleValue (-79.0)); 	//The energy of a received signal should be higher than this threshold (dbm) to allow the PHY layer to detect the signal.
	Config::SetDefault("ns3::WifiPhy::CcaMode1Threshold", DoubleValue (-62.0) ); 		//The energy of a received signal should be higher than this threshold (dbm) to allow the PHY layer to declare CCA BUSY state.
	Config::SetDefault("ns3::WifiPhy::TxGain", DoubleValue (0) ); 					//Transmission gain (dB).
	Config::SetDefault("ns3::WifiPhy::RxGain", DoubleValue (0) ); 					//Reception gain (dB).
	Config::SetDefault("ns3::WifiPhy::TxPowerLevels", UintegerValue (1) ); 			//Number of transmission power levels available between TxPowerStart and TxPowerEnd included.
	Config::SetDefault("ns3::WifiPhy::TxPowerEnd", DoubleValue (16.0) );  			//Maximum available transmission level (dbm).
	Config::SetDefault("ns3::WifiPhy::TxPowerStart", DoubleValue (16.0) ); 			//Minimum available transmission level (dbm).
	 */

	SpectrumWifiPhyHelper phy = SpectrumWifiPhyHelper::Default();
	//Ptr<SingleModelSpectrumChannel> spectrumChannel =  CreateObject<SingleModelSpectrumChannel> ();
	Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

	Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel>();;
	lossModel->SetPathLossExponent (2.7);
	lossModel->SetReference (1, 46.6777);
	spectrumChannel->AddPropagationLossModel(lossModel);
	Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
	spectrumChannel->SetPropagationDelayModel(delayModel);

	phy.SetChannel(spectrumChannel);
	phy.SetErrorRateModel("ns3::NistErrorRateModel");
	//spectrumPhy.Set("Frequency", UintegerValue(5180)); // channel 36 at 20 MHz
	phy.Set("Frequency", UintegerValue(2412)); // channel 1 at 20 MHz

	phy.Set("ShortGuardEnabled", BooleanValue(false));

	Config::SetDefault("ns3::WifiPhy::EnergyDetectionThreshold", DoubleValue (-78.0)); 	//The energy of a received signal should be higher than this threshold (dbm) to allow the PHY layer to detect the signal.
	Config::SetDefault("ns3::WifiPhy::CcaMode1Threshold", DoubleValue (-62.0) ); 		//The energy of a received signal should be higher than this threshold (dbm) to allow the PHY layer to declare CCA BUSY state.
	Config::SetDefault("ns3::WifiPhy::TxGain", DoubleValue (0) ); 					//Transmission gain (dB).
	Config::SetDefault("ns3::WifiPhy::RxGain", DoubleValue (0) ); 					//Reception gain (dB).
	Config::SetDefault("ns3::WifiPhy::TxPowerLevels", UintegerValue (1) ); 			//Number of transmission power levels available between TxPowerStart and TxPowerEnd included.
	Config::SetDefault("ns3::WifiPhy::TxPowerEnd", DoubleValue (26.0) );  			//Maximum available transmission level (dbm).
	Config::SetDefault("ns3::WifiPhy::TxPowerStart", DoubleValue (26.0) ); 			//Minimum available transmission level (dbm).

	WifiHelper wifi;
	//wifi.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);
	wifi.SetStandard(WIFI_PHY_STANDARD_80211n_2_4GHZ);
	Ssid ssid = Ssid("NS3-80211n");

	StringValue DataRate_;
	DataRate_ = StringValue("HtMcs1");

	wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
			"DataMode", 		DataRate_, 				//
			"ControlMode", 		DataRate_, 				//
			"NonUnicastMode", 	DataRate_, 				//Wifi mode used for non-unicast transmissions.
			"MaxSsrc", 			UintegerValue(1), 		//The maximum number of retransmission attempts for an RTS.
			"MaxSlrc", 			UintegerValue(1), 		//The maximum number of retransmission attempts for a DATA packet.
			"RtsCtsThreshold", 	UintegerValue(10000), 	//If the size of the data packet + LLC header + MAC header + FCS trailer is bigger than this value, we use an RTS/CTS handshake before sending the data.
			"FragmentationThreshold", StringValue("10000"));//Disable fragmentation for frames below 10000 bytes.

	//Create WifiMac for each wifi node:
	NqosWifiMacHelper mac;
	mac = NqosWifiMacHelper::Default();
	mac.SetType ("ns3::AdhocWifiMac", "Ssid", SsidValue(ssid));

	//install radios in nodes:
	NetDeviceContainer wifiDevice_1;
	NetDeviceContainer wifiDevice_2;

	wifiDevice_1 = wifi.Install(phy, mac, wifiNodes);
	wifiDevice_2 = wifi.Install(phy, mac, wifiNodes);

	Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue(channelWidth));

	//mobility.
	// mobility.
	MobilityHelper mobility;
	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

	positionAlloc->Add (Vector (0.0, 0.0, 0.0));
	positionAlloc->Add (Vector (50.0, 0.0, 0.0));
	positionAlloc->Add (Vector (0.0, 100.0, 0.0));
	mobility.SetPositionAllocator (positionAlloc);

	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.Install(wifiNodes);

	//Create stack protocols:
	InternetStackHelper stack;
	Ipv4ListRoutingHelper list;
	Ipv4JointRoutingHelper ipv4RoutingHelper;
	list.Add(ipv4RoutingHelper, 0);
	stack.SetRoutingHelper(list);
	stack.Install(wifiNodes);

	//address:
	Ipv4InterfaceContainer wifiNodeInterface;
	Ipv4AddressHelper address;
	address.SetBase("10.1.1.0", "255.255.255.255");

	wifiNodeInterface = address.Assign(wifiDevice_1);
	address.Assign(wifiDevice_2);


	//////////////////////////////////////////////////////////////////////////////////////////////
	Ptr<ns3::Node> node_A = wifiNodes.Get(0);
	Ptr<ns3::Node> node_B = wifiNodes.Get(1);
	Ptr<ns3::Node> node_C = wifiNodes.Get(2);
	//////////////////////////////////////////////////////////////////////////////////////////////
	Ptr<WifiNetDevice> A1 = node_A->GetDevice(0)->GetObject<WifiNetDevice>();
	Ptr<WifiPhy> wifiPhy_a1 = A1->GetPhy()->GetObject<WifiPhy>();
	wifiPhy_a1->SetChannelNumber(uint32_t(1));

	Ptr<WifiNetDevice> B1 = node_B->GetDevice(0)->GetObject<WifiNetDevice>();
	Ptr<WifiPhy> wifiPhy_b1 = B1->GetPhy()->GetObject<WifiPhy>();
	wifiPhy_b1->SetChannelNumber(uint32_t(1));

	Ptr<WifiNetDevice> C1 = node_C->GetDevice(0)->GetObject<WifiNetDevice>();
	Ptr<WifiPhy> wifiPhy_c1 = C1->GetPhy()->GetObject<WifiPhy>();
	wifiPhy_c1->SetChannelNumber(uint32_t(2));
	//////////////////////////////////////////////////////////////////////////////////////////////
	Ptr<WifiNetDevice> A2 = node_A->GetDevice(1)->GetObject<WifiNetDevice>();
	Ptr<WifiPhy> wifiPhy_a2 = A2->GetPhy()->GetObject<WifiPhy>();
	wifiPhy_a2->SetChannelNumber(uint32_t(11));

	Ptr<WifiNetDevice> B2 = node_B->GetDevice(1)->GetObject<WifiNetDevice>();
	Ptr<WifiPhy> wifiPhy_b2 = B2->GetPhy()->GetObject<WifiPhy>();
	wifiPhy_b2->SetChannelNumber(uint32_t(11));

	Ptr<WifiNetDevice> C2 = node_C->GetDevice(1)->GetObject<WifiNetDevice>();
	Ptr<WifiPhy> wifiPhy_c2 = C2->GetPhy()->GetObject<WifiPhy>();
	wifiPhy_c2->SetChannelNumber(uint32_t(11));
	//////////////////////////////////////////////////////////////////////////////////////////////
	std::cout << "NODE A RADIO 0 channel: " << wifiPhy_a1->GetChannelNumber() << " iif:" << node_A->GetDevice(0)->GetIfIndex() << " Address: " << node_A->GetObject<Ipv4>()->GetAddress(1,0).GetLocal() << std::endl;
	std::cout << "NODE A RADIO 1 channel: " << wifiPhy_a2->GetChannelNumber() << " iif:" << node_A->GetDevice(1)->GetIfIndex() << " Address: " << node_A->GetObject<Ipv4>()->GetAddress(2,0).GetLocal() << std::endl << std::endl;

	std::cout << "NODE B RADIO 0 channel: " << wifiPhy_b1->GetChannelNumber() << " iif:" << node_B->GetDevice(0)->GetIfIndex() << " Address: " << node_B->GetObject<Ipv4>()->GetAddress(1,0).GetLocal() << std::endl;
	std::cout << "NODE B RADIO 1 channel: " << wifiPhy_b2->GetChannelNumber() << " iif:" << node_B->GetDevice(1)->GetIfIndex() << " Address: " << node_B->GetObject<Ipv4>()->GetAddress(2,0).GetLocal() << std::endl;
	//////////////////////////////////////////////////////////////////////////////////////////////
	uint16_t quantityPackages = 1;
	uint16_t sizePackages = 1024;
	Time packetInterval = Seconds(1.0);
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

	Address BroadcastAddress (InetSocketAddress(Ipv4Address::GetBroadcast(), 80));
	InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);

	Ptr<Socket> recv_a1 = Socket::CreateSocket(node_A, tid);
	recv_a1->Bind(local);
	recv_a1->SetRecvCallback(MakeCallback (&LaunchGenerateMapReceiveTraffic));
	recv_a1->ShutdownSend();

	Ptr<Socket> send = Socket::CreateSocket(node_B, tid);
	send->Bind();
	send->BindToNetDevice(node_B->GetDevice(1));
	send->Connect(BroadcastAddress);
	send->SetAllowBroadcast(true);
	send->ShutdownRecv();

	Simulator::Schedule(Seconds(1.0), &changeChannel, node_C, 1, spectrumChannel);

	Simulator::Schedule(Seconds(2.0), &GenerateTrafficForSocket,
									send, sizePackages, quantityPackages,
									packetInterval, TypeTag::TAG_PRELOAD);

	//Install Animation
	AnimationInterface anim ("animationTESTE.xml"); // Mandatory
	//anim.EnableIpv4RouteTracking ("routingtable-wireless-RALL.xml", Seconds (0), Seconds (5), Seconds (0.25)); //Optional
	anim.EnablePacketMetadata (); // Optional


	std::cout << "Start...\n";
	Simulator::Stop(Seconds(simulationTime + 1));
	Simulator::Run();
	Simulator::Destroy();
	return 0;
}
