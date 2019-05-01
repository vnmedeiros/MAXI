
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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TESTE_PSD");

Ptr<SpectrumModel> SpectrumModelWifi_MHz;
Ptr<SpectrumModel> SpectrumModelWifi_MHz2;
Ptr<SpectrumModel> SpectrumModelWifi_MHz3;

class static_SpectrumModelWifi_MHz_initializer {
public:
	static_SpectrumModelWifi_MHz_initializer(double bandMHz) {
		//see values the Frequency/ChannelWidth in wifi-phy.cc line 68
		BandInfo bandInfo;
		bandInfo.fc = bandMHz;
		bandInfo.fl = bandMHz - 20e6;
		bandInfo.fh = bandMHz + 20e6;
		
		BandInfo bandInfo2;
		bandInfo2.fc = 5600e6;
		bandInfo2.fl = 5600e6 - 20e6;
		bandInfo2.fh = 5600e6 + 20e6;
        
        BandInfo bandInfo3;
		bandInfo3.fc = 5700e6;
		bandInfo3.fl = 5700e6 - 20e6;
		bandInfo3.fh = 5700e6 + 20e6;
		
		Bands bands;
		bands.push_back(bandInfo);
		//bands.push_back(bandInfo2);

		Bands bands2;
		bands2.push_back(bandInfo2);
        
        Bands bands3;
		bands3.push_back(bandInfo3);
		
		SpectrumModelWifi_MHz = Create<SpectrumModel>(bands);
		SpectrumModelWifi_MHz2 = Create<SpectrumModel>(bands2);
        SpectrumModelWifi_MHz3 = Create<SpectrumModel>(bands3);
		
	}
} //static_SpectrumModelWifi_MHz_initializer_instance(/*2412e6*/5180e6);
static_SpectrumModelWifi_MHz_initializer_instance(/*2412e6*/5480e6);
//36 – 5180e6
//60 – 5300e6
//112 – 5560e6


	


static void GenerateTrafficForSocket(Ptr<Socket> socket, uint32_t sizePackages, uint32_t packagesCount, Time interval) {
	if (packagesCount > 0) {
		Ptr<Packet> pkt = Create<Packet>(sizePackages);
		socket->Send(pkt);
		//std::cout << "GenerateTrafficForSocket\n";
		Simulator::Schedule(interval, &GenerateTrafficForSocket, socket, sizePackages, packagesCount - 1, interval);
	} else {
		socket->Close();
	}
}
int cc = 0;
static void ReceiveTrafficForSocket(Ptr<Socket> socket) {
	Ptr<Packet> packet;
	while ((packet = socket->Recv())) {
		std::cout << ++cc << " ReceiveTrafficForSocket\n";
	}
}

int main(int argc, char *argv[]) {
	
	double 		simulationTime = 11.5;
		
	CommandLine cmd;
	cmd.AddValue("simulationTime", 	"Simulation time in seconds",	simulationTime);
	cmd.Parse(argc, argv);	
	
	NodeContainer nodesWifi;
	NodeContainer waveformGeneratorNodes;
	NodeContainer spectrumAnalyzerNodes;
	NodeContainer allNodes;

	nodesWifi.Create (4);
	waveformGeneratorNodes.Create (3);
	spectrumAnalyzerNodes.Create (1);
	allNodes.Add (nodesWifi);
	allNodes.Add (waveformGeneratorNodes);
	allNodes.Add (spectrumAnalyzerNodes);
		
	MobilityHelper mobility;
	Ptr<ListPositionAllocator> nodePositionList = CreateObject<ListPositionAllocator> ();
	nodePositionList->Add (Vector (0.0, 0.0, 0.0));  	// RX-1 node
	nodePositionList->Add (Vector (0.0,144.0, 0.0));  	// TX-2 node
	nodePositionList->Add (Vector (0.0,20.0, 0.0));  	// TX-3 node
	nodePositionList->Add (Vector (0.0, 0.0, 0.0));  	// TX-4 node
	nodePositionList->Add (Vector (0.0,20.0, 0.0)); 	// Wave Oven
    nodePositionList->Add (Vector (0.0,20.0, 0.0)); 	// Wave Oven
    nodePositionList->Add (Vector (0.0,20.0, 0.0)); 	// Wave Oven
	nodePositionList->Add (Vector (0.0, 0.0, 0.0));  	// Spectrum Analyzer
	mobility.SetPositionAllocator (nodePositionList);
	mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobility.Install (allNodes);

	Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
	Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel>();
	lossModel->SetPathLossExponent (2.7);
	lossModel->SetReference (1, 46.6777);

	SpectrumWifiPhyHelper spectrumPhy = SpectrumWifiPhyHelper::Default();
    Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
	spectrumChannel->AddPropagationLossModel(lossModel);
	spectrumChannel->SetPropagationDelayModel(delayModel);

	spectrumPhy.SetPcapDataLinkType(SpectrumWifiPhyHelper::DLT_IEEE802_11_RADIO);
	spectrumPhy.SetChannel(spectrumChannel);
	spectrumPhy.Set ("ShortGuardEnabled", BooleanValue(true));
	spectrumPhy.SetErrorRateModel("ns3::NistErrorRateModel");
	spectrumPhy.Set ("Frequency", UintegerValue(/*2412*/5180));
    spectrumPhy.Set ("ChannelNumber", UintegerValue (36));

	spectrumPhy.Set ("TxGain", 		 	DoubleValue (0) ); 	// set it to zero; otherwise, gain will be added
	spectrumPhy.Set ("RxGain", 			DoubleValue (0) ); 	// set it to zero; otherwise, gain will be added
	spectrumPhy.Set ("TxPowerStart", 	DoubleValue (30)); 	//changes default TxPower from 16.0206dBm to 10dBm //22 36mb - 20nós
	spectrumPhy.Set ("TxPowerEnd", 		DoubleValue (30));  //changes default TxPower from 16.0206dBm to 10dBm //22 36mb - 20nós
	//--------------Energy Threshold and Cca Threshold Configuration------------------------------------------------------------------
	spectrumPhy.Set ("EnergyDetectionThreshold", DoubleValue(-78.0)); //dmax = 150m
	spectrumPhy.Set ("CcaMode1Threshold", 		 DoubleValue(-62.0)); //dmax=200m
	// Set MIMO capabilities
    spectrumPhy.Set ("TxAntennas", UintegerValue (2));
    spectrumPhy.Set ("RxAntennas", UintegerValue (2));

	WifiHelper wifi;
	wifi.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);
	//wifi.SetStandard(WIFI_PHY_STANDARD_80211n_2_4GHZ);

	Ssid ssid = Ssid("NS3-80211n");
	StringValue DataRate;
	DataRate = StringValue("HtMcs4");

	wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
			"DataMode", 		DataRate, 				//
			"ControlMode", 		DataRate, 				//
			"NonUnicastMode", 	DataRate, 				//Wifi mode used for non-unicast transmissions.
			"MaxSsrc", 			UintegerValue(0), 		//The maximum number of retransmission attempts for an RTS.
			"MaxSlrc", 			UintegerValue(7), 		//The maximum number of retransmission attempts for a DATA packet.
			"RtsCtsThreshold", 	UintegerValue(65535), 	//If the size of the data packet + LLC header + MAC header + FCS trailer is bigger than this value, we use an RTS/CTS handshake before sending the data.
			"FragmentationThreshold", StringValue("10000"));//2346 Disable fragmentation for frames below 10000 bytes.

	Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (DataRate));
	Config::SetDefault("ns3::WifiMacQueue::MaxDelay", TimeValue(MilliSeconds(200)));
	Config::SetDefault("ns3::ArpCache::PendingQueueSize", UintegerValue (800));
	
	HtWifiMacHelper mac = HtWifiMacHelper::Default();	
	mac.SetType ("ns3::AdhocWifiMac", "Ssid", SsidValue(ssid));
	
	//install radios in nodes:
	NetDeviceContainer wifiDevice;
	wifiDevice = wifi.Install(spectrumPhy, mac, nodesWifi);
	
	// Channel width must be set *after* installation because the attribute is overwritten by the ConfigureStandard method ()
	Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue(40));
	
	//nodesWifi.Get(0)->GetDevice(0)->GetObject<WifiNetDevice>()->GetPhy()->GetObject<SpectrumWifiPhy>()->SetChannelNumber(uint32_t(112));
    nodesWifi.Get(1)->GetDevice(0)->GetObject<WifiNetDevice>()->GetPhy()->GetObject<SpectrumWifiPhy>()->SetChannelNumber(uint32_t(40));
	nodesWifi.Get(2)->GetDevice(0)->GetObject<WifiNetDevice>()->GetPhy()->GetObject<SpectrumWifiPhy>()->SetChannelNumber(uint32_t(60));

	//Create stack protocols:
	InternetStackHelper stack;
	Ipv4ListRoutingHelper list;
	Ipv4JointRoutingHelper ipv4RoutingHelper;
	list.Add(ipv4RoutingHelper, 0);
	stack.SetRoutingHelper(list);
	stack.Install(nodesWifi);

	//address:
	Ipv4AddressHelper address;
	Ipv4InterfaceContainer wifiNodeInterface;
	address.SetBase("10.1.1.0", "255.255.255.255");	
	wifiNodeInterface = address.Assign(wifiDevice);
	
	
	/////////////////////////////////
	// APP
	/////////////////////////////////
	Time 	 packetInterval = Seconds(0.25);
	uint16_t quantityPackages = 40;
	uint16_t sizePackages = 2048;    
	
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Address BroadcastAddress (InetSocketAddress(Ipv4Address::GetBroadcast(), 80));
	InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);

	
    Ptr<Socket> recvSockets = Socket::CreateSocket(nodesWifi.Get(0), tid);
	recvSockets->Bind(local);
	recvSockets->ShutdownSend();
	recvSockets->SetRecvCallback(MakeCallback (&ReceiveTrafficForSocket));
    
    //Ptr<Socket> sendSockets0 = Socket::CreateSocket(nodesWifi.Get(0), tid);
	//sendSockets0->Bind();
	//sendSockets0->Connect(BroadcastAddress);
	//sendSockets0->SetAllowBroadcast(true);
	//sendSockets0->ShutdownRecv();
	
	Ptr<Socket> sendSockets1 = Socket::CreateSocket(nodesWifi.Get(1), tid);	
	sendSockets1->Bind();
	sendSockets1->Connect(BroadcastAddress);
	sendSockets1->SetAllowBroadcast(true);
	sendSockets1->ShutdownRecv();
	
	Ptr<Socket> sendSockets2 = Socket::CreateSocket(nodesWifi.Get(2), tid);	
	sendSockets2->Bind();
	sendSockets2->Connect(BroadcastAddress);
	sendSockets2->SetAllowBroadcast(true);
	sendSockets2->ShutdownRecv();	
	
	//Simulator::ScheduleWithContext(sendSockets0->GetNode()->GetId(),	Seconds(2.0), 
	//							   &GenerateTrafficForSocket, sendSockets0, sizePackages, 
	//							   quantityPackages, packetInterval);	
								   
	Simulator::ScheduleWithContext(sendSockets1->GetNode()->GetId(),	Seconds(2.0), 
								   &GenerateTrafficForSocket, sendSockets1, sizePackages, 
								   quantityPackages, packetInterval);	
								   
	//Simulator::ScheduleWithContext(sendSockets2->GetNode()->GetId(),	Seconds(2.0), 
	//							   &GenerateTrafficForSocket, sendSockets2, sizePackages, 
	//							   quantityPackages, packetInterval);
	
		
	/////////////////////////////////
	// Configure spectrum analyzer
	/////////////////////////////////
	
    std::vector<double> freqs;
    /*for (int i = 0; i < 800; ++i) {
        freqs.push_back ((i + 5035) * 1e6);
    }*/
    
    freqs.push_back ((5200) * 1e6); //no-1
    freqs.push_back ((5300) * 1e6); //no-2
    freqs.push_back ((5480) * 1e6); //wave-0.001
    freqs.push_back ((5600) * 1e6); //wave-0.010
    freqs.push_back ((5700) * 1e6); //wave-0.200*/
    
    Ptr<SpectrumModel> SpectrumModelIsm5000MhzRes1Mhz;
    SpectrumModelIsm5000MhzRes1Mhz = Create<SpectrumModel> (freqs);
    
	
	SpectrumAnalyzerHelper spectrumAnalyzerHelper;
	spectrumAnalyzerHelper.SetChannel (spectrumChannel);
	//spectrumAnalyzerHelper.SetRxSpectrumModel (SpectrumModelIsm2400MhzRes1Mhz);
	spectrumAnalyzerHelper.SetRxSpectrumModel (SpectrumModelIsm5000MhzRes1Mhz);
	spectrumAnalyzerHelper.SetPhyAttribute ("Resolution", TimeValue (MilliSeconds (100)));
	//spectrumAnalyzerHelper.SetPhyAttribute ("NoisePowerSpectralDensity", DoubleValue (1e-15));  // -120 dBm/Hz
	spectrumAnalyzerHelper.EnableAsciiAll ("spectrum-analyzer-output");
	NetDeviceContainer spectrumAnalyzerDevices = spectrumAnalyzerHelper.Install (spectrumAnalyzerNodes);

  /*
    you can get a nice plot of the output of SpectrumAnalyzer with this gnuplot script:

    unset surface
    set pm3d at s 
    set palette
    set key off
    set view 50,50
    set xlabel "time (ms)"
    set ylabel "freq (MHz)"
    set zlabel "PSD (dBW/Hz)" offset 15,0,0
    splot "./spectrum-analyzer-output-3-0.tr" using ($1*1000.0):($2/1e6):(10*log10($3))
  */
	
	std::cout << "Start...\n";
	Simulator::Stop(Seconds(simulationTime+0.01));
	Simulator::Run();
	Simulator::Destroy();	
	return 0;
}
