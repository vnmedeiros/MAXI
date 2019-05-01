/*
 * teste.cc
 *
 *  Created on: Oct 20, 2016
 *      Author: vinicius
 */

#include <sstream>
#include <iomanip>
#include <vector>

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

int nPkt = 800;
std::vector<InfoTag> pktReceived(nPkt, InfoTag());
std::vector<double> pktDist(nPkt, -1);

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

static void ReceivePacket(Ptr<Socket> socket) {
	Address addr;
	std::ostringstream oss;
	Ptr<Packet> packet;
	while ((packet = socket->Recv())) {
		socket->GetSockName(addr);
		InfoTag infoTag;
		packet->RemovePacketTag(infoTag);
		pktReceived[infoTag.GetId() -1].SetId(infoTag.GetId());
		pktReceived[infoTag.GetId() -1].SetSignal(infoTag.GetSignal());
		pktReceived[infoTag.GetId() -1].SetSnr(infoTag.GetSnr());
		pktReceived[infoTag.GetId() -1].SetNoise(infoTag.GetNoise());
	}
}

static void GenerateTraffic(Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval, Ptr<ns3::Node> disNode) {
	if (pktCount > 0) {

		Ptr<MobilityModel> position = socket->GetNode()->GetObject<MobilityModel>();
		Ptr<MobilityModel> nodePosition = disNode->GetObject<MobilityModel>();
		NS_ASSERT (position != 0 && nodePosition != 0);
		Ptr<Packet> pkt = Create<Packet>(pktSize);
		InfoTag infoTag(pktCount, socket->GetNode()->GetId());
		pkt->AddPacketTag(infoTag);

		pktDist[pktCount-1] = position->GetDistanceFrom(nodePosition);

		socket->Send(pkt);
		Simulator::Schedule(pktInterval, &GenerateTraffic, socket, pktSize, pktCount - 1, pktInterval, disNode);
	} else {
		socket->Close();
	}
}

NS_LOG_COMPONENT_DEFINE("Calculate_Threshold");

int main(int argc, char *argv[]) {
	//double distance = 50;
	double simulationTime = 400; //seconds
	double interval = 0.5; // seconds
	Time interPacketInterval = Seconds(interval);

	////////////////////////////////////////////////
	char ssid_str[20]; //SSID string
	sprintf(ssid_str, "%s", "data");

	std::string wifiType = "ns3::SpectrumWifiPhy";
	std::string errorModelType = "ns3::NistErrorRateModel";
	std::string phyMode ("DsssRate1Mbps");

	//the node 0 is the sink

	NodeContainer staticNode;
	NodeContainer mobilyNode;
	staticNode.Create(1);
	mobilyNode.Create(1);
	NodeContainer allNodes = NodeContainer (staticNode, mobilyNode);


	// Bug 2460: CcaMode1Threshold default should be set to -62 dBm when using Spectrum and
	// disable fragmentation for frames below 2200 bytes and
	// turn off RTS/CTS for frames below 2200 bytes and
	// Fix non-unicast data rate to be the same as that of unicast.
	Config::SetDefault("ns3::WifiPhy::CcaMode1Threshold", DoubleValue(-62.0));
	Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
	Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
	Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));

	Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
	Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
	//Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel>();
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

	//number of radios????
	NetDeviceContainer wifiDevice;
	wifiDevice = wifi.Install(spectrumPhy, mac, allNodes);

	// mobility.
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
	positionAlloc->Add(Vector(0.0, 0.0, 0.0));
	positionAlloc->Add(Vector(50.0, 0.0, 0.0));
	//positionAlloc->Add(Vector(25.0, 0.0, 0.0));
	MobilityHelper staticMobility;
	staticMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	staticMobility.SetPositionAllocator (positionAlloc);
	//staticMobility.Install(staticNode);
	staticMobility.Install(allNodes);


	/*Ptr<ListPositionAllocator> stationPositionAlloc = CreateObject<ListPositionAllocator>();
	stationPositionAlloc->Add(Vector(0.0, 0.0, 0.0));		//it is the starting position
	MobilityHelper nodeMobilityHelper;
	nodeMobilityHelper.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
	nodeMobilityHelper.SetPositionAllocator(stationPositionAlloc);
	nodeMobilityHelper.Install(mobilyNode);
	(mobilyNode.Get(0)->GetObject<ConstantVelocityMobilityModel>())->SetVelocity(Vector(1.0, 0.0, 0.0));*/


	/*
	* Create stack protocols:
	*/
	InternetStackHelper stack;
	stack.Install(allNodes);

	Ipv4AddressHelper address;
	address.SetBase("10.1.1.0", "255.255.255.0");

	Ipv4InterfaceContainer wifiNodeInterface;
	wifiNodeInterface = address.Assign(wifiDevice);

	/*
	* APP.
	*/
	std::cout << "Apps:" << std::endl;

	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Address sinkAddress (InetSocketAddress (wifiNodeInterface.GetAddress(0), 80));

	Ptr<Socket> recvSink = Socket::CreateSocket(allNodes.Get(0), tid);
	InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);
	recvSink->Bind(local);
	recvSink->SetRecvCallback(MakeCallback (&ReceivePacket));

	Ptr<Socket> sendSocket = Socket::CreateSocket (allNodes.Get(1), tid);
	sendSocket->Bind();
	sendSocket->ShutdownRecv();
	sendSocket->Connect(sinkAddress);


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

	Simulator::ScheduleWithContext (sendSocket->GetNode()->GetId(), Seconds(1), &GenerateTraffic,
										sendSocket, 1024, nPkt, interPacketInterval, staticNode.Get(0));
	std::cout << "Start...\n";
	Simulator::Stop(Seconds(simulationTime + 1));
	Simulator::Run();
	Simulator::Destroy();
	std::cout << "dist " << "snr " << "signal " << "noise " << std::endl;
	int r = 0;
	for(int i=nPkt-1; i >=0 ; i--) {
		if (pktReceived[i].GetId() != 0 ) {
			r++;
			std::cout << std::setw(5) << pktDist[i] << " " << std::setw(12) << pktReceived[i].GetSnr() << " " << std::setw(16) << pktReceived[i].GetSignal() << " "<< std::setw(12) << pktReceived[i].GetNoise() << std::endl;
		} else {
			std::cout << std::setw(5) << pktDist[i] << " " << std::setw(12) << "---" << std::setw(17) << "---" << std::setw(13) << "---" << std::endl;
		}
	}
	std::cout << "\n gerados: " << nPkt << " recebidos " << r << std::endl;


	std::cout << "\n\n\nfim!\n";
	return 0;
}
