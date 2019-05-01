/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "labora-application-helper.h"

#include "ns3/inet-socket-address.h"
#include "ns3/packet-socket-address.h"
#include "ns3/string.h"
#include "ns3/data-rate.h"
#include "ns3/uinteger.h"
#include "ns3/names.h"
#include "ns3/random-variable-stream.h"
#include "ns3/labora-application.h"

namespace ns3 {

LaboraAppHelper::LaboraAppHelper(std::string protocol, Address address) {
	m_factory.SetTypeId("ns3::LaboraApplication");
	m_factory.Set("Protocol", StringValue(protocol));
	m_factory.Set("Remote", AddressValue(address));
}

LaboraAppHelper::LaboraAppHelper(std::string protocol, Address address, LaboraAppHelper::Class classType) {
	m_factory.SetTypeId("ns3::LaboraApplication");
	m_factory.Set("Protocol", StringValue(protocol));
	m_factory.Set("Remote", AddressValue(address));
	m_factory.Set("MaxPackets", UintegerValue(250));
	//m_factory.Set("MaxPackets", UintegerValue(4294967295u));
	m_factory.Set("classApp", UintegerValue(static_cast<uint32_t>(classType)));
	switch (classType) {
		case Class_1 :			
			m_factory.Set("DataRate", StringValue("264kbps"));
			m_factory.Set("PacketSize", UintegerValue(80));
			m_factory.Set("local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), 41)));
		break;
		case Class_2 :
			m_factory.Set("DataRate", StringValue("1024kbps"));
			m_factory.Set("PacketSize", UintegerValue(80));
			m_factory.Set("local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), 42)));
		break;
		case Class_3 :			
			m_factory.Set("DataRate", StringValue("10bps"));
			m_factory.Set("PacketSize", UintegerValue(80));
			m_factory.Set("local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), 43)));
		break;
		case Class_4 :		
			m_factory.Set("DataRate", StringValue("32kbps"));
			m_factory.Set("PacketSize", UintegerValue(80));
			m_factory.Set("local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), 44)));			
			break;
		case Class_5 :		
			m_factory.Set("DataRate", StringValue("3kbps"));
			m_factory.Set("PacketSize", UintegerValue(80));
			m_factory.Set("local", AddressValue(InetSocketAddress(Ipv4Address::GetAny(), 45)));
		break;
	}

}

void LaboraAppHelper::SetAttribute(std::string name, const AttributeValue &value) {
	m_factory.Set(name, value);
}

ApplicationContainer LaboraAppHelper::Install(Ptr<Node> node) const {
	return ApplicationContainer(InstallPriv(node));
}

ApplicationContainer LaboraAppHelper::Install(std::string nodeName) const {
	Ptr<Node> node = Names::Find<Node>(nodeName);
	return ApplicationContainer(InstallPriv(node));
}

ApplicationContainer LaboraAppHelper::Install(NodeContainer c) const {
	ApplicationContainer apps;
	for (NodeContainer::Iterator i = c.Begin(); i != c.End(); ++i) {
		apps.Add(InstallPriv(*i));
	}
	return apps;
}

Ptr<Application> LaboraAppHelper::InstallPriv(Ptr<Node> node) const {
	Ptr<Application> app = m_factory.Create<Application>();
	node->AddApplication(app);
	return app;
}

int64_t LaboraAppHelper::AssignStreams(NodeContainer c, int64_t stream) {
	int64_t currentStream = stream;
	Ptr<Node> node;
	for (NodeContainer::Iterator i = c.Begin(); i != c.End(); ++i) {
		node = (*i);
		for (uint32_t j = 0; j < node->GetNApplications(); j++) {
			Ptr<LaboraApplication> laboraApp = DynamicCast<LaboraApplication>(node->GetApplication(j));
			if (laboraApp) {
				currentStream += laboraApp->AssignStreams(currentStream);
			}
		}
	}
	return (currentStream - stream);
}

void LaboraAppHelper::SetConstantRate(DataRate dataRate, uint32_t packetSize) {
	m_factory.Set("OnTime",	StringValue("ns3::ConstantRandomVariable[Constant=1000]"));
	m_factory.Set("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
	m_factory.Set("DataRate", DataRateValue(dataRate));
	m_factory.Set("PacketSize", UintegerValue(packetSize));
}

} // namespace ns3
