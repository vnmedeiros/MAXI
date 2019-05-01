/*
 * jointrouting.cc
 *
 *  Created on: Nov 9, 2016
 *      Author: vinicius
 */

#include "joint-routing.h"

#include <iomanip>
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/packet.h"
#include "ns3/node.h"
#include "ns3/simulator.h"
#include "ns3/ipv4-route.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/udp-header.h"
#include "ns3/tcp-header.h"

NS_LOG_COMPONENT_DEFINE("JointRouting");

namespace ns3 {

	JointRouting::JointRouting():m_ipv4(0) {
		// TODO Auto-generated constructor stub
	}

	JointRouting::~JointRouting() {
		// TODO Auto-generated destructor stub
	}

	TypeId JointRouting::GetTypeId(void) {
		static TypeId tid = TypeId("ns3::JointRouting").SetParent<Ipv4RoutingProtocol>().AddConstructor<JointRouting>();
		return tid;
	}

	Ptr<Ipv4Route> JointRouting::RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr) {
		NS_LOG_FUNCTION(this << p << header << oif << sockerr);

		Ipv4Address destination = header.GetDestination();
		Ptr<Ipv4Route> rtentry = 0;

		// Multicast goes here
		if (destination.IsMulticast()) {
			NS_LOG_LOGIC("RouteOutput()::Multicast destination");
		}

		int source = m_nodeId;
		int sink;
		int sourcePort = -1;
		int sinkPort = -1;

		Ptr<Packet> packetCopy = p->Copy(); // Make a copy of the packet
		//Ipv4Header iph;
		//packetCopy->RemoveHeader(iph);
		if(header.GetProtocol() == 6) {	//TCP
			TcpHeader tcph;
			packetCopy->RemoveHeader(tcph);
			sourcePort = tcph.GetSourcePort();
			sinkPort = tcph.GetDestinationPort();
		} else if (header.GetProtocol() == 17) { //UPD
			UdpHeader udph;
			if(packetCopy->PeekHeader(udph)) {
				packetCopy->RemoveHeader(udph);
				sourcePort = udph.GetSourcePort();
				sinkPort = udph.GetDestinationPort();
			}
		}

		InfoTag it;
		int classApp = -1;
		if(packetCopy->PeekPacketTag(it)) {
			classApp = it.GetClassApp();
			//std::cout << "Route Out Put: " << classApp << std::endl;
		}

		for (std::map< std::pair<int, int>, Ipv4Address>::iterator it = m_nodesAddress.begin(); it != m_nodesAddress.end(); ++it) {
			if (destination.IsEqual(it->second)) {
				std::pair<int, int> ic = it->first;
				NS_LOG_LOGIC(ic.first << " = " << it->second);
				sink = (int) ic.first;
			}
		}

		NS_LOG_LOGIC("source = " << source);
		NS_LOG_LOGIC("sink = " << sink);
		NS_LOG_LOGIC("ports = " << sourcePort << "->" << sinkPort);		

		path * bestPath;
		//bestPath = m_routing->takeRoute(element::Flow(source, sink, sourcePort, sinkPort, 2));
		//bestPath = m_routing->takeRoute(element::Flow(source, sink, sourcePort, sinkPort, 1));
		//bestPath = m_routing->takeRoute(element::Flow(source, sink, sourcePort, sinkPort, 0));
		bestPath = m_routing->takeRoute(element::Flow(source, sink, sourcePort, sinkPort, classApp));
		//int myints[] = {0,1,6,8,9};
		//path * bestPath = new path(myints, myints + sizeof(myints) / sizeof(int) );

		std::string path = "path: ";
		for (unsigned int l = 0; bestPath && l < bestPath->size(); l++) {
			char hop[10];
			sprintf(hop, "%d", bestPath->at(l));
			path += hop;
			path += "-";
		}
		NS_LOG_LOGIC(path);
		//if (source == 6)
		//std::cout << "s: " << source << " d: " << sink << " ports = " << sourcePort << "->" << sinkPort << " path: "  << path << std::endl;

		RoutingHeader jointRouteHeader(*bestPath);
		if (p == 0) {
			NS_LOG_LOGIC("p == 0");
			rtentry = Create<Ipv4Route>();
			rtentry->SetDestination(destination);
			int channel = m_ipv4->GetNetDevice(1)->GetObject<WifiPhy>()->GetChannelNumber();
			rtentry->SetSource(m_nodesAddress[std::make_pair(m_nodeId,channel)]);
			rtentry->SetOutputDevice(m_ipv4->GetNetDevice(1));
		} else {
			NS_LOG_LOGIC("p != 0");
			rtentry = LookupStatic(destination, &jointRouteHeader, oif);
			p->AddHeader(jointRouteHeader);
		}

		if (rtentry) {
			sockerr = Socket::ERROR_NOTERROR;
		} else {
			sockerr = Socket::ERROR_NOROUTETOHOST;
		}
		return rtentry;
	}

	bool JointRouting::RouteInput ( Ptr<const Packet> p, const Ipv4Header &header, Ptr<const NetDevice> idev,
									UnicastForwardCallback ucb, MulticastForwardCallback mcb,
									LocalDeliverCallback lcb, ErrorCallback ecb) {
		NS_LOG_FUNCTION(this << p << header << header.GetSource () << header.GetDestination () << idev << &ucb << &mcb << &lcb << &ecb);
		//std::cout << "SRC:"  << header.GetSource () << "DESR:" << header.GetDestination () << "modeID:" << m_nodeId << std::endl;
		Ptr<Packet> packet = p->Copy(); // Make a copy of the packet
		Ipv4Header iph;
		UdpHeader udph;
		RoutingHeader jointRouteHeader;
		packet->RemoveHeader (iph);
		packet->RemoveHeader (udph);
		packet->PeekHeader(jointRouteHeader);
		packet->AddHeader(udph);
		packet->AddHeader(iph);

		InfoTag infoTag;
		if ( packet->PeekPacketTag(infoTag) ) {
			//SEE ipv4-l3-protocol.cc line 1031
			//std::cout << "INFO-TAG src:" << infoTag.GetSrc() << " TO " << m_nodeId << " SNR: " << infoTag.GetSnr()
			//<< " Noise: " << infoTag.GetNoise()
			//<< " Signal: " << infoTag.GetSignal()
			//<< std::endl;
			//m_callbackUpdateLinkQuality(infoTag.GetSrc(), m_nodeId, infoTag.GetSnr());
		}


		NS_ASSERT(m_ipv4 != 0);
		// Check if input device supports IP
		NS_ASSERT(m_ipv4->GetInterfaceForDevice(idev) >= 0);
		uint32_t iif = m_ipv4->GetInterfaceForDevice(idev);

		// Multicast recognition; handle local delivery here
		if(header.GetDestination().IsMulticast()) {
			NS_LOG_LOGIC("Routing Multicast not implement!");
			NS_LOG_LOGIC("Multicast route not found");
			return false; // Let other routing protocols try to handle this
		}
		if (header.GetDestination().IsBroadcast()) {
			NS_LOG_LOGIC("For me (Ipv4Addr broadcast address)");
			std::cout << "For me (Ipv4Addr broadcast address)" << std::endl;
		}

		NS_LOG_LOGIC("Unicast destination");
		// TODO:  Configurable option to enable RFC 1222 Strong End System Model
		// Right now, we will be permissive and allow a source to send us
		// a packet to one of our other interface addresses; that is, the
		// destination unicast address does not match one of the iif addresses,
		// but we check our other interfaces.  This could be an option
		// (to remove the outer loop immediately below and just check iif).

		for (uint32_t j = 0; j < m_ipv4->GetNInterfaces(); j++) {
			for (uint32_t i = 0; i < m_ipv4->GetNAddresses(j); i++) {
				Ipv4InterfaceAddress iaddr = m_ipv4->GetAddress(j, i);
				Ipv4Address addr = iaddr.GetLocal();
				if (addr.IsEqual(header.GetDestination())) {
					if (j == iif) {
						std::cout << m_nodeId << " - For me (destination " << addr << " match)" << std::endl;
						NS_LOG_LOGIC("For me (destination " << addr << " match)");
					} else {
						std::cout << m_nodeId << " - For me (destination " << addr << " match) on another interface " << header.GetDestination()  << std::endl;
						NS_LOG_LOGIC("For me (destination " << addr << " match) on another interface " << header.GetDestination());
					}
					lcb(p, header, iif);
					return true;
				}
				if (header.GetDestination().IsEqual(iaddr.GetBroadcast())) {
					std::cout << "For me (interface broadcast address)" << std::endl;
					NS_LOG_LOGIC("For me (interface broadcast address)");
					lcb(p, header, iif);
					return true;
				}
				NS_LOG_LOGIC("Address "<< addr << " not a match");
			}
		}

		// Check if input device supports IP forwarding
		if (m_ipv4->IsForwarding(iif) == false) {
			NS_LOG_LOGIC("Forwarding disabled for this interface");
			ecb(p, header, Socket::ERROR_NOROUTETOHOST);
			return false;
		}

		Ptr<Ipv4Route> rtentry = LookupStatic(header.GetDestination(), &jointRouteHeader);

		if (rtentry != 0) {
			NS_LOG_LOGIC("Found unicast destination - calling unicast callback");
			ucb(rtentry, p, header);  // unicast forwarding callback
			return true;
		} else {
			NS_LOG_LOGIC("Did not find unicast destination- returning false");
			return false; // Let other routing protocols try to handle this
		}
	}

	Ptr<Ipv4Route> JointRouting::LookupStatic(Ipv4Address dest, RoutingHeader * jointRoute, Ptr<NetDevice> oif) {
		NS_LOG_FUNCTION(this << dest << " " << jointRoute << " " << oif);
		Ptr<Ipv4Route> rtentry = 0;
		std::vector<uint16_t> hops = jointRoute->GetHops();

		for (uint32_t i = 0; i < hops.size(); i++) {
			int hop = hops[i];
			if (hop == m_nodeId) { //hop corresponding to the node
				int m, n;
				if (m_nodeId < hops[i - 1]) {
					m = m_nodeId;
					n = hops[i - 1];
				} else {
					m = hops[i - 1];
					n = m_nodeId;
				}
				int destInterface = 1;
				element::Edge e = element::Edge(m, n, 1);
				//element::Edge e = element::Edge(n, m, 1);
				CRAAChannel channel = m_craa->m_allocatedChannels[e];
				for (int radio = 0; radio < m_craa->m_nRadios; radio++) {
					if (m_craa->m_channelsToRadios[m_nodeId * m_craa->m_nRadios	+ radio] == channel) {
						destInterface = radio + 1; //+1 because in m_ipv4 the first interfaces is the loopback
						break;
					}
				}

				rtentry = Create<Ipv4Route>();
				rtentry->SetDestination(dest);
				rtentry->SetSource(m_nodesAddress[std::make_pair(m_nodeId,channel)]);
				rtentry->SetGateway(m_nodesAddress[std::make_pair(hops[i - 1], channel)]);
				rtentry->SetOutputDevice(m_ipv4->GetNetDevice(destInterface));

				//std::cout << "Node = " << m_nodeId << " send using channel = " << channel << " to Address = " << rtentry->GetGateway() << std::endl;
				break;
			}

			std::string msg = "hop = ";
			char temp[10];
			sprintf(temp, "%d", hop);
			msg += temp;

			NS_LOG_LOGIC(msg);
		}

		if (rtentry != 0) {
			NS_LOG_LOGIC("Matching route via " << rtentry->GetGateway() << " at the end");
		} else {
			NS_LOG_LOGIC("No matching route to " << dest << " found");
		}
		return rtentry;
	}

	void JointRouting::NotifyInterfaceUp (uint32_t interface) {
		NS_LOG_FUNCTION(this << interface);
		// If interface address and network mask have been set, add a route
		// to the network of the interface (like e.g. ifconfig does on a Linux box)
		for (uint32_t j = 0; j < m_ipv4->GetNAddresses(interface); j++) {
			if (m_ipv4->GetAddress(interface, j).GetLocal() != Ipv4Address() &&
				m_ipv4->GetAddress(interface, j).GetMask()  != Ipv4Mask() 	 &&
				m_ipv4->GetAddress(interface, j).GetMask()  != Ipv4Mask::GetOnes()) {
				AddNetworkRouteTo(
						m_ipv4->GetAddress(interface, j).GetLocal().CombineMask(m_ipv4->GetAddress(interface, j).GetMask()),
						m_ipv4->GetAddress(interface, j).GetMask(),
						interface);
			}
		}
	}

	void JointRouting::NotifyInterfaceDown (uint32_t interface) {
		NS_LOG_FUNCTION(this << interface);
		// Remove all static routes that are going through this interface
		uint32_t j = 0;
		while (j < GetNRoutes()) {
			Ipv4RoutingTableEntry route = GetRoute(j);
			if (route.GetInterface() == interface) {
				RemoveRoute(j);
			} else {
				j++;
			}
		}
	}

	void JointRouting::NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address) {
		NS_LOG_FUNCTION(this << interface << " " << address.GetLocal ());
		if (!m_ipv4->IsUp(interface)) {
			return;
		}
		Ipv4Address networkAddress = address.GetLocal().CombineMask(address.GetMask());
		Ipv4Mask networkMask = address.GetMask();
		if (address.GetLocal() != Ipv4Address()	&& address.GetMask() != Ipv4Mask()) {
			AddNetworkRouteTo(networkAddress, networkMask, interface);
		}
	}

	void JointRouting::NotifyRemoveAddress(uint32_t interface, Ipv4InterfaceAddress address) {
		NS_LOG_FUNCTION(this << interface << " " << address.GetLocal ());
		if (!m_ipv4->IsUp(interface)) {
			return;
		}
		Ipv4Address networkAddress = address.GetLocal().CombineMask(address.GetMask());
		Ipv4Mask networkMask = address.GetMask();
		// Remove all static routes that are going through this interface
		// which reference this network
		for (uint32_t j = 0; j < GetNRoutes(); j++) {
			Ipv4RoutingTableEntry route = GetRoute(j);
			if (route.GetInterface() == interface && route.IsNetwork() &&
				route.GetDestNetwork() == networkAddress &&
				route.GetDestNetworkMask() == networkMask) {
				RemoveRoute(j);
			}
		}
	}

	void JointRouting::SetIpv4 (Ptr<Ipv4> ipv4){
		NS_LOG_FUNCTION(this << ipv4);
		NS_ASSERT(m_ipv4 == 0 && ipv4 != 0);
		m_ipv4 = ipv4;
		for (uint32_t i = 0; i < m_ipv4->GetNInterfaces(); i++) {
			if (m_ipv4->IsUp(i)) {
				NotifyInterfaceUp(i);
			} else {
				NotifyInterfaceDown(i);
			}
		}
	}

	void JointRouting::PrintRoutingTable (Ptr<OutputStreamWrapper> stream) const  {
		std::ostream* os = stream->GetStream();
		if(GetNRoutes() > 0) {
			*os << "Destination     Gateway         Genmask         Flags Metric Ref    Use Iface"	<< std::endl;
			for (uint32_t j = 0; j < GetNRoutes(); j++) {
				std::ostringstream dest, gw, mask, flags;
				Ipv4RoutingTableEntry route = GetRoute(j);
				dest << route.GetDest();
				*os << std::setiosflags(std::ios::left) << std::setw(16) << dest.str();
				gw << route.GetGateway();
				*os << std::setiosflags(std::ios::left) << std::setw(16) << gw.str();
				mask << route.GetDestNetworkMask();
				*os << std::setiosflags(std::ios::left) << std::setw(16) << mask.str();
				flags << "U";
				flags << (route.IsHost() ? "HS" : "GS");
				/*if (route.IsHost()) { flags << "HS"; } else if (route.IsGateway()) { flags << "GS"; }*/
				*os << std::setiosflags(std::ios::left) << std::setw(6)	<< flags.str();
				*os << std::setiosflags(std::ios::left) << std::setw(7)	<< GetMetric(j);
				*os << "-" << "      ";
				*os << "-" << "   ";
				if (Names::FindName(m_ipv4->GetNetDevice(route.GetInterface())) != "") {
					*os	<< Names::FindName(m_ipv4->GetNetDevice(route.GetInterface()));
				} else {
					*os << route.GetInterface();
				}
				*os << std::endl;
			}
		}
	}

	/*void JointRouting::AddMulticastRoute (Ipv4Address origin, Ipv4Address group, uint32_t inputInterface, std::vector<uint32_t> outputInterfaces){
		NS_LOG_FUNCTION(this << origin << " " << group << " " << inputInterface << " " << &outputInterfaces);
		Ipv4MulticastRoutingTableEntry *route = new Ipv4MulticastRoutingTableEntry();
		*route = Ipv4MulticastRoutingTableEntry::CreateMulticastRoute(origin, group, inputInterface, outputInterfaces);
		m_multicastRoutes.push_back(route);
	}*/

	uint32_t JointRouting::GetNRoutes(void) const {
		NS_LOG_FUNCTION(this);
		return m_networkRoutes.size();
	}

	Ipv4RoutingTableEntry JointRouting::GetRoute(uint32_t index) const {
		NS_LOG_FUNCTION(this << index);
		uint32_t tmp = 0;
		for (NetworkRoutes::const_iterator j = m_networkRoutes.begin(); j != m_networkRoutes.end(); j++) {
			if (tmp == index) {
				return j->first;
			}
			tmp++;
		}
		NS_ASSERT(false);
		return 0;
	}

	uint32_t JointRouting::GetMetric(uint32_t index) const {
		NS_LOG_FUNCTION(this << index);
		uint32_t tmp = 0;
		for (NetworkRoutes::const_iterator j = m_networkRoutes.begin(); j != m_networkRoutes.end(); j++) {
			if (tmp == index)
				return j->second;
			tmp++;
		}
		NS_ASSERT(false);
		return 0;
	}

	void JointRouting::RemoveRoute(uint32_t index) {
		NS_LOG_FUNCTION(this << index);
		uint32_t tmp = 0;
		for (NetworkRoutes::iterator j = m_networkRoutes.begin(); j != m_networkRoutes.end(); j++) {
			if (tmp == index) {
				delete j->first;
				m_networkRoutes.erase(j);
				return;
			}
			tmp++;
		}
		NS_ASSERT(false);
	}

	void JointRouting::AddNetworkRouteTo(Ipv4Address network, Ipv4Mask networkMask, uint32_t interface, uint32_t metric) {
		NS_LOG_FUNCTION(this << network << " " << networkMask << " " << interface << " " << metric);
		Ipv4RoutingTableEntry * route = new Ipv4RoutingTableEntry();
		*route = Ipv4RoutingTableEntry::CreateNetworkRouteTo(network, networkMask, interface);
		m_networkRoutes.push_back(std::make_pair(route, metric));
	}

	int JointRouting::getNodeId() const {
		return this->m_nodeId;
	}

	void JointRouting::setNodeId(int nodeId) {
		this->m_nodeId = nodeId;
	}

	const std::map< std::pair<int, int>, Ipv4Address>& JointRouting::getNodesAddress() const {
		return m_nodesAddress;
	}

	void JointRouting::setNodesAddress(const std::map< std::pair<int, int>, Ipv4Address>& nodesAddress) {
		this->m_nodesAddress = nodesAddress;
	}

	algorithm::LABORARouting * JointRouting::getLRouting() const {
		return this->m_routing;
	}

	void JointRouting::setLRouting(algorithm::LABORARouting * rall) {
		this->m_routing = rall;
	}

	void JointRouting::setRcaa(algorithm::CRAA *  craa) {
		this->m_craa = craa;
	}

}
