/*
 * joint-routing.h
 *
 *  Created on: Nov 10, 2016
 *      Author: vinicius
 */

#ifndef SRC_LAB_MODEL_JOINT_ROUTING_H_
#define SRC_LAB_MODEL_JOINT_ROUTING_H_

#include <iostream>
#include <list>
#include <utility>
#include <stdint.h>
#include <map>
#include "ns3/object.h"
#include "ns3/core-module.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv4-header.h"
#include "ns3/socket.h"
#include "ns3/ptr.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-routing-protocol.h"
//#include "ns3/joint-route.h"
#include "ns3/lab-module.h"

namespace ns3 {

	class Packet;
	class NetDevice;
	class Ipv4Interface;
	class Ipv4Address;
	class Ipv4Header;
	class Ipv4RoutingTableEntry;
	class Ipv4MulticastRoutingTableEntry;
	class Node;

	class JointRouting : public Ipv4RoutingProtocol {
	public:
		JointRouting();
		virtual ~JointRouting();

		static TypeId GetTypeId (void);
		virtual Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p, const Ipv4Header &header, Ptr<NetDevice> oif, Socket::SocketErrno &sockerr);
		virtual bool RouteInput (Ptr<const Packet> p, const Ipv4Header &header, Ptr<const NetDevice> idev,
				UnicastForwardCallback ucb, MulticastForwardCallback mcb,
				LocalDeliverCallback lcb, ErrorCallback ecb);

		virtual void NotifyInterfaceUp (uint32_t interface);
		virtual void NotifyInterfaceDown (uint32_t interface);
		virtual void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address);
		virtual void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address);
		virtual void SetIpv4 (Ptr<Ipv4> ipv4);
		virtual void PrintRoutingTable (Ptr<OutputStreamWrapper> stream) const;
		//void AddMulticastRoute (Ipv4Address origin, Ipv4Address group, uint32_t inputInterface, std::vector<uint32_t> outputInterfaces);

		Callback<void, int, int, double> m_callbackUpdateLinkQuality;

		int getNodeId() const;
		void setNodeId(int nodeId);
		const std::map< std::pair<int, int>, Ipv4Address>& getNodesAddress() const;
		void setNodesAddress(const std::map< std::pair<int, int>, Ipv4Address>& nodesAddress);
		algorithm::LABORARouting * getLRouting() const;
		void setLRouting(algorithm::LABORARouting * rall);
		void setRcaa(algorithm::CRAA *  craa);
	private:
		typedef std::list<std::pair <Ipv4RoutingTableEntry *, uint32_t> > NetworkRoutes;

		Ptr<Ipv4Route> LookupStatic(Ipv4Address dest, RoutingHeader * jointRoute, Ptr<NetDevice> oif = 0);

		Ipv4RoutingTableEntry GetRoute(uint32_t index) const;
		uint32_t GetNRoutes(void) const;
		uint32_t GetMetric(uint32_t index) const;
		void RemoveRoute(uint32_t index);
		void AddNetworkRouteTo (Ipv4Address network, Ipv4Mask networkMask, uint32_t interface, uint32_t metric = 0);
	private:
		std::map< std::pair<int, int>, Ipv4Address > m_nodesAddress;
		NetworkRoutes m_networkRoutes;
		Ptr<Ipv4> m_ipv4;
		algorithm::LABORARouting * m_routing;
		algorithm::CRAA * m_craa;
		int m_nodeId;

	};
} // namespace ns3
#endif /* SRC_LAB_MODEL_JOINT_ROUTING_H_ */
