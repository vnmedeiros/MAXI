/*
 * info-tag.cc
 *
 *  Created on: Oct 21, 2016
 *      Author: vinicius
 */
#include "ns3/assert.h"
#include "ns3/abort.h"
#include "ns3/log.h"
#include "routing-header.h"

NS_LOG_COMPONENT_DEFINE ("RoutingHeader");

using namespace ns3;

//InfoTag::InfoTag:():m_sinr(0){}

RoutingHeader::RoutingHeader() : m_hops(0) {
}

RoutingHeader::RoutingHeader (const RoutingHeader &o) {
	this->m_hops = o.m_hops;
}

RoutingHeader::RoutingHeader(std::vector<int> hops) {
	this->m_hops.assign(hops.begin(),hops.end());
}

RoutingHeader::~RoutingHeader() {

}

TypeId RoutingHeader::GetTypeId (void) {
  static TypeId tid = TypeId ("ns3::InfoHeader")
		  .SetParent<Header>()
		  .AddConstructor<RoutingHeader>()
  ;
  return tid;
}
TypeId RoutingHeader::GetInstanceTypeId (void) const {
  return GetTypeId();
}

void RoutingHeader::Print (std::ostream &os) const {
	os << "[ ";
	for(unsigned int i=0; i< this->m_hops.size(); i++) {
		os << this->m_hops[i] << " ";
	}
	os << "]\n";
}

uint32_t RoutingHeader::GetSerializedSize (void) const {
	return sizeof(unsigned int) + this->m_hops.size () * sizeof(uint16_t) + sizeof(m_messageType);
}

void RoutingHeader::Serialize (Buffer::Iterator start) const {
	start.WriteU8 (this->m_messageType);
	start.WriteHtonU32(this->m_hops.size());
	for(unsigned int i=0; i< this->m_hops.size(); i++) {
		start.WriteHtonU16 (this->m_hops[i]);
	}
}

uint32_t RoutingHeader::Deserialize (Buffer::Iterator start) {
	this->m_messageType  = (MessageType) start.ReadU8();
	unsigned int size = start.ReadNtohU32();
	for(unsigned int i=0; i < size; i++) {
		int item = start.ReadNtohU16();
		this->m_hops.push_back(item);
	}
	return size * sizeof(uint16_t) + sizeof(unsigned int);
}

void RoutingHeader::SetHops (std::vector<uint16_t> hops) {
	this->m_hops = hops;
}

std::vector<uint16_t> RoutingHeader::GetHops () {
	return this->m_hops;
}

void RoutingHeader::SetMessageType (MessageType messageType) {
    this->m_messageType = messageType;
}

RoutingHeader::MessageType RoutingHeader::GetMessageType () const {
    return m_messageType;
}
