/*
 * info-tag.cc
 *
 *  Created on: Oct 21, 2016
 *      Author: vinicius
 */
#include "info-tag.h"

using namespace ns3;

//InfoTag::InfoTag:():m_sinr(0){}

InfoTag::InfoTag() :
		m_id(0), m_src(0) ,m_nodeID(0), m_previousNodeID(0), m_typeTag(TypeTag::TAG_DATA) {
}
InfoTag::InfoTag(uint32_t id, uint32_t src) {
	this->m_id = id;
	this->m_src = src;
	this->m_nodeID = 0;
	this->m_previousNodeID = 0;
	this->m_typeTag = TypeTag::TAG_DATA;
}

InfoTag::~InfoTag() {

}

TypeId InfoTag::GetTypeId (void) {
	static TypeId tid = TypeId ("ns3::InfoTag")
	    .SetParent<Tag> ()
	    .AddConstructor<InfoTag> ();
	return tid;
}

TypeId InfoTag::GetInstanceTypeId (void) const {
	return GetTypeId ();
}

void InfoTag::Print (std::ostream &os) const {
  os << "[ id:" << m_id << "] "
	 << "src = " << m_src << " "
	 << "nodeId = " << m_nodeID << "\n";
}

uint32_t InfoTag::GetSerializedSize (void) const {
	return sizeof(uint32_t) * 4 + sizeof(m_typeTag) + sizeof(m_classApp);
}

void InfoTag::Serialize (TagBuffer i) const {
	i.WriteU32(m_id);
	i.WriteU32(m_src);
	i.WriteU32(m_nodeID);
	i.WriteU32(m_previousNodeID);
	//i.WriteDouble(m_per);
	i.WriteU32(static_cast<unsigned int>(m_typeTag));
	i.WriteU32(m_classApp);
}

void InfoTag::Deserialize (TagBuffer i) {
	m_id = i.ReadU32();
	m_src = i.ReadU32();
	m_nodeID = i.ReadU32();
	m_previousNodeID = i.ReadU32();
	//m_per = i.ReadDouble();
	m_typeTag = static_cast<TypeTag>(i.ReadU32());
	m_classApp = i.ReadU32();
}

void InfoTag::SetId(uint32_t id) {
	m_id = id;
}

uint32_t InfoTag::GetId() const {
	return m_id;
}

void InfoTag::SetSrc(uint32_t src) {
	this->m_src = src;
}
uint32_t InfoTag::GetSrc(void) const {
	return this->m_src;
}

void InfoTag::SetTypeTag(TypeTag typeTag) {
	this->m_typeTag = typeTag;
}

TypeTag InfoTag::GetTypeTag(void) const {
	return this->m_typeTag;
}

void InfoTag::SetNodeId(uint32_t nodeId) {
	this->m_nodeID = nodeId;
}

uint32_t InfoTag::GetNodeID(void) const {
	return this->m_nodeID;
}

uint32_t InfoTag::GetPreviousNodeID(void) const {
	return this->m_previousNodeID;
}

void InfoTag::SetPreviousNodeID(uint32_t id) {
	this->m_previousNodeID = id;
}

void InfoTag::SetClassApp(uint32_t classApp) {
	this->m_classApp = classApp;
}

uint32_t InfoTag::GetClassApp(void) const {
	return this->m_classApp;
}
