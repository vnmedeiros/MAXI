/*
 * info-tag.h
 *
 *  Created on: Oct 21, 2016
 *      Author: vinicius
 *
 *
 *      alter the MAX SIZE OF TAG in file packet-tag-list.h :
 *      enum TagData_e
 *   	{
 *   		MAX_SIZE = 64
 * 		};
 */

#ifndef SRC_LAB_MODEL_INFO_TAG_H_
#define SRC_LAB_MODEL_INFO_TAG_H_

#include "ns3/tag.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include <iostream>

using namespace ns3;

enum TypeTag {
  TAG_PRELOAD,
  TAG_DATA,
};


/* A Info Header class responsible for store information source, destination and quality metrics
 */
class InfoTag : public Tag {
private:
	uint32_t m_id;
	uint32_t m_src;
	uint32_t m_nodeID;
	uint32_t m_previousNodeID;
	TypeTag m_typeTag;
	//LaboraAppHelper::Class m_classApp;
	uint32_t m_classApp;

public:
	InfoTag();
	InfoTag(uint32_t id, uint32_t src);
	virtual ~InfoTag();
	/**
	 * \brief Get the type ID.
	 * \return the object TypeId
	 */
	static TypeId GetTypeId (void);
	virtual TypeId GetInstanceTypeId (void) const;
	virtual uint32_t GetSerializedSize (void) const;
	virtual void Serialize (TagBuffer i) const;
	virtual void Deserialize (TagBuffer i);
	virtual void Print (std::ostream &os) const;

	/**
	 * \brief methods of access to data header attributes
	 */
	void SetId(uint32_t id);
	void SetSrc (uint32_t src);
	void SetNodeId(uint32_t id);
	void SetTypeTag(TypeTag typeTag);
	void SetPreviousNodeID(uint32_t id);
	void SetClassApp(uint32_t classApp);

	uint32_t GetId() const;
	uint32_t GetSrc (void) const;
	TypeTag GetTypeTag(void) const;
	uint32_t GetNodeID(void) const;
	uint32_t GetPreviousNodeID(void) const;
	uint32_t GetClassApp(void) const;
};

#endif /* SRC_LAB_MODEL_INFO_TAG_H_ */
