/*
 * info-tag.h
 *
 *  Created on: Oct 21, 2016
 *      Author: vinicius
 */

#ifndef SRC_LAB_MODEL_ROUTING_HEADER_H_
#define SRC_LAB_MODEL_ROUTING_HEADER_H_

#include <iostream>
#include "vector"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/packet.h"
#include "ns3/header.h"

using namespace ns3;

class RoutingHeader : public Header {
public:
	enum MessageType {
		HELLO_MESSAGE = 1,
		DT_MESSAGE    = 2,
	};

	RoutingHeader();
	RoutingHeader(const RoutingHeader &o);
	RoutingHeader(std::vector<int> hops);
	virtual ~RoutingHeader();

	/**
	 * \brief Get the type ID.
	 * \return the object TypeId
	 */
	static TypeId GetTypeId (void);
	virtual TypeId GetInstanceTypeId (void) const;
	virtual void Print (std::ostream &os) const;
	virtual uint32_t GetSerializedSize (void) const;
	virtual void Serialize (Buffer::Iterator start) const;
	virtual uint32_t Deserialize (Buffer::Iterator start);

	void SetHops (std::vector<uint16_t> hops);
	std::vector<uint16_t> GetHops ();
	void SetMessageType (MessageType messageType);
	MessageType GetMessageType() const;
private:
	std::vector<uint16_t> m_hops;
	MessageType m_messageType;
};

#endif /* SRC_LAB_MODEL_ROUTING_HEADER_H_ */
