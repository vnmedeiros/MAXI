/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#ifndef LABORA_APP_HELPER_H
#define LABORA_APP_HELPER_H

#include <stdint.h>
#include <string>
#include "ns3/object-factory.h"
#include "ns3/address.h"
#include "ns3/attribute.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/application-container.h"
#include "ns3/labora-application.h"

namespace ns3 {

	class DataRate;

	/**
	 * \ingroup onoff
	 * \brief A helper to make it easier to instantiate an ns3::OnOffApplication
	 * on a set of nodes.
	 */
	class LaboraAppHelper
	{
	public:
		/**
		 * \brief Types of Application Classes.
		 *
		 */
		enum Class {
			Class_1, // tamanho do pacotes: 1024 | taxa de dados: 264kbps
			Class_2, // tamanho do pacotes: 1024 | taxa de dados: 1024kbps
			Class_3, // tamanho do pacotes: 1024 | taxa de dados: 10bps
			Class_4, // tamanho do pacotes: 1024 | taxa de dados: 3kpbs
			Class_5  // tamanho do pacotes: 1024 | taxa de dados: 32kpbs
		};


		LaboraAppHelper (std::string protocol, Address address);
		LaboraAppHelper (std::string protocol, Address address, LaboraAppHelper::Class classType);
		void SetAttribute (std::string name, const AttributeValue &value);
		void SetConstantRate (DataRate dataRate, uint32_t packetSize = 512);
		ApplicationContainer Install (NodeContainer c) const;
		ApplicationContainer Install (Ptr<Node> node) const;
		ApplicationContainer Install (std::string nodeName) const;
		int64_t AssignStreams (NodeContainer c, int64_t stream);
	private:
		Ptr<Application> InstallPriv (Ptr<Node> node) const;
		ObjectFactory m_factory; //!< Object factory.
	};

} // namespace ns3

#endif /* LABORA_APP_HELPER_H */

