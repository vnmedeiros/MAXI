/*
 * labora-application.cc
 *
 *  Created on: Nov 23, 2016
 *      Author: vinicius
 */

#include "ns3/log.h"
#include "ns3/address.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/packet-socket-address.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/data-rate.h"
#include "ns3/random-variable-stream.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/application.h"
#include "labora-application.h"
#include "ns3/udp-header.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("LaboraApplication");
NS_OBJECT_ENSURE_REGISTERED(LaboraApplication);

TypeId
LaboraApplication::GetTypeId (void) {
	static TypeId tid = TypeId ("ns3::LaboraApplication")
		  .SetParent<Application>()
		  .SetGroupName("Applications")
		  .AddConstructor<LaboraApplication>()
		  .AddAttribute ("DataRate", "The data rate in on state.",
                   DataRateValue (DataRate ("500kb/s")),
                   MakeDataRateAccessor (&LaboraApplication::m_cbrRate),
                   MakeDataRateChecker())
		  .AddAttribute ("PacketSize", "The size of packets sent in on state",
                   UintegerValue (512),
                   MakeUintegerAccessor (&LaboraApplication::m_pktSize),
                   MakeUintegerChecker<uint32_t>(1))
		  .AddAttribute ("Remote", "The address of the destination",
                   AddressValue (),
                   MakeAddressAccessor (&LaboraApplication::m_peer),
                   MakeAddressChecker ())
		   .AddAttribute ("local", "The address of the local",
				   AddressValue (),
				   MakeAddressAccessor (&LaboraApplication::m_local),
				   MakeAddressChecker ())
		  .AddAttribute ("MaxBytes",
                   "The total number of bytes to send. Once these bytes are sent, "
                   "no packet is sent again, even in on state. The value zero means "
                   "that there is no limit.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&LaboraApplication::m_maxBytes),
                   MakeUintegerChecker<uint64_t>())
		  .AddAttribute ("MaxPackets",
				   "The total number of packets to send. Once these packets are sent, "
				   "no packet is sent again, even in on state. The value zero means "
				   "that there is no limit.",
				    UintegerValue (0),
				    MakeUintegerAccessor (&LaboraApplication::m_maxPackets),
				    MakeUintegerChecker<uint64_t>())
		  .AddAttribute ("Protocol", "The type of protocol to use.",
                   TypeIdValue (UdpSocketFactory::GetTypeId ()),
                   MakeTypeIdAccessor (&LaboraApplication::m_tid),
                   MakeTypeIdChecker())
		  .AddAttribute ("OnTime", "A RandomVariableStream used to pick the duration of the 'On' state.",
			       StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
			       MakePointerAccessor (&LaboraApplication::m_onTime),
			       MakePointerChecker <RandomVariableStream>())
		  .AddAttribute ("OffTime", "A RandomVariableStream used to pick the duration of the 'Off' state.",
			       StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
			       MakePointerAccessor (&LaboraApplication::m_offTime),
			       MakePointerChecker <RandomVariableStream>())
		  .AddAttribute ("classApp", "Type Class App.",
				  UintegerValue (0),
				  MakeUintegerAccessor (&LaboraApplication::classApp),
				  MakeUintegerChecker<uint32_t>())
		  .AddTraceSource ("Tx", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&LaboraApplication::m_txTrace),
                     "ns3::Packet::TracedCallback");

	return tid;

}

LaboraApplication::LaboraApplication() :
		m_idControl(0),
		m_socket(0),
		m_connected(false),
		m_residualBits(0),
		m_lastStartTime(Seconds(0)),
		m_totBytes(0),
		m_totPackets(0){
	;
	NS_LOG_FUNCTION(this);
}

LaboraApplication::~LaboraApplication() {
	NS_LOG_FUNCTION(this);
}

void LaboraApplication::SetMaxBytes(uint64_t maxBytes) {
	NS_LOG_FUNCTION (this << maxBytes);
	m_maxBytes = maxBytes;
}

void LaboraApplication::SetMaxPackets(uint64_t maxPackets) {
	NS_LOG_FUNCTION (this << maxPackets);
	m_maxPackets = maxPackets;
}

Ptr<Socket> LaboraApplication::GetSocket(void) const {
	NS_LOG_FUNCTION (this);
	return m_socket;
}

int64_t LaboraApplication::AssignStreams(int64_t stream) {
	NS_LOG_FUNCTION (this << stream);
	//m_onTime->SetStream (stream);
	//m_offTime->SetStream (stream + 1);
	return 2;
}

void LaboraApplication::DoDispose(void) {
	NS_LOG_FUNCTION(this);
	m_socket = 0;
	Application::DoDispose();
}

// Application Methods
void LaboraApplication::StartApplication () { // Called at time specified by Start
	NS_LOG_FUNCTION (this);
	// Create the socket if not already
	if (!m_socket) {
		m_socket = Socket::CreateSocket(GetNode(), m_tid);
		if (Inet6SocketAddress::IsMatchingType(m_peer)) {
			m_socket->Bind6();
		}
		else if (InetSocketAddress::IsMatchingType(m_peer) || PacketSocketAddress::IsMatchingType(m_peer)) {
			m_socket->Bind(this->m_local);
			//m_socket->Bind();
		}
		m_socket->Connect(m_peer);
		m_socket->SetAllowBroadcast(true);
		m_socket->ShutdownRecv();
		m_socket->SetConnectCallback(
				MakeCallback(&LaboraApplication::ConnectionSucceeded, this),
				MakeCallback(&LaboraApplication::ConnectionFailed, this));
	}
	m_cbrRateFailSafe = m_cbrRate;
	// Insure no pending event
	CancelEvents();
	// If we are not yet connected, there is nothing to do here
	// The ConnectionComplete upcall will start timers at that time
	//if (!m_connected) return;

	double time = (double)(rand() % 400)/200;
	Simulator::Schedule (Seconds(time), &LaboraApplication::ScheduleStartEvent, this);
	//ScheduleStartEvent ();
}

void LaboraApplication::StopApplication () {// Called at time specified by Stop
	NS_LOG_FUNCTION(this);
	CancelEvents();
	if(m_socket != 0) {
		m_socket->Close();
	} else {
		NS_LOG_WARN ("LaboraApplication found null socket to close in StopApplication");
    }
}

void LaboraApplication::CancelEvents() {
	NS_LOG_FUNCTION(this);
	if (m_sendEvent.IsRunning() && m_cbrRateFailSafe == m_cbrRate ) {
		// Cancel the pending send packet event
		// Calculate residual bits since last packet sent
		Time delta (Simulator::Now() - m_lastStartTime);
		int64x64_t bits = delta.To(Time::S) * m_cbrRate.GetBitRate();
		m_residualBits += bits.GetHigh();
    }
	m_cbrRateFailSafe = m_cbrRate;
	Simulator::Cancel(m_sendEvent);
	Simulator::Cancel(m_startStopEvent);
}

void LaboraApplication::ScheduleStartEvent() { // Schedules the event to start sending data
	NS_LOG_FUNCTION (this);
	Time offInterval = Seconds (m_offTime->GetValue ());
	NS_LOG_LOGIC ("start at " << offInterval);
	m_startStopEvent = Simulator::Schedule (offInterval, &LaboraApplication::StartSending, this);
}

void LaboraApplication::ScheduleStopEvent () { // Schedules the event to stop sending data
	NS_LOG_FUNCTION(this);
	Time onInterval = Seconds(m_onTime->GetValue());
	NS_LOG_LOGIC ("stop at " << onInterval);
	m_startStopEvent = Simulator::Schedule(onInterval, &LaboraApplication::StopSending, this);
}

// Event handlers
void LaboraApplication::StartSending() {
	NS_LOG_FUNCTION(this);
	m_lastStartTime = Simulator::Now();
	ScheduleNextTx();	//Schedule the send packet event
	ScheduleStopEvent();
}

void LaboraApplication::StopSending() {
	NS_LOG_FUNCTION (this);
	CancelEvents();
	ScheduleStartEvent();
}

// Private helpers
void LaboraApplication::ScheduleNextTx() {
	NS_LOG_FUNCTION (this);
	if ((m_maxBytes == 0 || m_totBytes < m_maxBytes) && (m_maxPackets == 0 || m_totPackets < m_maxPackets)) {
		uint32_t bits = m_pktSize * 8 - m_residualBits;
		NS_LOG_LOGIC("bits = " << bits);
		Time nextTime (Seconds(bits/static_cast<double>(m_cbrRate.GetBitRate()))); //Time till next packet
		NS_LOG_LOGIC("nextTime = " << nextTime);
		m_sendEvent = Simulator::Schedule(nextTime, &LaboraApplication::SendPacket, this);
    } else { // All done, cancel any pending events
    	StopApplication ();
    }
}

void LaboraApplication::SendPacket() {
	NS_LOG_FUNCTION (this);
	NS_ASSERT(m_sendEvent.IsExpired());
	//std::cout << "Apllication Send" << m_idControl << std::endl;
	Ptr<Packet> packet = Create<Packet>(m_pktSize);
	InfoTag infoTag;
	infoTag.SetId(m_idControl++);
	infoTag.SetSrc(this->GetNode()->GetId());
	infoTag.SetNodeId(this->GetNode()->GetId());
	infoTag.SetClassApp(static_cast<uint32_t>(this->classApp));
	//std::cout << "LaboraApplication::SendPacket " << infoTag.GetClassApp( ) << std::endl;
	packet->AddPacketTag(infoTag);
	m_txTrace(packet);

	UdpHeader udph;
	if(packet->RemoveHeader(udph) != 0) {
		udph.SetDestinationPort(InetSocketAddress::ConvertFrom (m_peer).GetPort ());
		udph.SetSourcePort(InetSocketAddress::ConvertFrom (m_local).GetPort ());		
		packet->AddHeader(udph);
	}

	m_socket->Send(packet);

	m_totBytes += m_pktSize;
	m_totPackets++;
	if(InetSocketAddress::IsMatchingType(m_peer)) {
		/*std::cout << "At time " << Simulator::Now ().GetSeconds ()
						<< "s on-off application sent "
						<<  packet->GetSize () << " bytes to "
						<< InetSocketAddress::ConvertFrom(m_peer).GetIpv4 ()
						<< " port " << InetSocketAddress::ConvertFrom (m_peer).GetPort ()
						<< " total Tx " << m_totBytes << " bytes"
						<< " for node ID" << this->GetNode()->GetId() << std::endl;*/

		NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds ()
				<< "s on-off application sent "
				<<  packet->GetSize () << " bytes to "
				<< InetSocketAddress::ConvertFrom(m_peer).GetIpv4 ()
				<< " port " << InetSocketAddress::ConvertFrom (m_peer).GetPort ()
				<< " total Tx " << m_totBytes << " bytes");
    } else if (Inet6SocketAddress::IsMatchingType (m_peer)) {
    	NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds ()
    			<< "s on-off application sent "
				<<  packet->GetSize () << " bytes to "
				<< Inet6SocketAddress::ConvertFrom(m_peer).GetIpv6 ()
				<< " port " << Inet6SocketAddress::ConvertFrom (m_peer).GetPort ()
				<< " total Tx " << m_totBytes << " bytes");
    }
	m_lastStartTime = Simulator::Now();
	m_residualBits = 0;
	ScheduleNextTx();
}

void LaboraApplication::ConnectionSucceeded (Ptr<Socket> socket) {
	NS_LOG_FUNCTION (this << socket);
	m_connected = true;
}

void LaboraApplication::ConnectionFailed (Ptr<Socket> socket) {
	NS_LOG_FUNCTION (this << socket);
}

} /* namespace ns3 */
