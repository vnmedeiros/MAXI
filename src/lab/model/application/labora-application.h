/*
 * labora-application.h
 *
 *  Created on: Nov 23, 2016
 *      Author: vinicius
 */

#ifndef SRC_LABORA_APPLICATION_H_
#define SRC_LABORA_APPLICATION_H_

#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/data-rate.h"
#include "ns3/traced-callback.h"
#include "ns3/info-tag.h"

namespace ns3 {

	class Address;
	class RandomVariableStream;
	class Socket;

	class LaboraApplication : public Application {

	public:
		/**
		 * \brief Get the type ID.
		 * \return the object TypeId
		 */
		static TypeId GetTypeId (void);
		LaboraApplication();
		virtual ~LaboraApplication();
		/**
		 * \brief Set the total number of bytes to send.
		 *
		 * Once these bytes are sent, no packet is sent again, even in on state.
		 * The value zero means that there is no limit.
		 *
		 * \param maxBytes the total number of bytes to send
		 */
		void SetMaxBytes (uint64_t maxBytes);

		/**
		* \brief Set the total number of packets to send.
		*
		* Once these packets are sent, no packet is sent again.
		* The value zero means that there is no limit.
		*
		* \param maxPackets the total number of packets to send
		*/
		void SetMaxPackets (uint64_t maxPackets);

		/**
		 * \brief Return a pointer to associated socket.
		 * \return pointer to associated socket
		 */
		Ptr<Socket> GetSocket (void) const;

		/**
		 * \brief Assign a fixed random variable stream number to the random variables
		 * used by this model.
		 *
		 * \param stream first stream index to use
		 * \return the number of stream indices assigned by this model
		 */
		int64_t AssignStreams (int64_t stream);

		uint32_t classApp;

	protected:
		virtual void DoDispose (void);
	private:
		// inherited from Application base class.
		virtual void StartApplication (void);	// Called at time specified by Start
		virtual void StopApplication (void);	// Called at time specified by Stop

		//helpers
		/**
		 * \brief Cancel all pending events.
		 */
		void CancelEvents ();

		// Event handlers
		void StartSending ();
		void StopSending ();
		void SendPacket ();

		uint32_t m_idControl;

		Ptr<Socket> m_socket;		//!< Associated socket
		uint32_t m_waitToStart;
		Address m_peer;				//!< Peer address
		Address m_local;			//!< local address
		bool m_connected;			//!< True if connected
		Ptr<RandomVariableStream>  m_onTime;       //!< rng for On Time
		Ptr<RandomVariableStream>  m_offTime;      //!< rng for Off Time
		DataRate m_cbrRate;			//!< Rate that data is generated
		DataRate m_cbrRateFailSafe;	//!< Rate that data is generated (check copy)
		uint32_t m_pktSize;			//!< Size of packets
		uint32_t m_residualBits;	//!< Number of generated, but not sent, bits
		Time m_lastStartTime;		//!< Time last packet sent
		uint64_t m_maxBytes;		//!< Limit total number of bytes sent
		uint64_t m_totBytes;		//!< Total bytes sent so far
		uint64_t m_maxPackets;		//!< Limit total number of bytes sent
		uint64_t m_totPackets;		//!< Total packets sent so far
		EventId m_startStopEvent;	//!< Event id for next start or stop event
		EventId m_sendEvent;		//!< Event id of pending "send packet" event
		TypeId m_tid;				//!< Type of the socket used

		/// Traced Callback: transmitted packets.
		TracedCallback<Ptr<const Packet> > m_txTrace;

	private:
		/**
		 * \brief Schedule the next packet transmission
		 */
		void ScheduleNextTx ();
		/**
		 * \brief Schedule the next On period start
		 */
		void ScheduleStartEvent ();

		/**
		 * \brief Schedule the next Off period start
		 */
		void ScheduleStopEvent ();

		/**
		 * \brief Handle a Connection Succeed event
		 * \param socket the connected socket
		 */
		void ConnectionSucceeded (Ptr<Socket> socket);
		/**
		 * \brief Handle a Connection Failed event
		 * \param socket the not connected socket
		 */
		void ConnectionFailed (Ptr<Socket> socket);
	};
} // namespace ns3
#endif /* SRC_LABORA_APPLICATION_H_ */
