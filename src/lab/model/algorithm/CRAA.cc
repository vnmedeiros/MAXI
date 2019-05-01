/*
 * CRAA.cc
 *
 *  Created on: Nov 4, 2016
 *      Author: vinicius
 */

#include "CRAA.h"

NS_LOG_COMPONENT_DEFINE("CRAA");

namespace algorithm {

/*CRAA::CRAA(LBR *lbr, int allowedReadjustments, double interferenceThreshold,
		std::set<CRAAChannel> channels, int nRadios, int nFlows,
		std::map<element::Edge, std::map<element::Edge, double> > per) {
	for (unsigned int i = 0; i < lbr->m_E.size(); i++) {
		m_allocatedChannels[lbr->m_E[i]] = 1;
	}

	m_lbr = lbr;
	m_allowedReadjustments = allowedReadjustments;
	m_interferenceThreshold = interferenceThreshold;
	m_channels = channels;
	m_nRadios = nRadios;
	m_nFlows = nFlows;
	m_nFinishedFlows = 0;
	m_per = per;
	m_nChannelChanges = 0;
}

CRAA::CRAA(RALL *rall, int allowedReadjustments, double interferenceThreshold,
		std::set<CRAAChannel> channels, int nRadios, int nFlows,
		std::map<element::Edge, std::map<element::Edge, double> > per) {
	for (unsigned int i = 0; i < rall->getGraph()->E.size(); i++) {
		m_allocatedChannels[rall->getGraph()->E[i]] = 1;
	}

	m_rall = rall;
	m_allowedReadjustments = allowedReadjustments;
	m_interferenceThreshold = interferenceThreshold;
	m_channels = channels;
	m_nRadios = nRadios;
	m_nFlows = nFlows;
	m_nFinishedFlows = 0;
	m_per = per;
	m_nChannelChanges = 0;
}*/

CRAA::CRAA(LABORARouting * routing, int allowedReadjustments, double interferenceThreshold,
		std::set<CRAAChannel> channels, int nRadios, int nFlows,
		std::map<element::Edge, std::map<element::Edge, double> > per, int channelDefault) {
	for (unsigned int i = 0; i < routing->getGraph()->E.size(); i++) {
		m_allocatedChannels[routing->getGraph()->E[i]] = channelDefault;
		routing->getGraph()->getEdge(routing->getGraph()->E[i].m_source, routing->getGraph()->E[i].m_sink)->channel = channelDefault;
	}

	m_routing = routing;
	m_allowedReadjustments = allowedReadjustments;
	m_interferenceThreshold = interferenceThreshold;
	m_channels = channels;
	m_nRadios = nRadios;
	m_nFlows = nFlows;
	m_nFinishedFlows = 0;
	m_per = per;
	m_nChannelChanges = 0;
}

inline CRAA::~CRAA() {
}

double CRAA::EdgeInterferenceInitial(element::Edge u, element::Edge w) {
	return (u.m_utilization * w.m_utilization * 1);
}

double CRAA::EdgeInterference(element::Edge u, element::Edge w) {
	element::Edge mn(u.m_source, u.m_sink, 1);
	element::Edge nm(u.m_sink, u.m_source, 1);
	element::Edge pq(w.m_source, w.m_sink, 1);
	element::Edge qp(w.m_sink, w.m_source, 1);

	for (unsigned int i = 0; i < m_routing->getGraph()->E.size(); i++) {
		if (mn == m_routing->getGraph()->E[i]) {
			mn = m_routing->getGraph()->E[i];
		}
		if (nm == m_routing->getGraph()->E[i]) {
			nm = m_routing->getGraph()->E[i];
		}
		if (pq == m_routing->getGraph()->E[i]) {
			pq = m_routing->getGraph()->E[i];
		}
		if (qp == m_routing->getGraph()->E[i]) {
			qp = m_routing->getGraph()->E[i];
		}
	}

	return (mn.m_utilization * pq.m_utilization * 1)
			+ (mn.m_utilization * qp.m_utilization * 1)
			+ (nm.m_utilization * pq.m_utilization * 1)
			+ (nm.m_utilization * qp.m_utilization * 1);
}

bool CRAA::ICV(std::vector<element::Edge> E, int node, CRAAChannel currentChannel,
		CRAAChannel candidateChannel) {
	int transmittingInTheCandidate = 0;
	int transmittingInTheCurrent = 0;
	int nUsedChannels = 0;

	for (uint32_t i = 0; i < E.size(); i++) {
		element::Edge e = E[i];

		if (e.m_source == node || e.m_sink == node) {
			if (m_allocatedChannels[e] == candidateChannel) {
				transmittingInTheCandidate++;
			}

			if (m_allocatedChannels[e] == currentChannel) {
				transmittingInTheCurrent++;
			}
		}
	}

	for (std::set<CRAAChannel>::iterator it = m_channels.begin();
			it != m_channels.end(); ++it) {
		CRAAChannel channel = *it;

		for (uint32_t i = 0; i < E.size(); i++) {
			element::Edge e = E[i];

			if (e.m_source == node || e.m_sink == node) {
				if (m_allocatedChannels[e] == channel) {
					nUsedChannels++;
					break;
				}
			}
		}
	}

	if (nUsedChannels > m_nRadios) {
		NS_LOG_ERROR("ERROR: nUsedChannels > nRadios");
		exit(1);
	}

	NS_LOG_LOGIC("ICV:");
	NS_LOG_LOGIC(
			"transmittingInTheCandidate = " << transmittingInTheCandidate << " transmittingInTheCurrent = " << transmittingInTheCurrent << " nUsedChannels = " << nUsedChannels);

	if (transmittingInTheCandidate == 0 && transmittingInTheCurrent > 1
			&& nUsedChannels == m_nRadios) {
		NS_LOG_LOGIC("ICV == true -- node " << node);
		return true;
	} else {
		NS_LOG_LOGIC("ICV == false -- node " << node);
		return false;
	}
}

std::vector<element::Edge> CRAA::SortEdges(std::vector<element::Edge> E) {
	for (unsigned int i = 0; i < E.size() - 1; i++) {
		int max = i;
		for (unsigned int j = i + 1; j < E.size(); j++) {
			if (E[j].m_utilization > E[max].m_utilization) {
				max = j;
			}
		}

		element::Edge temp = E[max];
		E[max] = E[i];
		E[i] = temp;
	}

	NS_LOG_LOGIC("sorted E: " << E.size());
	for (unsigned int i = 0; i < E.size(); i++) {
		NS_LOG_LOGIC(
				E[i].m_source << "-" << E[i].m_sink << " -- m_utilization " << E[i].m_utilization);
	}
	NS_LOG_LOGIC("");

	return E;
}

void CRAA::InitialUtilization() {
	int G = 0;

	for (unsigned int i = 0; i < m_routing->getGraph()->E.size(); i++) {
		path * p = m_routing->ShortPath(G, m_routing->getGraph()->E[i].m_source);
		int dGM;
		if (p->size() == 0) {
			dGM = 0;
		} else {
			dGM = p->size() - 1;
		}
		delete p;

		p = m_routing->ShortPath(G, m_routing->getGraph()->E[i].m_sink);
		int dGN;
		if (p->size() == 0) {
			dGN = 0;
		} else {
			dGN = p->size() - 1;
		}
		delete p;

		m_routing->getGraph()->E[i].m_utilization = (dGM + dGN) / 2.0;
		m_routing->getGraph()->E[i].m_utilization = std::pow(m_routing->getGraph()->E[i].m_utilization, -1);
	}

	for (unsigned int i = 0; i < m_routing->getGraph()->U.size(); i++) {
		path * p = m_routing->ShortPath(G, m_routing->getGraph()->U[i].m_source);
		int dGM;
		if (p->size() == 0) {
			dGM = 0;
		} else {
			dGM = p->size() - 1;
		}

		p = m_routing->ShortPath(G, m_routing->getGraph()->U[i].m_sink);
		int dGN;
		if (p->size() == 0) {
			dGN = 0;
		} else {
			dGN = p->size() - 1;
		}
		m_routing->getGraph()->U[i].m_utilization = (dGM + dGN) / 2.0;
		m_routing->getGraph()->U[i].m_utilization = std::pow(m_routing->getGraph()->U[i].m_utilization, -1);
	}
}

void CRAA::RunInitial() {
	std::string msg;
	char temp[30];
	InitialUtilization();
	std::vector<element::Edge> sortedEdges = SortEdges(m_routing->getGraph()->U); //current U
	int nReadjustments = 0;
	for (uint32_t i = 0; i < sortedEdges.size(); i++) { //visit all sorted edges
		element::Edge u = sortedEdges[i]; //current edge
		NS_LOG_LOGIC("u = " << u.m_source << "-" << u.m_sink);
		NS_LOG_LOGIC("u.m_utilization = " << u.m_utilization);
		if (u.m_utilization > 0) { //change channel only to edges with flows
			CRAAChannel currentChannel = m_allocatedChannels[u];
			NS_LOG_LOGIC("currentChannel = " << currentChannel);

			double bestValue = 0;
			CRAAChannel bestChannel = currentChannel;

			for (std::set<CRAAChannel>::iterator it = m_channels.begin(); it != m_channels.end(); ++it) {
				NS_LOG_LOGIC("-----------------------------------");
				CRAAChannel candidateChannel = *it;
				NS_LOG_LOGIC("candidateChannel = " << candidateChannel);

				if (candidateChannel == currentChannel) {
					NS_LOG_LOGIC("candidateChannel == currentChannel");
					continue;
				}

				if (ICV(sortedEdges, u.m_source, currentChannel, candidateChannel) ||
					ICV(sortedEdges, u.m_sink, currentChannel, candidateChannel)) {
					//obey interfaces constraints
					NS_LOG_LOGIC("obey interfaces constraints");
					continue;
				}

				double value = 0;
				for (uint32_t j = 0; j < sortedEdges.size(); j++) {
					element::Edge w = sortedEdges[j];
					if (u == w) {
						continue;
					}
					if (m_allocatedChannels[w] == currentChannel) {
						value -= EdgeInterference(u, w);
					} else if (m_allocatedChannels[w] == candidateChannel) {
						value += EdgeInterference(u, w);
					}
				}

				NS_LOG_LOGIC("value = " << value << " bestValue = " << bestValue);
				if (value < bestValue) {
					bestValue = value;
					bestChannel = candidateChannel;
				}
			}
			NS_LOG_LOGIC("-----------------------------------");
			NS_LOG_LOGIC("bestValue = " << bestValue << " m_interferenceThreshold = " << m_interferenceThreshold);
			if (bestValue < m_interferenceThreshold) {
				//if(bestChannel == 2) // \FAZER
				//	bestChannel = 9;
				m_allocatedChannels[u] = bestChannel;
				m_routing->getGraph()->getEdge(u.m_source, u.m_sink)->channel = bestChannel;
				nReadjustments++;
				NS_LOG_LOGIC("bestChannel = " << bestChannel);
			}
		}
		NS_LOG_LOGIC("");
	}

	NS_LOG_LOGIC("Edges - m_allocatedChannels:");
	for (uint32_t i = 0; i < sortedEdges.size(); i++) {
		element::Edge e = sortedEdges[i];
		NS_LOG_LOGIC("Edge " << e.m_source << "-" << e.m_sink << " = " << m_allocatedChannels[e]);
	}
	NS_LOG_LOGIC("");

	NS_LOG_LOGIC("Nodes - m_allocatedChannels:");
	int size = m_routing->getGraph()->getNumberVertices();
	for (int i = 0; i < size; i++) {
		int node = i;
		NS_LOG_LOGIC("Node " << node << ":");
		std::set<CRAAChannel> usedChannels;
		for (uint32_t j = 0; j < sortedEdges.size(); j++) {
			element::Edge e = sortedEdges[j];
			if (e.m_source == i || e.m_sink == i) {
				NS_LOG_LOGIC("Edge " << e.m_source << "-" << e.m_sink << " = " << m_allocatedChannels[e]);
				usedChannels.insert(m_allocatedChannels[e]);
			}
		}
		msg = "nUsedChannels = ";
		sprintf(temp, "%d", (int) usedChannels.size());
		msg += temp;
		msg += ": ";
		if ((int) usedChannels.size() > m_nRadios) {
			NS_LOG_LOGIC("ERROR - usedChannel > m_nRadios");
			exit(1);
		}
		for (std::set<CRAAChannel>::iterator it = usedChannels.begin(); it != usedChannels.end(); ++it) {
			sprintf(temp, "%d", *it);
			msg += temp;
			msg += " ";
		}
		NS_LOG_LOGIC(msg);
		m_channelsToNodes[i] = usedChannels;
	}
	NS_LOG_LOGIC("");
}

void CRAA::Utilization() {
	for (unsigned int i = 0; i < m_routing->getGraph()->E.size(); i++) {
		m_routing->getGraph()->E[i].m_utilization = m_routing->getGraph()->E[i].m_flows.size();
	}

	for (unsigned int i = 0; i < m_routing->getGraph()->U.size(); i++) {
		m_routing->getGraph()->U[i].m_utilization = m_routing->getGraph()->U[i].m_flows.size();
	}
}

void CRAA::Run() {
	char temp[30];
	std::string msg;

	Utilization();
	std::vector<element::Edge> sortedEdges = SortEdges(m_routing->getGraph()->U); //current U
	int nReadjustments = 0;
	for (uint32_t i = 0; i < sortedEdges.size(); i++) { //visit all sorted edges
		element::Edge u = sortedEdges[i]; //current edge
		NS_LOG_LOGIC("u = " << u.m_source << "-" << u.m_sink);

		if (u.m_source == 0 || u.m_sink == 0) {
			break;
		}

		NS_LOG_LOGIC(
				"nReadjustments = " << nReadjustments << " m_allowedReadjustments = " << m_allowedReadjustments);
		if (nReadjustments == m_allowedReadjustments) { //obey readjustment constraint
			break;
		}

		NS_LOG_LOGIC("u.m_utilization = " << u.m_utilization);
		if (u.m_utilization > 0) { //change channel only to edges with flows
			CRAAChannel currentChannel = m_allocatedChannels[u];
			NS_LOG_LOGIC("currentChannel = " << currentChannel);

			double bestValue = 0;
			CRAAChannel bestChannel = currentChannel;

			for (std::set<CRAAChannel>::iterator it = m_channels.begin();
					it != m_channels.end(); ++it) {
				NS_LOG_LOGIC("-----------------------------------");
				CRAAChannel candidateChannel = *it;
				NS_LOG_LOGIC("candidateChannel = " << candidateChannel);

				if (candidateChannel == currentChannel) {
					NS_LOG_LOGIC("candidateChannel == currentChannel");
					continue;
				}

				if (ICV(sortedEdges, u.m_source, currentChannel,
						candidateChannel)
						|| ICV(sortedEdges, u.m_sink, currentChannel,
								candidateChannel)) { //obey interfaces constraints
					NS_LOG_LOGIC("obey interfaces constraints");
					continue;
				}

				double value = 0;
				for (uint32_t j = 0; j < sortedEdges.size(); j++) {
					element::Edge w = sortedEdges[j];
					if (u == w) {
						continue;
					}

					if (m_allocatedChannels[w] == currentChannel) {
						value -= EdgeInterference(u, w);
					} else if (m_allocatedChannels[w] == candidateChannel) {
						value += EdgeInterference(u, w);
					}
				}

				NS_LOG_LOGIC(
						"value = " << value << " bestValue = " << bestValue);
				if (value < bestValue) {
					bestValue = value;
					bestChannel = candidateChannel;
				}
			}
			NS_LOG_LOGIC("-----------------------------------");

			NS_LOG_LOGIC(
					"bestValue = " << bestValue << " m_interferenceThreshold = " << m_interferenceThreshold);
			if (bestValue < m_interferenceThreshold) {
				m_allocatedChannels[u] = bestChannel;
				nReadjustments++;

				NS_LOG_LOGIC("bestChannel = " << bestChannel);

				m_nChannelChanges++;
			}
		}

		NS_LOG_LOGIC("");
	}

	NS_LOG_INFO(
			"nReadjustments = " << nReadjustments << " m_allowedReadjustments = " << m_allowedReadjustments);

	NS_LOG_LOGIC("Edges - m_allocatedChannels:");
	for (uint32_t i = 0; i < sortedEdges.size(); i++) {
		element::Edge e = sortedEdges[i];

		NS_LOG_LOGIC(
				"Edge " << e.m_source << "-" << e.m_sink << " = " << m_allocatedChannels[e]);
	}
	NS_LOG_LOGIC("");

	NS_LOG_LOGIC("Nodes - m_allocatedChannels:");
	for (int i = 0; i < m_routing->getGraph()->getNumberVertices(); i++) {
		int node = i;
		NS_LOG_LOGIC("Node " << node << ":");

		std::set<CRAAChannel> usedChannels;
		for (uint32_t j = 0; j < sortedEdges.size(); j++) {
			element::Edge e = sortedEdges[j];

			if (e.m_source == i || e.m_sink == i) {
				NS_LOG_LOGIC(
						"Edge " << e.m_source << "-" << e.m_sink << " = " << m_allocatedChannels[e]);
				usedChannels.insert(m_allocatedChannels[e]);
			}
		}

		msg = "nUsedChannels = ";
		sprintf(temp, "%d", (int) usedChannels.size());
		msg += temp;
		msg += ": ";
		if ((int) usedChannels.size() > m_nRadios) {
			NS_LOG_LOGIC("ERROR - usedChannel > m_nRadios");
			exit(1);
		}
		for (std::set<CRAAChannel>::iterator it = usedChannels.begin();
				it != usedChannels.end(); ++it) {
			sprintf(temp, "%d", *it);
			msg += temp;
			msg += " ";
		}
		NS_LOG_LOGIC(msg);

		m_channelsToNodes[i] = usedChannels;
	}
	NS_LOG_LOGIC("");
}

void CRAA::SetChannelsToRadios(int *channelsToRadios) {
	m_channelsToRadios = channelsToRadios;
}

} /* namespace */
