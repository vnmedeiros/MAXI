/*
 * JRCAR.cc
 *
 *  Created on: Nov 4, 2016
 *      Author: vinicius
 */

#include "LBRJrcar.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LBR-JR-CAR");

std::string pathToString(path p) {
	char temp[30];
	std::string path = "";
	for (unsigned int i = 0; i < p.size(); i++) {
		sprintf(temp, "%d", p[i]);
		path += temp;
		path += "-";
	}
	return path;
}

std::string pathToString(std::vector<element::Edge> p) {
	char temp[30];
	std::string path = "";
	for (unsigned int i = 0; i < p.size(); i++) {
		sprintf(temp, "%d", p[i].m_source);
		path += temp;
		path += "-";
		sprintf(temp, "%d", p[i].m_sink);
		path += temp;
		path += ";";
	}
	return path;
}

bool are_the_paths_equals(path p, path q) {
	bool result = true;
	if (p.size() != q.size()) {
		result = false;
	} else {
		for (unsigned int i = 0; i < p.size(); i++) {
			if (p[i] != q[i]) {
				result = false;
				break;
			}
		}
	}
	return result;
}

namespace algorithm {

	LBRJrcar::LBRJrcar() {

	}

	LBRJrcar::~LBRJrcar() {

	}
	
	LBRJrcar::LBRJrcar(int nNodes, double stretchFactor, element::Graph * graph) {
		m_nNodes = nNodes;
		m_stretchFactor = stretchFactor;
		this->m_graph = graph;
		m_E = this->m_graph->E;

		m_dijkstra.m_graph = this->m_graph->listEdges;
		m_dijkstra.m_dist.resize(nNodes);
		m_dijkstra.m_previous.resize(nNodes);
		m_dijkstra.m_nVertices = nNodes;

		m_longestPath = 0;
		m_nPathChanges = 0;
		m_nGatewayUpdates = 0;
		m_pE = m_E;
	}

	LBRJrcar::LBRJrcar(int nNodes, std::vector<element::Edge> E,
			double stretchFactor, std::vector<std::vector<element::Edge> > graph) {
		m_nNodes = nNodes;
		m_E = E;
		m_stretchFactor = stretchFactor;

		m_dijkstra.m_graph = graph;
		m_dijkstra.m_dist.resize(nNodes);
		m_dijkstra.m_previous.resize(nNodes);
		m_dijkstra.m_nVertices = nNodes;

		m_longestPath = 0;
		m_nPathChanges = 0;
		m_nGatewayUpdates = 0;
		m_pE = E;
	}

	std::vector<element::Flow> LBRJrcar::SortFlows(
			std::vector<element::Flow> flows) {
		for (unsigned int i = 0; i < flows.size() - 1; i++) {
			int min = i;
			for (unsigned int j = i + 1; j < flows.size(); j++) {
				if (m_candidatesPaths[flows[j].m_sourceSink].size()
						< m_candidatesPaths[flows[min].m_sourceSink].size()) {
					min = j;
				}
			}

			element::Flow temp = flows[min];
			flows[min] = flows[i];
			flows[i] = temp;
		}

		NS_LOG_INFO("sorted flows: " << flows.size());
		for (unsigned int i = 0; i < flows.size(); i++) {
			NS_LOG_INFO(
					flows[i].to_string() << " -- " << "m_candidatesPaths " << m_candidatesPaths[ flows[i].m_sourceSink ].size());
		}

		return flows;
	}

	path LBRJrcar::AddFlowF(element::Flow f) {		
		std::string msg;
		path shortestPath = m_candidatesPaths[f.m_sourceSink][0]; //select a temporary path.
		
		m_flows.push_back(f);
		m_flowsPaths[f.to_string()] = shortestPath;

		msg = "ADD flow: ";
		msg += f.to_string();
		//msg += " shortestPath "; msg += this->pathToString(shortestPath);
		NS_LOG_INFO(msg);

		return shortestPath;
	}

	/*void LBRJrcar::RemFlowF (Flow f) {
	 std::string msg;

	 for (unsigned i = 0; i < m_flows.size(); i++) {
	 Flow w = m_flows[i];
	 if (w.m_sourceSink == f.m_sourceSink &&
	 w.m_sourcePort == f.m_sourcePort &&
	 w.m_sinkPort == f.m_sinkPort) {
	 m_flows.erase(m_flows.begin() + i);
	 break;
	 }
	 }

	 msg = "REM flow: "; msg += f.to_string();
	 NS_LOG_INFO(msg);
	 }*/

	void LBRJrcar::RemFlowF(element::Flow f) {
		std::string msg;

		for (unsigned i = 0; i < m_flows.size(); i++) {
			element::Flow w = m_flows[i];
			if (w.m_sourceSink == f.m_sourceSink && w.m_sinkPort == f.m_sinkPort) {
				f = w;
				m_flows.erase(m_flows.begin() + i);
				break;
			}
		}

		msg = "REM flow: ";
		msg += f.to_string();
		NS_LOG_INFO(msg);
	}

	void LBRJrcar::Run() {
		std::string msg;
		char temp[10];

		for (unsigned int i = 0; i < m_E.size(); i++) {
			m_E[i].m_flows.clear();
			m_E[i].m_utilization = 0;
		}

		for (unsigned int i = 0; i < m_U.size(); i++) {
			m_U[i].m_flows.clear();
			m_U[i].m_utilization = 0;
		}

		msg = "JRCAR LOAD BALANCING -";
		msg += " N_FLOWS ";
		sprintf(temp, "%d", (int) m_flows.size());
		msg += temp;
		NS_LOG_INFO(msg);

		if (m_flows.size() == 0) {
			return;
		}

		std::vector<element::Flow> flows = SortFlows(m_flows);
		for (unsigned int i = 0; i < flows.size(); i++) {
			element::Flow f = flows[i];

			msg = "ADD flow: ";
			msg += f.to_string();

			Time currentTime = Simulator::Now();
			msg += " -- currentTime = ";
			sprintf(temp, "%lf", currentTime.GetSeconds());
			msg += temp;
			Time flowModifyTime = m_flowsModifyTime[f.to_string()];
			msg += " -- flowModifyTime = ";
			sprintf(temp, "%lf", flowModifyTime.GetSeconds());
			msg += temp;
			if (flowModifyTime.GetSeconds() != 0
					&& currentTime.GetSeconds() - flowModifyTime.GetSeconds()
							<= 2.0) {
				msg += " -- FLOW IS LOOKED";
				NS_LOG_INFO(msg);
				UpdateUtilizationAddFlow(m_flowsPaths[f.to_string()], f);
				continue;
			}
			NS_LOG_INFO(msg);

			path p = MinBL(m_candidatesPaths[f.m_sourceSink]);
			UpdateUtilizationAddFlow(p, f);

			msg = "currentPath = ";
			//msg += pathToString(m_flowsPaths[f.to_string()]);
			msg += " -- newPath = ";
			msg += pathToString(p);
			if (m_flowsModifyTime[f.to_string()].GetSeconds() == 0
					|| are_the_paths_equals(p, m_flowsPaths[f.to_string()])
							== false) {
				msg += " -- flow modified ADDING LOOK";
				NS_LOG_INFO(msg);
				m_flowsModifyTime[f.to_string()] = currentTime;
			} else {
				NS_LOG_INFO(msg);
			}
			m_flowsPaths[f.to_string()] = p;
		}
	}
} /* namespace algorithm */
