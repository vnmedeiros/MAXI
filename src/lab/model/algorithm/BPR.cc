/*
 * BPR.cpp
 *
 *  Created on: Nov 4, 2016
 *      Author: vinicius
 */

#include "BPR.h"

using namespace ns3;

namespace algorithm {
//BPR
	BPR::BPR() {}
	BPR::BPR(int nNodes, std::vector<element::Edge> E, double stretchFactor,
			std::vector<std::vector<element::Edge> > graph) {
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

	BPR::BPR(int nNodes, double stretchFactor, element::Graph * graph) {
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

	BPR::~BPR() {}

	element::Edge BPR::ExtractBottleneck() {
		int max = -1;
		element::Edge e;
		for (unsigned int i = 0; i < m_E.size(); i++) {
			if ((int) m_E[i].m_flows.size() > max) {
				max = m_E[i].m_flows.size();
				e = m_E[i];
			}
		}

		return e;
	}

	std::vector<element::Edge> BPR::ExtractBottlenecks() {
		int max = -1;
		std::vector<element::Edge> bE;
		for (unsigned int i = 0; i < m_E.size(); i++) {
			int bottleneck = (int) m_E[i].m_flows.size();
			if (bottleneck > max) {
				max = m_E[i].m_flows.size();
				bE.clear();
				bE.resize(0);
				bE.push_back(m_E[i]);
			} else if (bottleneck == max) {
				bE.push_back(m_E[i]);
			}
		}

		return bE;
	}

	element::Edge BPR::ExtractPathBottleneck(element::Flow f) {
		int max = -1;
		element::Edge e;

		for (unsigned int i = 0; i < m_E.size(); i++) {
			std::vector<element::Flow> flows = m_E[i].m_flows;
			for (unsigned int j = 0; j < flows.size(); j++) {
				element::Flow w = flows[j];
				if (w.m_sourceSink == f.m_sourceSink &&
				//w.m_sourcePort == f.m_sourcePort &&
						w.m_sinkPort == f.m_sinkPort) {
					if ((int) m_E[i].m_flows.size() > max) {
						max = m_E[i].m_flows.size();
						e = m_E[i];
					}
					break;
				}
			}
		}

		return e;
	}

	element::Edge BPR::ExtractPathBottleneck(std::vector<element::Edge> p) {
		int max = -1;
		element::Edge e;

		for (unsigned int i = 0; i < p.size(); i++) {
			if ((int) p[i].m_flows.size() > max) {
				max = p[i].m_flows.size();
				e = p[i];
			}
		}

		return e;
	}

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

	path BPR::AddFlowF(element::Flow f) { //original with look
		std::string msg;
		char temp[30];
		path bestPath;
		element::Edge prevBottleneck = ExtractBottleneck();
		bestPath = MinBL(m_candidatesPaths[f.m_sourceSink]);
		UpdateUtilizationAddFlow(bestPath, f);
		element::Edge curBottleneck = ExtractBottleneck();

		m_flowsPaths[f.to_string()] = bestPath;

		msg = "\nADD flow ";
		msg += f.to_string();
		msg += " -- bestPath ";
		msg += pathToString(bestPath);
		msg += " -- prevBottleneck = ";
		sprintf(temp, "%d", (int) prevBottleneck.m_flows.size());
		msg += temp;
		msg += " -- curBottleneck = ";
		sprintf(temp, "%d", (int) curBottleneck.m_flows.size());
		msg += temp;
		//NS_LOG_INFO(msg);

		if (curBottleneck.m_flows.size() > prevBottleneck.m_flows.size()) {

			std::vector<element::Flow> flows = curBottleneck.m_flows;
			for (unsigned int i = 0; i < flows.size(); i++) {
				element::Flow ff = flows[i];
				if (ff == f) {
					continue;
				}

				msg = "    > RE-ROUTING flow ";
				msg += ff.to_string();
				Time currentTime = Simulator::Now();
				msg += " -- currentTime = ";
				sprintf(temp, "%lf", currentTime.GetSeconds());
				msg += temp;
				Time flowModifyTime = m_flowsModifyTime[ff.to_string()];
				msg += " -- flowModifyTime = ";
				sprintf(temp, "%lf", flowModifyTime.GetSeconds());
				msg += temp;
				if (flowModifyTime.GetSeconds() != 0
						&& currentTime.GetSeconds() - flowModifyTime.GetSeconds()
								<= 2.0) {
					msg += " -- FLOW IS LOOKED";
					//NS_LOG_INFO(msg);
					continue;
				}
				//NS_LOG_INFO(msg);

				std::vector<element::Edge> tempE = m_E;

				std::vector<element::Edge> remPath = UpdateUtilizationRemoveFlow(
						ff);
				path newPath = MinBL(m_candidatesPaths[ff.m_sourceSink]);
				UpdateUtilizationAddFlow(newPath, ff);
				element::Edge newBottleneck = ExtractBottleneck();

				msg = "    > newBottleneck = ";
				sprintf(temp, "%d", (int) newBottleneck.m_flows.size());
				msg += temp;
				msg += " -- curBottleneck = ";
				sprintf(temp, "%d", (int) curBottleneck.m_flows.size());
				msg += temp;
				if (newBottleneck.m_flows.size() > curBottleneck.m_flows.size()) {
					msg += " -- WORST";
				} else if (newBottleneck.m_flows.size()
						== curBottleneck.m_flows.size()) {
					msg += " -- SAME";
				} else {
					msg += " -- BETTER";
				}
				//NS_LOG_INFO(msg);

				int remPathSize = remPath.size();
				int newPathSize = newPath.size() - 1;
				msg = "    > remPathSize = ";
				sprintf(temp, "%d", remPathSize);
				msg += temp;
				msg += " -- newPathSize = ";
				sprintf(temp, "%d", newPathSize);
				msg += temp;
				if (newPathSize > remPathSize) {
					msg += " -- WORST";
				} else if (newPathSize == remPathSize) {
					msg += " -- SAME";
				} else {
					msg += " -- BETTER";
				}
				//NS_LOG_INFO(msg);

				if (!(newBottleneck.m_flows.size() < curBottleneck.m_flows.size())
						&& !(newPathSize < remPathSize)) {
					//NS_LOG_INFO("    > NOTHING TO DO");
					m_E = tempE;
					continue;
				}

				m_flowsPaths[ff.to_string()] = newPath;

				currentTime = Simulator::Now();
				m_flowsModifyTime[ff.to_string()] = currentTime;

				msg = "    > remPath ";
				msg += pathToString(remPath);
				msg += " -- newPath ";
				msg += pathToString(newPath);
				//NS_LOG_INFO(msg);

				if (newBottleneck.m_flows.size() < curBottleneck.m_flows.size()) {
					//NS_LOG_INFO("    > ADD BETTER bottleneck");
					break;
				}
			}
		}

		return bestPath;
	}

	/*path BPR::AddFlowF (Flow f) { //test
	 std::string msg;
	 char temp[30];
	 path bestPath;

	 Edge prevBottleneck = ExtractBottleneck();
	 bestPath = MinBL(m_candidatesPaths[ f.m_sourceSink ]);
	 UpdateUtilizationAddFlow(bestPath, f);
	 Edge curBottleneck = ExtractBottleneck();

	 m_flowsPaths[f.to_string()] = bestPath;

	 Time currentTime = Simulator::Now();
	 m_flowsModifyTime[f.to_string()] = currentTime;

	 msg = "\nADD flow "; msg += f.to_string();
	 msg += " -- bestPath "; msg += pathToString(bestPath);
	 msg += " -- prevBottleneck = ";
	 sprintf(temp, "%d", (int)prevBottleneck.m_flows.size()); msg += temp;
	 msg += " -- curBottleneck = ";
	 sprintf(temp, "%d", (int)curBottleneck.m_flows.size()); msg += temp;
	 NS_LOG_INFO(msg);

	 if (curBottleneck.m_flows.size() > prevBottleneck.m_flows.size()) {

	 std::vector<Flow> flows = curBottleneck.m_flows;
	 for (unsigned int i = 0; i < flows.size(); i++) {
	 Flow ff = flows[i];
	 if (ff == f) {
	 continue;
	 }
	 msg = "    > RE-ROUTING flow "; msg += ff.to_string();
	 NS_LOG_INFO(msg);

	 std::vector<Edge> tempE = m_E;

	 std::vector<Edge> remPath = UpdateUtilizationRemoveFlow(ff);
	 path newPath = MinBL(m_candidatesPaths[ ff.m_sourceSink ]);
	 UpdateUtilizationAddFlow(newPath, ff);
	 Edge newBottleneck = ExtractBottleneck();

	 msg = "    > newBottleneck = ";
	 sprintf(temp, "%d", (int)newBottleneck.m_flows.size()); msg += temp;
	 msg += " -- curBottleneck = ";
	 sprintf(temp, "%d", (int)curBottleneck.m_flows.size()); msg += temp;
	 if (newBottleneck.m_flows.size() > curBottleneck.m_flows.size()) {
	 msg += " -- WORST";
	 } else if (newBottleneck.m_flows.size() == curBottleneck.m_flows.size()) {
	 msg += " -- SAME";
	 } else {
	 msg += " -- BETTER";
	 }
	 NS_LOG_INFO(msg);

	 int remPathSize = remPath.size();
	 int newPathSize = newPath.size() - 1;
	 msg = "    > remPathSize = ";
	 sprintf(temp, "%d", remPathSize); msg += temp;
	 msg += " -- newPathSize = ";
	 sprintf(temp, "%d", newPathSize); msg += temp;
	 if (newPathSize > remPathSize) {
	 msg += " -- WORST";
	 } else if (newPathSize == remPathSize) {
	 msg += " -- SAME";
	 } else {
	 msg += " -- BETTER";
	 }
	 NS_LOG_INFO(msg);

	 if (!(newBottleneck.m_flows.size () < curBottleneck.m_flows.size ())
	 && !(newPathSize < remPathSize)) {
	 NS_LOG_INFO("    > NOTHING TO DO");
	 m_E = tempE;
	 continue;
	 } else {
	 m_flowsPaths[ff.to_string()] = newPath;

	 currentTime = Simulator::Now();
	 m_flowsModifyTime[ff.to_string()] = currentTime;

	 msg = "    > remPath "; msg += pathToString(remPath);
	 msg += " -- newPath "; msg += pathToString(newPath);
	 NS_LOG_INFO(msg);

	 break;
	 }
	 }
	 }

	 return bestPath;
	 }*/

	void BPR::RemFlowF(element::Flow f) { //original with look
		std::string msg;
		char temp[10];

		/*Edge prevBottleneck = ExtractBottleneck();
		 std::vector<Edge> remPath = UpdateUtilizationRemoveFlow(&f);
		 //std::vector<Edge> remPath = UpdateUtilizationRemoveFlow(f);
		 Edge pathBottleneck = ExtractPathBottleneck(remPath);*/

		std::vector<element::Edge> remPath = UpdateUtilizationRemoveFlow(&f);
		//std::vector<element::Edge> remPath = UpdateUtilizationRemoveFlow(f);
		element::Edge prevBottleneck = ExtractBottleneck();
		element::Edge pathBottleneck = ExtractPathBottleneck(remPath);

		msg = "\nREM flow ";
		msg += f.to_string();
		msg += " -- remPath ";
		msg += pathToString(remPath);
		msg += " -- prevBottleneck = ";
		sprintf(temp, "%d", (int) prevBottleneck.m_flows.size());
		msg += temp;
		msg += " -- pathBottleneck = ";
		sprintf(temp, "%d", (int) pathBottleneck.m_flows.size());
		msg += temp;
		//NS_LOG_INFO(msg);

		if (prevBottleneck.m_flows.size() - pathBottleneck.m_flows.size() == 2) {
			std::vector<element::Edge> bottlenecks = ExtractBottlenecks();

			bool moved = false;
			for (unsigned int k = 0; k < bottlenecks.size(); k++) {
				//NS_LOG_INFO("    > k = " << k << "/" << bottlenecks.size() - 1);
				element::Edge curBottleneck = bottlenecks[k];

				std::vector<element::Flow> flows = curBottleneck.m_flows;
				for (unsigned int i = 0; i < flows.size(); i++) {
					element::Flow ff = flows[i];

					msg = "        >> RE-ROUTING flow ";
					msg += ff.to_string();
					Time currentTime = Simulator::Now();
					msg += " -- currentTime = ";
					sprintf(temp, "%lf", currentTime.GetSeconds());
					msg += temp;
					Time flowModifyTime = m_flowsModifyTime[ff.to_string()];
					msg += " -- flowModifyTime = ";
					sprintf(temp, "%lf", flowModifyTime.GetSeconds());
					msg += temp;
					if (flowModifyTime.GetSeconds() != 0
							&& currentTime.GetSeconds()
									- flowModifyTime.GetSeconds() <= 2.0) {
						msg += " -- FLOW IS LOOKED";
						//NS_LOG_INFO(msg);
						continue;
					}
					//NS_LOG_INFO(msg);

					std::vector<element::Edge> tempE = m_E;

					remPath = UpdateUtilizationRemoveFlow(ff);
					path newPath = MinBL(m_candidatesPaths[ff.m_sourceSink]);
					UpdateUtilizationAddFlow(newPath, ff);

					element::Edge newPathBottleneck;
					for (unsigned int j = 0; j < m_E.size(); j++) {
						if (m_E[j].m_source == pathBottleneck.m_source
								&& m_E[j].m_sink == pathBottleneck.m_sink) {
							newPathBottleneck = m_E[j];
							break;
						}
					}
					msg = "        >> newPathBottleneck = ";
					sprintf(temp, "%d", (int) newPathBottleneck.m_flows.size());
					msg += temp;
					msg += " -- pathBottleneck = ";
					sprintf(temp, "%d", (int) pathBottleneck.m_flows.size());
					msg += temp;
					if (newPathBottleneck.m_flows.size()
							< pathBottleneck.m_flows.size()) {
						msg += " -- WORST";
					} else if (newPathBottleneck.m_flows.size()
							== pathBottleneck.m_flows.size()) {
						msg += " -- SAME";
					} else {
						msg += " -- BETTER";
					}
					//NS_LOG_INFO(msg);

					int remPathSize = remPath.size();
					int newPathSize = newPath.size() - 1;
					msg = "        >> remPathSize = ";
					sprintf(temp, "%d", remPathSize);
					msg += temp;
					msg += " -- newPathSize = ";
					sprintf(temp, "%d", newPathSize);
					msg += temp;
					if (newPathSize > remPathSize) {
						msg += " -- WORST";
					} else if (newPathSize == remPathSize) {
						msg += " -- SAME";
					} else {
						msg += " -- BETTER";
					}
					//NS_LOG_INFO(msg);

					if (!(newPathBottleneck.m_flows.size()
							> pathBottleneck.m_flows.size())
							&& !(newPathSize < remPathSize)) {
						//NS_LOG_INFO("    > NOTHING TO DO");
						m_E = tempE;
						continue;
					}

					m_flowsPaths[ff.to_string()] = newPath;

					currentTime = Simulator::Now();
					m_flowsModifyTime[ff.to_string()] = currentTime;

					msg = "        >> remPath ";
					msg += pathToString(remPath);
					msg += " -- newPath ";
					msg += pathToString(newPath);
					//NS_LOG_INFO(msg);

					if (newPathBottleneck.m_flows.size()
							> pathBottleneck.m_flows.size()) {
						//NS_LOG_INFO("        >> REM BETTER bottleneck");
						moved = true;
						break;
					}
				}

				if (moved == true) {
					break;
				}
			}
		}
	}

	/*void BPR::RemFlowF (Flow f) { //test with look
	 std::string msg;
	 char temp[10];

	 Edge prevBottleneck = ExtractBottleneck();
	 std::vector<Edge> remPath = UpdateUtilizationRemoveFlow(&f);
	 //std::vector<Edge> remPath = UpdateUtilizationRemoveFlow(f);
	 Edge pathBottleneck = ExtractPathBottleneck(remPath);

	 msg = "\nREM flow "; msg += f.to_string();
	 msg += " -- remPath "; msg += pathToString(remPath);
	 msg += " -- prevBottleneck = ";
	 sprintf(temp, "%d", (int)prevBottleneck.m_flows.size()); msg += temp;
	 msg += " -- pathBottleneck = ";
	 sprintf(temp, "%d", (int)pathBottleneck.m_flows.size()); msg += temp;
	 NS_LOG_INFO(msg);

	 if (prevBottleneck.m_flows.size() - pathBottleneck.m_flows.size() >= 2) {
	 std::vector<Edge> bottlenecks = ExtractBottlenecks();

	 bool moved = false;
	 for (unsigned int k = 0; k < bottlenecks.size(); k++) {
	 NS_LOG_INFO("    > k = " << k << "/" << bottlenecks.size() - 1);
	 Edge curBottleneck = bottlenecks[k];

	 std::vector<Flow> flows = curBottleneck.m_flows;
	 for (unsigned int i = 0; i < flows.size(); i++) {
	 Flow ff = flows[i];

	 msg = "        >> RE-ROUTING flow "; msg += ff.to_string();
	 Time currentTime = Simulator::Now();
	 msg += " -- currentTime = ";
	 sprintf(temp, "%lf", currentTime.GetSeconds()); msg += temp;
	 Time flowModifyTime = m_flowsModifyTime[ff.to_string()];
	 msg += " -- flowModifyTime = ";
	 sprintf(temp, "%lf", flowModifyTime.GetSeconds()); msg += temp;
	 if (flowModifyTime.GetSeconds() != 0
	 && currentTime.GetSeconds() - flowModifyTime.GetSeconds() <= 2.0) {
	 msg += " -- FLOW IS LOOKED";
	 NS_LOG_INFO(msg);
	 continue;
	 }
	 NS_LOG_INFO(msg);

	 std::vector<Edge> tempE = m_E;

	 remPath = UpdateUtilizationRemoveFlow(ff);
	 path newPath = MinBL(m_candidatesPaths[ ff.m_sourceSink ]);
	 UpdateUtilizationAddFlow(newPath, ff);

	 Edge newPathBottleneck;
	 for (unsigned int j = 0; j < m_E.size(); j++) {
	 if (m_E[j].m_source == pathBottleneck.m_source &&
	 m_E[j].m_sink == pathBottleneck.m_sink) {
	 newPathBottleneck = m_E[j];
	 break;
	 }
	 }
	 msg = "        >> newPathBottleneck = ";
	 sprintf(temp, "%d", (int)newPathBottleneck.m_flows.size());
	 msg += temp;
	 msg += " -- pathBottleneck = ";
	 sprintf(temp, "%d", (int)pathBottleneck.m_flows.size());
	 msg += temp;
	 if (newPathBottleneck.m_flows.size() < pathBottleneck.m_flows.size()) {
	 msg += " -- WORST";
	 } else if (newPathBottleneck.m_flows.size() == pathBottleneck.m_flows.size()) {
	 msg += " -- SAME";
	 } else {
	 msg += " -- BETTER";
	 }
	 NS_LOG_INFO(msg);

	 int remPathSize = remPath.size();
	 int newPathSize = newPath.size() - 1;
	 msg = "        >> remPathSize = ";
	 sprintf(temp, "%d", remPathSize); msg += temp;
	 msg += " -- newPathSize = ";
	 sprintf(temp, "%d", newPathSize); msg += temp;
	 if (newPathSize > remPathSize) {
	 msg += " -- WORST";
	 } else if (newPathSize == remPathSize) {
	 msg += " -- SAME";
	 } else {
	 msg += " -- BETTER";
	 }
	 NS_LOG_INFO(msg);

	 if (newPathSize > remPathSize) {
	 NS_LOG_INFO("        >> NOTHING TO DO");
	 m_E = tempE;
	 continue;
	 }

	 if (!(newPathBottleneck.m_flows.size() > pathBottleneck.m_flows.size())) {
	 NS_LOG_INFO("        >> NOTHING TO DO");
	 m_E = tempE;
	 continue;
	 } else {
	 m_flowsPaths[ff.to_string()] = newPath;

	 currentTime = Simulator::Now();
	 m_flowsModifyTime[ff.to_string()] = currentTime;

	 msg = "        >> remPath "; msg += pathToString(remPath);
	 msg += " -- newPath "; msg += pathToString(newPath);
	 NS_LOG_INFO(msg);

	 NS_LOG_INFO("        >> REM BETTER bottleneck");
	 moved = true;
	 break;
	 }
	 }

	 if (moved == true) {
	 break;
	 }
	 }
	 }
	 }*/

	void BPR::Run() {
		//BPR add and remove flows over demand
	}
} /* namespace algorithm */
