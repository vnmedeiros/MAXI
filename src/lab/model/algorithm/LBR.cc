/*
 * LBR.cpp
 *
 *  Created on: Nov 4, 2016
 *      Author: vinicius
 */

#include "LBR.h"

NS_LOG_COMPONENT_DEFINE("ALGORITHM_LBR");

namespace algorithm {

	LBR::LBR() {
	}

	LBR::LBR(int nNodes, std::vector<element::Edge> E, double stretchFactor, std::vector<std::vector<element::Edge> > graph) {
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

	LBR::~LBR() {
	}

	path * LBR::takeRoute(element::Flow flow) {
		int source = flow.m_sourceSink.first;
		int sourcePort = flow.m_sourcePort;
		int sink = flow.m_sourceSink.second;
		int sinkPort = flow.m_sinkPort;
		path bp;
		
				
		if (source == 0 && sourcePort != -1) {
			element::Flow f(source, sink, sourcePort, sinkPort);
			NS_LOG_LOGIC("flow: " << f.to_string());
			bp = this->m_flowsPaths[f.to_string()];
			if (bp.size() == 0) {
				bp = this->AddFlowF(f);
			}
		} else if (source != 0 && sourcePort != -1) {
			element::Flow f(sink, source, sinkPort, sourcePort);
			NS_LOG_LOGIC("flow: " << f.to_string());
			bp = this->m_flowsPaths[f.to_string()];
			if (bp.size() == 0) {
				bp = this->AddFlowF(f);
			}			
		}		
		path * bestPath = bp.size() == 0 ? NULL : new path(bp);
		return bestPath;
	}

	path * LBR::ShortPath(int verticeSource, int verticeDestination) {
		std::vector<path> paths = this->ShortestPaths(verticeSource, verticeDestination);
		return &(paths[0]);
	}

	element::Graph * LBR::getGraph() {
		element::Graph * g = new element::Graph();
		g->listEdges = m_dijkstra.m_graph;
		g->E = this->m_E;
		g->U = this->m_U;
		return g;
	}

	path LBR::AddFlowF(element::Flow f) {
		path p;
		return p;
	}

	void LBR::RemFlowF(element::Flow f) {
	}

	void LBR::Run() {
	}

	std::vector<path> LBR::ShortestPaths(int verticeSource, int verticeSink) {
		return m_dijkstra.Run(verticeSource, verticeSink);
	}

	path LBR::Concatenate(path p, path q) {
		for (unsigned int i = 1; i < q.size(); i++) {
			p.push_back(q[i]);
		}

		return p;
	}

	bool LBR::HasLoops(path p) {
		bool result = false;
		int counter[m_nNodes];

		for (int i = 0; i < m_nNodes; i++) {
			counter[i] = 0;
		}

		for (unsigned int i = 0; i < p.size(); i++) {
			counter[p[i]]++;
			if (counter[p[i]] > 1) {
				result = true;
				break;
			}
		}

		return result;
	}

	std::vector<path> LBR::ReversePaths(std::vector<path> paths) {
		std::vector<path> reversed;

		for (int i = paths.size() - 1; i > -1; i--) {
			reversed.push_back(paths[i]);
		}

		return reversed;
	}

	bool SortPathsCCP(path p, path q) {
		return (p.size() < q.size());
	}

	void LBR::CalculateCandidatesPaths() {
		int I = 0;
		for (int n = 1; n < m_nNodes; n++) {
			std::vector<path> paths = ShortestPaths(I, n);
			for (int t = 1; t < m_nNodes; t++) {
				if (t != n) {
					std::vector<path> pathsP = ShortestPaths(I, t);
					std::vector<path> pathsQ = ShortestPaths(t, n);
					for (unsigned int p = 0; p < pathsP.size(); p++) {
						for (unsigned int q = 0; q < pathsQ.size(); q++) {
							path pathP = pathsP[p];
							path pathQ = pathsQ[q];
							path p = Concatenate(pathP, pathQ);
							int lenP = p.size() - 1;
							int lenSmall = paths[0].size() - 1;
							if (HasLoops(p) == false && lenP <= m_stretchFactor * lenSmall) {
								paths.push_back(p);
								if (lenP >= m_longestPath) {
									m_longestPath = lenP;
								}
							}
						}
					}
				}
			}
			std::sort(paths.begin(), paths.end(), SortPathsCCP);
			m_candidatesPaths.insert(std::pair<FlowSourceSink, std::vector<path> >(FlowSourceSink(I, n), paths));
			m_candidatesPathsReverse.insert(std::pair<FlowSourceSink, std::vector<path> >(FlowSourceSink(n, I), ReversePaths(paths)));
		}
	}

	path LBR::MinBL(std::vector<path> paths) {
		path bestPath;
		int minTmn = 1000000000;

		for (unsigned int i = 0; i < paths.size(); i++) {
			path p = paths[i];
			int maxTmn = 0;
			for (unsigned int j = 0; j < p.size() - 1; j++) {
				int m = p[j];
				int n = p[j + 1];
				int tmn = 0;
				for (unsigned int k = 0; k < m_E.size(); k++) {
					if (m_E[k].m_source == m && m_E[k].m_sink == n) {
						tmn = m_E[k].m_flows.size();
						break;
					}
				}

				if (tmn > maxTmn) {
					maxTmn = tmn;
				}
			}

			if (maxTmn < minTmn) {
				minTmn = maxTmn;
				bestPath = p;
			}
		}

		return bestPath;
	}

	void LBR::UpdateUtilizationAddFlow(path p, element::Flow f) {
		for (unsigned int i = 0; i < p.size() - 1; i++) {
			int m = p[i];
			int n = p[i + 1];
			for (unsigned int j = 0; j < m_E.size(); j++) {
				if (m_E[j].m_source == m && m_E[j].m_sink == n) {
					m_E[j].m_flows.push_back(f);
					break;
				}
			}
		}

		for (unsigned int i = 0; i < p.size() - 1; i++) {
			int m = p[i];
			int n = p[i + 1];
			for (unsigned int j = 0; j < m_U.size(); j++) {
				if ((m_U[j].m_source == m && m_U[j].m_sink == n)
						|| (m_U[j].m_source == n && m_U[j].m_sink == m)) {
					m_U[j].m_flows.push_back(f);
					break;
				}
			}
		}
	}

	std::vector<element::Edge> LBR::UpdateUtilizationRemoveFlow(element::Flow f) {
		std::vector<element::Edge> p;

		for (unsigned int i = 0; i < m_E.size(); i++) {
			std::vector<element::Flow> flows = m_E[i].m_flows;
			for (unsigned j = 0; j < flows.size(); j++) {
				element::Flow w = flows[j];
				if (w.m_sourceSink == f.m_sourceSink
						&& w.m_sourcePort == f.m_sourcePort
						&& w.m_sinkPort == f.m_sinkPort) {
					m_E[i].m_flows.erase(m_E[i].m_flows.begin() + j);
					p.push_back(m_E[i]); //modified
					break;
				}
			}
		}

		for (unsigned int i = 0; i < m_U.size(); i++) {
			std::vector<element::Flow> flows = m_U[i].m_flows;
			for (unsigned j = 0; j < flows.size(); j++) {
				element::Flow w = flows[j];
				if (w.m_sourceSink == f.m_sourceSink
						&& w.m_sourcePort == f.m_sourcePort
						&& w.m_sinkPort == f.m_sinkPort) {
					m_U[i].m_flows.erase(m_U[i].m_flows.begin() + j);
					break;
				}
			}
		}

		return p;
	}

	std::vector<element::Edge> LBR::UpdateUtilizationRemoveFlow(element::Flow *f) {
		std::vector<element::Edge> p;

		for (unsigned int i = 0; i < m_E.size(); i++) {
			std::vector<element::Flow> flows = m_E[i].m_flows;
			for (unsigned j = 0; j < flows.size(); j++) {
				element::Flow w = flows[j];
				if (w.m_sourceSink == f->m_sourceSink
						&& w.m_sinkPort == f->m_sinkPort) {
					m_E[i].m_flows.erase(m_E[i].m_flows.begin() + j);
					p.push_back(m_E[i]); //modified
					break;
				}
			}
		}

		for (unsigned int i = 0; i < m_U.size(); i++) {
			std::vector<element::Flow> flows = m_U[i].m_flows;
			for (unsigned j = 0; j < flows.size(); j++) {
				element::Flow w = flows[j];
				if (w.m_sourceSink == f->m_sourceSink && w.m_sinkPort == f->m_sinkPort) {
					m_U[i].m_flows.erase(m_U[i].m_flows.begin() + j);
					break;
				}
			}
		}

		return p;
	}

	void LBR::DetectChanges(std::vector<element::Flow> flows) {
		std::string msg;
		char temp[30];
		bool changed;

		//NS_LOG_INFO("DETECT CHANGES");

		int operationPathChanges = 0;
		int operationGatewayUpdates = 0;
		for (unsigned int i = 0; i < flows.size(); i++) {
			element::Flow f = flows[i];
			msg = "source ";
			sprintf(temp, "%d", f.m_sourceSink.first);
			msg += temp;
			msg += " sink ";
			sprintf(temp, "%d", f.m_sourceSink.second);
			msg += temp;
			msg += " source-port ";
			sprintf(temp, "%d", f.m_sourcePort);
			msg += temp;
			msg += " sink-port ";
			sprintf(temp, "%d", f.m_sinkPort);
			msg += temp;
			//NS_LOG_INFO(msg);

			msg = "currentPath ";
			std::vector<element::Edge> currentPath;
			for (unsigned int j = 0; j < m_E.size(); j++) {
				element::Edge e = m_E[j];
				for (unsigned int k = 0; k < e.m_flows.size(); k++) {
					element::Flow w = e.m_flows[k];
					if (w.m_sourceSink == f.m_sourceSink
							&& w.m_sourcePort == f.m_sourcePort
							&& w.m_sinkPort == f.m_sinkPort) {
						currentPath.push_back(e);
						sprintf(temp, "%d", e.m_source);
						msg += temp;
						msg += "-";
						sprintf(temp, "%d", e.m_sink);
						msg += temp;
						msg += " ";
						break;
					}
				}
			}
			//NS_LOG_INFO(msg);

			msg = "previousPath ";
			std::vector<element::Edge> previousPath;
			for (unsigned int j = 0; j < m_pE.size(); j++) {
				element::Edge e = m_pE[j];
				for (unsigned int k = 0; k < e.m_flows.size(); k++) {
					element::Flow w = e.m_flows[k];
					if (w.m_sourceSink == f.m_sourceSink
							&& w.m_sourcePort == f.m_sourcePort
							&& w.m_sinkPort == f.m_sinkPort) {
						previousPath.push_back(e);
						sprintf(temp, "%d", e.m_source);
						msg += temp;
						msg += "-";
						sprintf(temp, "%d", e.m_sink);
						msg += temp;
						msg += " ";
						break;
					}
				}
			}
			//NS_LOG_INFO(msg);

			//detect path changes
			changed = false;
			if (currentPath.size() < previousPath.size()) {
				changed = true;
				m_nPathChanges++;
				operationPathChanges++;
				msg = "CHANGED";
				msg += " m_nPathChanges = ";
				sprintf(temp, "%d", m_nPathChanges);
				msg += temp;
				//NS_LOG_INFO(msg);
			} else if (currentPath.size() > previousPath.size()) {
				changed = true;
				m_nPathChanges++;
				operationPathChanges++;
				msg = "CHANGED";
				msg += " m_nPathChanges = ";
				sprintf(temp, "%d", m_nPathChanges);
				msg += temp;
				//NS_LOG_INFO(msg);
			} else {
				for (unsigned int j = 0; j < currentPath.size(); j++) {
					if (currentPath[j].m_source != previousPath[j].m_source
							|| currentPath[j].m_sink != previousPath[j].m_sink) {
						m_nPathChanges++;
						operationPathChanges++;
						changed = true;
						break;
					}
				}

				if (changed == true) {
					msg = "CHANGED";
				} else {
					msg = "NOT CHANGED";
				}
				msg += " m_nPathChanges = ";
				sprintf(temp, "%d", m_nPathChanges);
				msg += temp;
				//NS_LOG_INFO(msg);
			}

			//detect mesh routers updated
			std::set<int> updatedGateways;
			for (unsigned int i = 0; i < currentPath.size(); i++) {
				for (unsigned int j = 0; j < previousPath.size(); j++) {
					if (currentPath[i].m_source == previousPath[j].m_source
							&& currentPath[i].m_sink == previousPath[j].m_sink) {
						currentPath.erase(currentPath.begin() + i);
						previousPath.erase(previousPath.begin() + j);
						i = -1;
						break;
					}
				}
			}

			for (unsigned int i = 0; i < currentPath.size(); i++) {
				updatedGateways.insert(currentPath[i].m_source);
				updatedGateways.insert(currentPath[i].m_sink);
			}
			for (unsigned int i = 0; i < previousPath.size(); i++) {
				updatedGateways.insert(previousPath[i].m_source);
				updatedGateways.insert(previousPath[i].m_sink);
			}

			msg = "updated gateways:";
			std::set<int>::iterator ugIT;
			for (ugIT = updatedGateways.begin(); ugIT != updatedGateways.end();
					++ugIT) {
				msg += " ";
				sprintf(temp, "%d", *ugIT);
				msg += temp;
			}
			//NS_LOG_INFO(msg);

			m_nGatewayUpdates += updatedGateways.size();
			operationGatewayUpdates += updatedGateways.size();

			msg = "m_nGatewayUpdates = ";
			sprintf(temp, "%d", m_nGatewayUpdates);
			msg += temp;
			//NS_LOG_INFO(msg);
		}

		msg = "m_nPathChanges = ";
		sprintf(temp, "%d", m_nPathChanges);
		msg += temp;
		//NS_LOG_INFO(msg);

		msg = "operationPathChanges = ";
		sprintf(temp, "%d", operationPathChanges);
		msg += temp;
		//NS_LOG_INFO(msg);

		m_operationNPathChanges.push_back(operationPathChanges);

		msg = "m_nGatewayChanges = ";
		sprintf(temp, "%d", m_nGatewayUpdates);
		msg += temp;
		//NS_LOG_INFO(msg);

		msg = "operationGatewayUpdates = ";
		sprintf(temp, "%d", operationGatewayUpdates);
		msg += temp;
		//NS_LOG_INFO(msg);

		m_operationNGatewayUpdates.push_back(operationGatewayUpdates);

		for (unsigned int i = 0; i < m_E.size(); i++) {
			m_pE[i] = m_E[i];
		}
	}
} /* namespace */
