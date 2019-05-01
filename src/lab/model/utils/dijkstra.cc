/*
 * Dijkstra.cc
 *
 *  Created on: Nov 4, 2016
 *      Author: vinicius
 */

#include "dijkstra.h"

namespace utils {

	void Dijkstra::ShowDist() {
		//NS_LOG_INFO("Dist vertice -> (1|0,cost):");
		for (unsigned int i = 0; i < m_dist.size(); i++) {
			//NS_LOG_INFO(i << " -> (" << m_dist[i].m_isOpen << "," << m_dist[i].m_weight << ")");
		}
	}

	void Dijkstra::ShowGraph() {
		//NS_LOG_INFO("Graph (verticeI -> verticeJ[weight]):");
		for (unsigned int i = 0; i < m_graph.size(); i++) {
			//NS_LOG_INFO("vertice " << i << " -> ");
			for (unsigned int j = 0; j < m_graph[i].size(); j++) {
				//NS_LOG_INFO(m_graph[i][j].m_sink << "[" << m_graph[i][j].m_weight << "] ");
			}
		}
	}

	int Dijkstra::ExtractVerticeWithMinDistance() {
		double min = 1000000, minIndex;
		for (unsigned int i = 0; i < m_dist.size(); i++) {
			if (m_dist[i].m_isOpen == true && m_dist[i].m_weight < min) {
				min = m_dist[i].m_weight;
				minIndex = i;
			}
		}
		return minIndex;
	}

	void Dijkstra::Initializations() {
		for (int i = 0; i < m_nVertices; i++) {
			m_dist[i] = Dijkstra::Dist(true, 1000000);
		}
		std::vector<std::vector<path>> previous;
		previous.resize(m_nVertices);
		m_previous = previous;
	}

	std::vector<path> Dijkstra::Run(int initialVertice, int finalVertice) {
		//ShowGraph();
		Initializations();
		m_dist[initialVertice] = Dist(true, 0);
		int visited = 0;
		while (visited != m_nVertices) {
			//ShowDist();
			int u = ExtractVerticeWithMinDistance();
			m_dist[u].m_isOpen = false;
			for (unsigned int i = 0; i < m_graph[u].size(); i++) {
				int v = m_graph[u][i].m_sink;
				int alt = m_dist[u].m_weight + m_graph[u][i].m_weight;
				if (alt < m_dist[v].m_weight) {
					m_dist[v].m_weight = alt;
					std::vector<path> paths = m_previous[u];
					if (paths.size() == 0) {
						path p;
						p.push_back(u);
						paths.push_back(p);
					} else {
						for (unsigned j = 0; j < paths.size(); j++) {
							paths[j].push_back(u);
						}
					}
					m_previous[v] = paths;
				} else if (alt == m_dist[v].m_weight) {
					std::vector<path> paths = m_previous[u];
					for (unsigned j = 0; j < paths.size(); j++) {
						paths[j].push_back(u);
						m_previous[v].push_back(paths[j]);
					}
				}
			}
			visited++;
		}

		for (unsigned i = 0; i < m_previous[finalVertice].size(); i++) {
			m_previous[finalVertice][i].push_back(finalVertice);
		}

		return m_previous[finalVertice];
	}
} /* namespace */
