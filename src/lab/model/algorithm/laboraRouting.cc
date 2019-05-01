/*
 * routing.cc
 *
 *  Created on: Nov 30, 2016
 *      Author: Vinícius Nunes Medeiros,
 *      LABORA(INF/UFG) - Instituto de informática (INF)
 *      				  Universidade federal de Goias (UFG)
 */

#include "laboraRouting.h"

namespace algorithm {

	LABORARouting::LABORARouting() {
	}

	LABORARouting::~LABORARouting() {}

	path * LABORARouting::ShortPath(int verticeSource, int verticeDestination) {
		if (verticeSource == verticeDestination) {
			path * r = new path(1, verticeSource);
			return r;
		}
		int n = this->m_graph->getNumberVertices();

		// Create a vector for distances and initialize all
		// distances as infinite (INF) and vector parents
		std::vector<int> dist(n, INF);
		std::vector<int> parent(n, -1);
		std::priority_queue<Node, std::vector<Node>, std::greater<Node> > pq;

		// Insert source itself in priority queue and initialize
		// its distance as 0.
		pq.push(std::make_pair(0, verticeSource));
		dist[verticeSource] = 0;
		parent[verticeSource] = verticeSource;
		/* Looping till priority queue becomes empty (or all distances are not finalized) */
		while (!pq.empty()) {
		// The first vertex in pair is the minimum distance
		// vertex, extract it from priority queue.
		// vertex label is stored in second of pair (it
		// has to be done this way to keep the vertices
		// sorted distance (distance must be first item
		// in pair)
			int u = pq.top().second;
			pq.pop();

			// 'i' is used to get all adjacent vertices of a vertex
			//list<pair<int, int> >::iterator i;
			std::vector<element::Edge> adj = this->m_graph->listEdges[u];
			for (unsigned int i = 0; i < adj.size(); i++) {
				// Get vertex label and weight of current adjacent of u.
				int v = adj[i].m_sink;
				int weight = 1;

				//  If there is shorted path to v through u.
				if (dist[v] > dist[u] + weight) {
					// Updating distance of v
					dist[v] = dist[u] + weight;
					pq.push(std::make_pair(dist[v], v));
					parent[v] = u;
				}
			}
		}

		path * r = new std::vector<int>();
		for (int i = verticeDestination; i != verticeSource; i = parent[i]) {
			if (i == -1) {
				return NULL;
			}
			r->push_back(i);
		}
		r->push_back(verticeSource);

		return r;
	}

	void LABORARouting::setGraph(element::Graph * graph) {
		this->m_graph = graph;
	}

	element::Graph * LABORARouting::getGraph() {
		return this->m_graph;
	}

	path * LABORARouting::takeRoute(element::Flow flow) {
		return this->ShortPath(flow.m_sourceSink.first, flow.m_sourceSink.second);
	}
}
