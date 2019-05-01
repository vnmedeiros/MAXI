/*
 * graph.h
 *
 *  Created on: Nov 17, 2016
 *      Author: vinicius
 */

#ifndef SRC_LAB_MODEL_ELEMENT_GRAPH_H_
#define SRC_LAB_MODEL_ELEMENT_GRAPH_H_

#include <vector>
#include <algorithm>
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/edge.h"
#include "ns3/lab.h"

namespace element {
	class Graph {
	public:
		Graph();
		Graph(int * matrix, uint32_t  nNodes);
		~Graph();
		std::vector< std::vector<element::Edge> > createAdjacencyList(int * matrix, uint32_t  nNodes);
		std::vector<element::Edge> createUndirectedEdgeList(int * matrix, uint32_t nNodes);
		std::vector<element::Edge> createDirectedEdgeList(int * matrix,	uint32_t nNodes);
		bool qualityLinkUpdate(int sourceVertice, int destinationVertice, double quality);
		bool existEdge(int sourceVertice, int destinationVertice);
		element::Edge * getEdge(int sourceVertice, int destinationVertice);
		void showAdjacencyList();
		int getNumberVertices();
		void updateEdgesMatrix(int * matrix, uint32_t nNodes, int min);

		std::vector< std::vector< element::Edge > > listEdges;
		std::vector<element::Edge> U;
		std::vector<element::Edge> E;

		void (*m_callbackUpdateLink)() = 0;

	};
}
#endif /* SRC_LAB_MODEL_ELEMENT_GRAPH_H_ */
