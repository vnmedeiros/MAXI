/*
 * graph.cc
 *
 *  Created on: Nov 17, 2016
 *      Author: vinicius
 */

#include "graph.h"

namespace element {

	Graph::Graph(){}

	Graph::Graph(int * matrix, uint32_t nNodes) {
		this->listEdges = this->createAdjacencyList(matrix, nNodes);
		this->U = this->createUndirectedEdgeList(matrix, nNodes);
		this->E = this->createDirectedEdgeList(matrix, nNodes);
	}

	Graph::~Graph() {

	}

	std::vector<std::vector<element::Edge> > Graph::createAdjacencyList(int * matrix, uint32_t nNodes) {
		std::vector<std::vector<element::Edge> > graph;
		graph.resize(nNodes);
		//int *matrix = mobility.GetMatrix();
		for (uint32_t i = 0; i < nNodes; i++) {
			std::vector<element::Edge> edges;
			for (uint32_t j = 0; j < nNodes; j++) {
				if (matrix[i * nNodes + j] != 0) {
					edges.push_back(element::Edge(i, j, 1));
				}
			}
			graph[i] = edges;
		}
		return graph;
	}

	std::vector<element::Edge> Graph::createUndirectedEdgeList(int * matrix, uint32_t nNodes) {
		std::vector<element::Edge> U;
		//int *matrix = mobility.GetMatrix();
		for (uint32_t i = 0; i < nNodes; i++) {
			for (uint32_t j = 0; j < nNodes; j++) {
				if (i < j && matrix[i * nNodes + j] != 0) {
					U.push_back(element::Edge(i, j, 1));
				}
			}
		}
		return U;
	}

	std::vector<element::Edge> Graph::createDirectedEdgeList(int * matrix, uint32_t nNodes) {
		std::vector<element::Edge> E;
		//int *matrix = mobility.GetMatrix();
		for (uint32_t i = 0; i < nNodes; i++) {
			for (uint32_t j = 0; j < nNodes; j++) {
				if (matrix[i * nNodes + j] != 0) {
					E.push_back(element::Edge(i, j, 1));
				}
			}
		}
		return E;
	}

	bool Graph::qualityLinkUpdate(int sourceVertice, int destinationVertice, double quality) {
		element::Edge item(sourceVertice, destinationVertice, quality);
		std::vector<element::Edge>::iterator ed = std::find(this->listEdges[sourceVertice].begin(), this->listEdges[sourceVertice].end(), item);
		if (ed != this->listEdges[sourceVertice].end()) {
			if ( (*ed).addQualitySample(quality) == true ) {
				//if ( !m_callbackUpdateLink.IsNull () )
				if (m_callbackUpdateLink != 0)
					m_callbackUpdateLink();
				return true;
			}
		}
		return false;
	}

	bool Graph::existEdge(int sourceVertice, int destinationVertice) {
		element::Edge item(sourceVertice, destinationVertice, 0.0);
		std::vector<element::Edge>::iterator ed = std::find(this->listEdges[sourceVertice].begin(), this->listEdges[sourceVertice].end(), item);
		return (ed != this->listEdges[sourceVertice].end());
	}

	element::Edge * Graph::getEdge(int sourceVertice, int destinationVertice) {
		element::Edge item(sourceVertice, destinationVertice, 0.0);
		std::vector<element::Edge>::iterator ed = std::find(this->listEdges[sourceVertice].begin(), this->listEdges[sourceVertice].end(), item);
		if ( ed != this->listEdges[sourceVertice].end() )
			return &*ed;
		return NULL;
	}

	void Graph::showAdjacencyList ( ) {
		printf ("Graph (verticeI -> verticeJ[FunctionWeight]):\n");
		for (unsigned int i = 0; i < this->listEdges.size (); i++) {
			printf ("vertice %d -> ", i);
			for (unsigned int j = 0; j < this->listEdges[i].size(); j++) {
				printf ("%d[%.2f, %d = %.2f, ch=%d] ", this->listEdges[i][j].m_sink, this->listEdges[i][j].quality, (int)this->listEdges[i][j].m_flows.size(), this->listEdges[i][j].functionAmount, this->listEdges[i][j].channel/*graph[i][j].m_weight*/);
			}
			printf ("\n");
		}
		printf ("\n");
	}

	int Graph::getNumberVertices() {
		return this->listEdges.size();
	}

	void Graph::updateEdgesMatrix(int * matrix, uint32_t nNodes, int min) {

//		std::cout << std::endl;
//		for (uint32_t i = 0; i < nNodes; i++) {
//			for (uint32_t j = 0; j < nNodes; j++) {
//				if (matrix[i * nNodes + j] > 0 /*&& map[j * sm.config.N_NODES + i] > 0 && i != j*/) {
//					std::cout << "map[" << i << " * " << nNodes << " + " << j << "] = " << matrix[i * nNodes + j] << ";";
//				}
//			}
//			std::cout << std::endl;
//		}
//		std::cout << std::endl;

		for (uint32_t src = 0; src < nNodes; src++) {
			std::vector<element::Edge>::iterator it;
			for (it = this->listEdges[src].begin();
				 it!= this->listEdges[src].end();) {
				if (matrix[src * nNodes + (*it).m_sink] < min) {
					element::Edge item(src, (*it).m_sink, 0.0);
					std::vector<element::Edge>::iterator ed;
					ed = std::find(this->U.begin(), this->U.end(), item);
					if (ed != this->U.end())
						this->U.erase(ed);

					ed = std::find(this->E.begin(), this->E.end(), item);
					if (ed != this->E.end())
						this->E.erase(ed);

					it = this->listEdges[src].erase(it);
				} else {
					++it;
				}
			}
		}
		return;
	}

} /* namespace element */
