/*
 * Dijkstra.h
 *
 *  Created on: Nov 4, 2016
 *      Author: vinicius
 */

#ifndef SRC_LAB_MODEL_ALGORITHM_DIJKSTRA_H_
#define SRC_LAB_MODEL_ALGORITHM_DIJKSTRA_H_

#include <vector>
#include "ns3/object.h"
#include "ns3/core-module.h"
#include "ns3/edge.h"

namespace utils {

	/*
	 * Define path
	 */
	typedef std::vector<int> path;

	class Dijkstra {
	public:
		class Dist {
		public:
			bool m_isOpen; //Is the edge open?
			double m_weight;//Weight
			Dist():m_isOpen(-1),m_weight(0.0){}
			Dist(bool isOpen, double weight){
				m_isOpen = isOpen;
				m_weight = weight;
			}
			~Dist(){}
		};

		std::vector< std::vector< element::Edge > > m_graph; //Adjacency list
		std::vector< Dijkstra::Dist > m_dist;//Dists vector
		std::vector< std::vector< path > > m_previous;//Previous vector
		int m_nVertices;//Number of vertices

		std::vector<path> Run (int initialVertice, int finalVertice);//Start point
		void ShowDist ();//Show Dists vector
		void ShowGraph ();//Show Adjacency list

	private:
		int ExtractVerticeWithMinDistance();//Extract vertice with min distance
		void Initializations();//Do the initializations
	};

} /* namespace */

#endif /* SRC_LAB_MODEL_ALGORITHM_DIJKSTRA_H_ */
