/*
 * routing.h
 *
 *  Created on: Nov 30, 2016
 *      Author: Vinícius Nunes Medeiros,
 *      LABORA(INF/UFG) - Instituto de informática (INF)
 *      				  Universidade federal de Goias (UFG)
 */

#ifndef SRC_LAB_ALGORITHM_ROUTING_H_
#define SRC_LAB_ALGORITHM_ROUTING_H_

#include <iostream>
#include <algorithm>
#include <utility>      // std::pair, std::make_pair
#include <queue>        // std::priority_queue
#include <map>
#include <vector>

#include "ns3/graph.h"
#include "ns3/lab.h"

namespace algorithm {
	class LABORARouting {

	protected:
		element::Graph * m_graph;

	public:
		LABORARouting();
		~LABORARouting();
		void setGraph(element::Graph * graph);
		element::Graph * getGraph();

		/*
		 * Calculate short path in hops verticeSource to verticeSink
		 * */
		path * ShortPath(int verticeSource, int verticeDestination);

		/*
		 * get route (path) to flow.
		 * return pointer to path or NULL, case not possible construct route.
		 * */
		virtual path * takeRoute(element::Flow flow);


		typedef std::pair<int, int> Node; // <functionWeight, vertice>
	};
}
#endif /* SRC_LAB_ALGORITHM_ROUTING_H_ */

