/*
 * JRCAR.h
 *
 *  Created on: Nov 4, 2016
 *      Author: vinicius
 */

#ifndef SRC_LAB_MODEL_ALGORITHM_LBRJRCAR_H_
#define SRC_LAB_MODEL_ALGORITHM_LBRJRCAR_H_

#include <vector>
#include "ns3/object.h"
#include "ns3/core-module.h"
#include "ns3/log.h"

#include "ns3/lab.h"
#include "ns3/edge.h"
#include "ns3/flow.h"
#include "ns3/LBR.h"

namespace algorithm {
	class LBRJrcar : public LBR {
	public:
		LBRJrcar ();
		LBRJrcar (int nNodes, double stretchFactor, element::Graph * graph);
		LBRJrcar (int nNodes, std::vector< element::Edge > E,
				double stretchFactor, std::vector<std::vector<element::Edge>> graph);
		virtual ~LBRJrcar();

		std::vector<element::Flow> SortFlows (std::vector<element::Flow> flows); //Sort flows based on opportunities
		virtual path AddFlowF (element::Flow f);
		virtual void RemFlowF (element::Flow f);
		virtual void Run ();

	};
}
#endif /* SRC_LAB_MODEL_ALGORITHM_LBRJRCAR_H_ */
