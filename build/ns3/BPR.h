/*
 * BPR.h
 *
 *  Created on: Nov 4, 2016
 *      Author: vinicius
 */

#ifndef SRC_LAB_MODEL_ALGORITHM_BPR_H_
#define SRC_LAB_MODEL_ALGORITHM_BPR_H_

#include <vector>
#include "ns3/simulator.h"
#include "ns3/object.h"
#include "ns3/core-module.h"
#include "ns3/log.h"

#include "ns3/lab.h"
#include "ns3/edge.h"
#include "ns3/flow.h"
#include "ns3/LBR.h"

namespace algorithm {
	class BPR : public LBR {
	public:
		BPR();
		BPR (int nNodes, std::vector<element::Edge> E, double stretchFactor,
				std::vector< std::vector< element::Edge > > graph);
		BPR(int nNodes, double stretchFactor, element::Graph * graph);
		virtual ~BPR();

		element::Edge ExtractBottleneck (); 								//Extract the network bottleneck
		std::vector<element::Edge> ExtractBottlenecks ();					//Extract the network bottlenecks
		element::Edge ExtractPathBottleneck (element::Flow f);				//Extract the path bottleneck
		element::Edge ExtractPathBottleneck (std::vector<element::Edge> p);	//Extract the path bottleneck
		virtual path AddFlowF (element::Flow f);
		virtual void RemFlowF (element::Flow f);
		virtual void Run ();
	};

}  // namespace algorithm

#endif /* SRC_LAB_MODEL_ALGORITHM_BPR_H_ */
