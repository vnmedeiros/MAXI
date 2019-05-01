/*
 * LBR.h
 *
 *  Created on: Nov 4, 2016
 *      Author: vinicius
 */

#ifndef SRC_LAB_MODEL_ALGORITHM_LBR_H_
#define SRC_LAB_MODEL_ALGORITHM_LBR_H_

#include <vector>
#include <algorithm>

#include "ns3/object.h"
#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/nstime.h"

#include "ns3/lab.h"
#include "ns3/laboraRouting.h"
#include "ns3/edge.h"
#include "ns3/flow.h"
#include "ns3/dijkstra.h"

namespace algorithm {

	class LBR : public LABORARouting {
	public:
		LBR();
		LBR(int nNodes, std::vector<element::Edge> E, double stretchFactor, std::vector< std::vector<element::Edge> > graph);
		~LBR();

		virtual path * takeRoute(element::Flow flow);
		path * ShortPath(int verticeSource, int verticeDestination);
		virtual path AddFlowF (element::Flow f);
		virtual void RemFlowF (element::Flow f);
		virtual void Run ();

		std::vector<path> ShortestPaths (int verticeSource, int verticeSink); //Calculate shortest paths
		path Concatenate (path p, path q);//Concatenate path p and q
		bool HasLoops (path p);//Has the path p loops?
		std::vector<path> ReversePaths (std::vector<path> paths);//Generate reverse paths
		void CalculateCandidatesPaths ();//Calculate the candidates paths

		int m_nNodes;//Number of nodes
		std::vector<element::Edge> m_E;//Directed Edge list
		std::vector<element::Edge> m_U;//Undirected Edge list
		double m_stretchFactor;//Stretch factor to calculate the candidates paths

		std::map< FlowSourceSink, std::vector<path> > m_candidatesPaths;//Candidates paths I-to-n
		std::map< FlowSourceSink, std::vector<path> > m_candidatesPathsReverse;//Reverse candidates paths n-to-I

		std::map< std::string, path> m_flowsPaths;//Map each flow to a path
		std::vector<element::Flow> m_flows;//Flows list
		std::map< std::string, double> m_flowsThroughputMbps;
		std::map< std::string, ns3::Time> m_flowsModifyTime;

		double m_longestPath;//longest path
		int m_nPathChanges;//Number of path changes
		int m_nGatewayUpdates;//Number of gateway updates
		std::vector<int> m_operationNPathChanges;//Number of path changes in each operation
		std::vector<int> m_operationNGatewayUpdates;//Number of gateway changes in each operation

		//TODO: gato para bular o acoplamento gerado pela classe utils::Dijkstra.
		element::Graph * getGraph();
	protected:
		utils::Dijkstra m_dijkstra;//Dijkstra's algorithm

		path MinBL (std::vector<path> paths);//Calculate de path with minimum bottleneck
		void UpdateUtilizationAddFlow (path p, element::Flow f);//Update utilization adding a flow
		std::vector<element::Edge> UpdateUtilizationRemoveFlow (element::Flow f);//Update utilization removing a flow
		std::vector<element::Edge> UpdateUtilizationRemoveFlow (element::Flow *f);//Update utilization removing a flow

		std::vector<element::Edge> m_pE;//Previous Edge list
		void DetectChanges (std::vector<element::Flow> flows);//Compare E and pE to detect path changes and gateway updates
	};
}
#endif /* SRC_LAB_MODEL_ALGORITHM_LBR_H_ */
