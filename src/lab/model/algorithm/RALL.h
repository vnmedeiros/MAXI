/*
 * RALL.h
 *
 *  Created on: Nov 6, 2016
 *      Author: vinicius
 */

#ifndef SRC_LAB_MODEL_ALGORITHM_RALL_H_
#define SRC_LAB_MODEL_ALGORITHM_RALL_H_
#define INF 0x3f3f3f3f

#include <iostream>
#include <iomanip>      // std::setw
#include <algorithm>
#include <utility>      // std::pair, std::make_pair
#include <queue>          // std::priority_queue
#include <map>
#include <vector>
#include "ns3/log.h"

#include "ns3/lab.h"
#include "ns3/labora-application-helper.h"
#include "ns3/laboraRouting.h"

namespace algorithm {
	class RALL : public LABORARouting {
	public:
		RALL(double weightPath, double weightLqi, element::Graph * graph);
		virtual ~RALL();
		virtual path * takeRoute(element::Flow flow);
		path * ShortPath(int verticeSource, int verticeDestination);
		//path * createRouteToFlow(int soucer, int sink);
		path * createRouteToFlow(element::Flow flow);
		void recreatesAllRoutes(void);
		path * getRoute(element::Flow flow);
		bool routeExist(element::Flow flow);
		void changeWeights();
		void updateEdgesValues(path * p);
		void updateAmountFlows(path * p, element::Flow flow);
		path * dijkstra(int vs, int vd);

		double getWeightLqi() const;
		void setWeightLqi(double weightLqi);
		double getWeightPath() const;
		void setWeightPath(double weightPath);
		void printExistingRoutes();

		static double THRESHOLD_QUALITY;
		static double TX_TOTAL_BALANCED; //GATO?
	private:
		std::map< std::string, path * > m_flowsPaths; //Map each flow to a path
		std::vector<element::Flow> m_orderflows; //Vector order insert flows
		double weightPath;
		double weightLqi;
		double weightBalance;
		typedef std::pair<int, int> Node; // <functionWeight, vertice>
		double calculateFunctionAmount(element::Edge edge);
		void printGraph ( );
	};
} // namespace algoritgm
#endif /* SRC_LAB_MODEL_ALGORITHM_RALL_H_ */
