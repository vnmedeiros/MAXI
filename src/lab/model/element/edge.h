/*
 * edge.h
 *
 *  Created on: Nov 2, 2016
 *      Author: vinicius
 */

#ifndef SRC_LAB_MODEL_ELEMENT_EDGE_H_
#define SRC_LAB_MODEL_ELEMENT_EDGE_H_

#include <iostream>
#include <vector>
#include <cstdio>
#include <queue>
#include <cmath>
#include "flow.h"

/*
 * Define Edge
 */
namespace element {

	class Edge {
	public:
		Edge ();
		Edge (int source, int sink, double weight);
		~Edge ();
		bool addQualitySample (double inElement);
		std::string to_string ();

		int m_source; 				//Source
		int m_sink;					//Sink = Destination
		double m_weight;			//Weight
		std::vector<Flow> m_flows;	//Flows traversing the edge
		double m_utilization;		//Utilization to CRAA
		double quality;
		double quality_MMS;
		double quality_MME;
		double functionAmount;
		double flowsFunctionAmount;
		int channel;
	private:
		unsigned int SIZE_QUEUE = 50;
		//double THRESHOLD_QUALITY = 0.10; //PER
		double THRESHOLD_QUALITY = 3.0; //SINR
		double lastQuality;
		std::queue<double> qualitySampleQueue;

	};

	inline bool operator==(const element::Edge e1, const element::Edge e2) {
		if (e1.m_source == e2.m_source && e1.m_sink == e2.m_sink) {
			return true;
		} else {
			return false;
		}
	}

	inline bool operator<(const element::Edge e1, const element::Edge e2) {
		if (e1.m_source < e2.m_source) {
			return true;
		} else if (e1.m_source == e2.m_source) {
			if (e1.m_sink < e2.m_sink) {
				return true;
			} else {
				return false;
			}
		} else {
			return false;
		}
	}

	inline bool operator>(const element::Edge e1, const element::Edge e2) {
		if (e1.m_source > e2.m_source) {
			return true;
		} else if (e1.m_source == e2.m_source) {
			if (e1.m_sink > e2.m_sink) {
				return true;
			} else {
				return false;
			}
		} else {
			return false;
		}
	}

}
#endif /* SRC_LAB_MODEL_ELEMENT_EDGE_H_ */
