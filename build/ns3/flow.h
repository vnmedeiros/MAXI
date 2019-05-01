/*
 * flow.h
 *
 *  Created on: Nov 2, 2016
 *      Author: vinicius
 */

#ifndef SRC_LAB_MODEL_ELEMENT_FLOW_H_
#define SRC_LAB_MODEL_ELEMENT_FLOW_H_

#include <map>
#include <cstdlib>
#include <cstdio>

typedef std::pair<int, int> FlowSourceSink;

/*
 * Define Flow
 */
namespace element {
	class Flow {
	public:
		Flow ();
		Flow (int source, int sink, int sourcePort, int sinkPort);
		Flow (int source, int sink, int sourcePort, int sinkPort, int classApp);
		~Flow ();

		std::string to_string ();

		FlowSourceSink m_sourceSink; //Source-sink pair
		int m_sourcePort;			 //Source port
		int m_sinkPort;				 //Sink port
		int m_classApp;
	};

	inline bool operator== (const element::Flow f1, const element::Flow f2) {
		if (f1.m_sourceSink == f2.m_sourceSink &&
			f1.m_sourcePort == f2.m_sourcePort &&
			f1.m_sinkPort == f2.m_sinkPort &&
			f1.m_classApp == f2.m_classApp) {
			return true;
		} else {
			return false;
		}
	}
}

#endif /* SRC_LAB_MODEL_ELEMENT_FLOW_H_ */
