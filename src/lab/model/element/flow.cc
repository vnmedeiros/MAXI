/*
 * flow.cc
 *
 *  Created on: Nov 2, 2016
 *      Author: vinicius
 */

#include "flow.h"

namespace element {
	Flow::Flow () {
		m_sourceSink.first = -1;
		m_sourceSink.second = -1;
		m_sourcePort = -1;
		m_sinkPort = -1;
		m_classApp = -1;
	}

	Flow::Flow (int source, int sink, int sourcePort, int sinkPort) {
		m_sourceSink.first = source;
		m_sourceSink.second = sink;
		m_sourcePort = sourcePort;
		m_sinkPort = sinkPort;
		m_classApp = -1;
	}

	Flow::Flow (int source, int sink, int sourcePort, int sinkPort, int classApp) {
		m_sourceSink.first = source;
		m_sourceSink.second = sink;
		m_sourcePort = sourcePort;
		m_sinkPort = sinkPort;
		m_classApp = classApp;
	}

	Flow::~Flow () {
	}

	std::string Flow::to_string () {
		std::string flow;
		char temp[10];

		flow = "";
		sprintf (temp, "%d", m_sourceSink.first); flow += temp;
		flow += ":";
		sprintf (temp, "%d", m_sourcePort); flow += temp;
		flow += "--";
		sprintf (temp, "%d", m_sourceSink.second); flow += temp;
		flow += ":";
		sprintf (temp, "%d", m_sinkPort); flow += temp;
		flow += "-";
		sprintf (temp, "%d", m_classApp); flow += temp;

		return flow;
	}

} /* namespace element */
