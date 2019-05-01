/*
 * CRAA.h
 *
 *  Created on: Nov 4, 2016
 *      Author: vinicius
 */

#ifndef SRC_LAB_MODEL_ALGORITHM_CRAA_H_
#define SRC_LAB_MODEL_ALGORITHM_CRAA_H_

#include <map>
#include <set>
#include <vector>
#include "ns3/object.h"
#include "ns3/core-module.h"
#include "ns3/log.h"

#include "ns3/lab.h"
#include "ns3/edge.h"
#include "ns3/flow.h"
#include "ns3/LBR.h"
#include "ns3/RALL.h"

namespace algorithm {

class CRAA {
public:
	/*CRAA (LBR *lbr, int allowedReadjustments, double interferenceThreshold,
			std::set<CRAAChannel> channels, int nRadios, int nFlows,
			std::map< element::Edge, std::map<element::Edge, double> > per);

	CRAA (RALL *rall, int allowedReadjustments, double interferenceThreshold,
				std::set<CRAAChannel> channels, int nRadios, int nFlows,
				std::map< element::Edge, std::map<element::Edge, double> > per);*/
	CRAA (LABORARouting * routing, int allowedReadjustments, double interferenceThreshold,
					std::set<CRAAChannel> channels, int nRadios, int nFlows,
					std::map< element::Edge, std::map<element::Edge, double> > per, int channelDefault);
	~CRAA ();
	void SetChannelsToRadios (int *channelsToRadios);

	double EdgeInterference (element::Edge u, element::Edge w);
	double EdgeInterferenceInitial (element::Edge u, element::Edge w);
	bool ICV (std::vector<element::Edge> E, int node, CRAAChannel currentChannel, CRAAChannel candidateChannel);
	std::vector<element::Edge> SortEdges (std::vector<element::Edge> E);

	void InitialUtilization ();
	void RunInitial ();

	void Utilization ();
	void Run ();

	//RALL * m_rall;
	//LBR * m_lbr;
	LABORARouting * m_routing;
	int m_allowedReadjustments;
	double m_interferenceThreshold;
	std::set<CRAAChannel> m_channels;
	int m_nRadios;
	std::map< element::Edge, std::map<element::Edge, double> > m_per;

	std::map<element::Edge, CRAAChannel> m_allocatedChannels;
	std::map< int, std::set<CRAAChannel> > m_channelsToNodes;
	int *m_channelsToRadios;

	int m_nFlows;
	int m_nFinishedFlows;

	int m_nChannelChanges;

};
}
#endif /* SRC_LAB_MODEL_ALGORITHM_CRAA_H_ */
