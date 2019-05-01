/*
 * edge.cc
 *
 *  Created on: Nov 2, 2016
 *      Author: vinicius
 */

#include "edge.h"

namespace element {

	Edge::Edge() {
		m_utilization = 0;
		m_source = -1;
		m_sink = -1;
		m_weight = -1;
		quality = 0.0;
		quality_MMS = 0.0;
		quality_MME = 0.0;
		functionAmount = 0.0;
		lastQuality  = 0.0;
	}

	Edge::Edge(int source, int sink, double weight) {
		m_source = source;
		m_sink = sink;
		m_weight = weight;
		m_utilization = 0;
		quality = 0.0;
		quality_MMS = 0.0;
		quality_MME = 0.0;
		functionAmount = 0.0;
		lastQuality  = 0.0;
	}

	Edge::~Edge() {
	}


	//MMS = Media Movel Simples
	//MME = Media Movel Exponencial
	bool Edge::addQualitySample (double inElement) {
		double outElement = 0.0;
		double amount = 0.0;
		double alfa = 0.2;
		unsigned int size = SIZE_QUEUE;
		this->qualitySampleQueue.push(inElement);
		if (this->qualitySampleQueue.size() == SIZE_QUEUE+1) {
			outElement = this->qualitySampleQueue.front();
			this->qualitySampleQueue.pop();
			amount = (this->quality * size - outElement + inElement);
			this->quality_MMS = amount/size;  //MMS
			this->quality_MME = (inElement - this->quality_MMS) * alfa + this->quality_MMS; //MME
		} else {
			size = this->qualitySampleQueue.size();
			amount = (this->quality * (size-1) - outElement + inElement);
			this->quality_MMS = amount/size;  //MMS
			this->quality_MME = this->quality_MMS; //MME
		}

		//this->quality = this->quality_MMS;
		this->quality = this->quality_MME;
		double diff = this->quality - this->lastQuality;
		if(std::abs(diff) >= this->THRESHOLD_QUALITY || this->lastQuality == 0) {
			this->lastQuality = this->quality;
			return true;
		}
		return false;
	}

	std::string Edge::to_string() {
		std::string edge;
		char temp[10];

		edge = "";
		sprintf(temp, "%d", m_source);
		edge += temp;
		edge += "-";
		sprintf(temp, "%d", m_sink);
		edge += temp;

		return edge;
	}

} /* namespace algorithm */
