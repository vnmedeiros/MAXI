/*
 * RALL.cc
 *
 *  Created on: Nov 6, 2016
 *      Author: vinicius
 */

#include "RALL.h"
NS_LOG_COMPONENT_DEFINE("ALGORITHM_RALL");

//double algorithm::RALL::THRESHOLD_QUALITY = 0.75; //PER
//double algorithm::RALL::THRESHOLD_QUALITY = 22.00;//13.00; //SINR
double algorithm::RALL::THRESHOLD_QUALITY = 0.75;//13.00; //SINR
double algorithm::RALL::TX_TOTAL_BALANCED = 1.0;


namespace algorithm {

	RALL::RALL(double weightPath, double weightLqi, element::Graph * graph) {
		if(weightLqi + weightPath > 1.0) {
			exit(1);
		}
		this->weightLqi = weightLqi;
		this->weightPath = weightPath;
		this->m_graph = graph;
		this->changeWeights();
	}

	RALL::~RALL() {

	}

	path * RALL::takeRoute(element::Flow flow) {
		int source = flow.m_sourceSink.first;
		//std::cout << "TakeRoute:" << flow.m_classApp << std::endl;
		if (source != 0) {
			NS_LOG_LOGIC("flow: " << flow.to_string());
			if(this->routeExist(flow)) {
				return this->m_flowsPaths[flow.to_string()];
			}
			m_orderflows.push_back(flow);
			return this->createRouteToFlow(flow);
		}
		NS_LOG_LOGIC("SINK to SINK?");
		return NULL;
	}

	//tree balanced short path and lqi hop weak
	/*path * RALL::createRouteToFlow(int soucer, int sink) {
		element::Flow flow(soucer, sink, -1, -1); //TODO: necessarioa dicionar as portas?
		path * p = this->dijkstra(soucer, sink);
		this->updateAmountFlows(p, flow);
		this->updateEdgesValues(p);
		//std::reverse(p->begin(),p->end());
		return p;
	}*/


	path * RALL::createRouteToFlow(element::Flow flow) {
		int source = flow.m_sourceSink.first;
		int sink = flow.m_sourceSink.second;

		LaboraAppHelper::Class classApp = static_cast<LaboraAppHelper::Class>(flow.m_classApp);
		//std::cout << "int:" << flow.m_classApp << " Cast:"  << classApp << " createRouteToFlow :  ";
		switch (classApp) {
            case LaboraAppHelper::Class::Class_1:
            case LaboraAppHelper::Class::Class_2:
				this->weightLqi = 0.46;
                this->weightPath = 0.37;
                this->weightBalance = 0.17;
                break;
            case LaboraAppHelper::Class::Class_3:
            case LaboraAppHelper::Class::Class_4:
            case LaboraAppHelper::Class::Class_5:
                this->weightLqi = 0.20;
                this->weightPath = 0.50;
                this->weightBalance = 0.30;
                break;
            default:
				std::cout << "\n[ATENÇÃO] - Default Ativo...\n";
                this->weightLqi = 0.5;
                this->weightPath = 0.5;
                this->weightBalance = 1.0;
                break;
        }

		//this->weightLqi = 1.0;
		//this->weightPath = 0.0;
		//this->weightBalance = 0.0;

		//this->printGraph();
		path * p = this->dijkstra(source, sink);
		//this->updateEdgesValues(p);
		this->updateAmountFlows(p, flow);

		this->m_flowsPaths[flow.to_string()] = p; // \FAZER: Adicionar contador para troca de rotas.

//		std::cout << "S= " << source << "(" << flow.m_sourcePort <<","<< flow.m_sinkPort << ")" << ": ";
//		for(unsigned int t = 0; t < p->size(); t++)
//			std::cout << (*p)[t] << " ";
//		std::cout << std::endl;
//		this->printGraph();

		return p;
	}

	void RALL::recreatesAllRoutes(void) {
		this->changeWeights();
		for (std::vector<element::Flow>::const_iterator it = this->m_orderflows.begin(); it != this->m_orderflows.end(); it++) {
			this->createRouteToFlow((*it));
		}
	}

	bool RALL::routeExist(element::Flow flow) {
		return this->m_flowsPaths.find(flow.to_string()) != this->m_flowsPaths.end();
	}

	path * RALL::getRoute(element::Flow flow) {
		if(this->routeExist(flow))
			return this->m_flowsPaths[flow.to_string()];
		return NULL;
	}

	void RALL::changeWeights() {
		int n = this->m_graph->listEdges.size();
		int p_const = n;
		//double factorPath = n * this->weightPath;
		//double factorLqi = n * this->weightLqi;
		for (int i = 0; i < n; i++) {
			//std::vector<element::Edge> * edgeNode = &this->m_graph->listEdges[i];
			for (unsigned int j = 0; j < this->m_graph->listEdges[i].size(); j++) {
				//TODO: quality THRESHOULD
				this->m_graph->listEdges[i][j].m_flows.clear();
				double lqiVal = 0;
				double pathVal = 1 * this->weightPath;
				if(this->m_graph->listEdges[i][j].quality <= RALL::THRESHOLD_QUALITY) {
					lqiVal = weightLqi - weightLqi * (this->m_graph->listEdges[i][j].quality / RALL::THRESHOLD_QUALITY);
					//lqiVal = weightLqi * (this->m_graph->listEdges[i][j].quality / RALL::THRESHOLD_QUALITY);
				}
				this->m_graph->listEdges[i][j].functionAmount = (lqiVal + pathVal) * p_const; // \FAZER acho que não via mais precisar dessa variavel.
				this->m_graph->listEdges[i][j].flowsFunctionAmount = 0.0;
			}
		}
	}

	/*
	 *
	 * TxChannels = soma de todas as taxas de transmissão do enlaces adjacentes que utilizam o mesmo canal.
	 * TX_TOTAL_BALANCED = total da taxa de transmissão gerada na rede, por todos os nós ativos.
	 *
	 * retorna TxChannels/TX_TOTAL_BALANCED. [0,1]
	 *
	 */
	double getValueBalanceFunction1(std::vector<element::Edge> * adj, CRAAChannel channel) {
		double TxChannels = 0.0;
		int nChannel = 0;
		for(unsigned int j = 0; j < adj->size(); j++) {
			if((*adj)[j].channel != channel)
				continue;
			nChannel++;
			for (unsigned int i = 0; i < (*adj)[j].m_flows.size(); i++) {
				LaboraAppHelper::Class classApp = static_cast<LaboraAppHelper::Class>((*adj)[j].m_flows[i].m_classApp);
				switch (classApp) { //converter a taxa para uma unica unidade Kb/p e somar...
					case LaboraAppHelper::Class::Class_1:
						TxChannels=TxChannels+264000.0;
						break;
					case LaboraAppHelper::Class::Class_2:
						TxChannels=TxChannels+1024000.0;
						break;
					case LaboraAppHelper::Class::Class_3:
						TxChannels=TxChannels+10.0;
						break;
					case LaboraAppHelper::Class::Class_4:
						TxChannels=TxChannels+32000.0;
						break;
					case LaboraAppHelper::Class::Class_5:
						TxChannels=TxChannels+3000.0;
						break;
					default:
						break;
				};
			}
		}
		TxChannels = TxChannels/(RALL::TX_TOTAL_BALANCED);
		return TxChannels;
	}

	/*
	 * TxChannels = soma de todas as taxas de transmissão do enlaces adjacentes que utilizam o mesmo canal.
	 * TX_LINK_BALANCED = capacidade de transmissão entre os enlaces
	 * nChannel = total de enlaces adjacentes utilizando o mesmo canal.
	 *
	 * retorna TxChannels/(TX_LINK_BALANCED/nChannel) [0,...]
	 *
	 */
	double getValueBalanceFunction2(std::vector<element::Edge> * adj, CRAAChannel channel) {
		double TX_LINK_BALANCED = 11 * 1000000;
		double TxChannels = 0.0;
		int nChannel = 0;
		for(unsigned int j = 0; j < adj->size(); j++) {
			if((*adj)[j].channel != channel)
				continue;
			nChannel++;
			for (unsigned int i = 0; i < (*adj)[j].m_flows.size(); i++) {
				LaboraAppHelper::Class classApp = static_cast<LaboraAppHelper::Class>((*adj)[j].m_flows[i].m_classApp);
				switch (classApp) { //converter a taxa para uma unica unidade Kb/p e somar...
					case LaboraAppHelper::Class::Class_1:
						TxChannels=TxChannels+320000.0;
						break;
					case LaboraAppHelper::Class::Class_2:
						TxChannels=TxChannels+512000.0;
						break;
					case LaboraAppHelper::Class::Class_3:
						TxChannels=TxChannels+1000000.0;
						break;
					default:
						break;
				};
			}
		}
		TxChannels = TxChannels/(TX_LINK_BALANCED/nChannel);
		return TxChannels;
	}

	/*
	 * Faz uma valor como referência (1Mbps) para contagem da quantidade de fluxo que utiliza determinado meio,
	 * os outros fluxo tem valores proporcional ao valo de referência.
	 * TxChannels = contagem do fluxos (proporcionalemnte a taxa de transmissão de cada fluxo).
	 *
	 */
	double getValueBalanceFunction3(std::vector<element::Edge> * adj, CRAAChannel channel) {
		double TxChannels = 0.0;
		int nChannel = 0;
		for(unsigned int j = 0; j < adj->size(); j++) {
			if((*adj)[j].channel != channel)
				continue;
			nChannel++;
			for (unsigned int i = 0; i < (*adj)[j].m_flows.size(); i++) {
				LaboraAppHelper::Class classApp = static_cast<LaboraAppHelper::Class>((*adj)[j].m_flows[i].m_classApp);
				switch (classApp) { //converter a taxa para uma unica unidade Kb/p e somar...
					case LaboraAppHelper::Class::Class_1:
						TxChannels=TxChannels+0.320; //TxChannels=TxChannels+320.0/1000.0;
						break;
					case LaboraAppHelper::Class::Class_2:
						TxChannels=TxChannels+0.512; //TxChannels+512.0/1000.0;
						break;
					case LaboraAppHelper::Class::Class_3:
						TxChannels=TxChannels+1.0;
						break;
					default:
						break;
				};
			}
		}
		return TxChannels;
	}

	/*
	 *
	 * TxChannels = soma de todas as taxas de transmissão multiplicadas pelo fator de overlay entre os canais do enlaces adjacentes.
	 * TX_TOTAL_BALANCED = total da taxa de transmissão gerada na rede, por todos os nós ativos.
	 *
	 * retorna TxChannels/TX_TOTAL_BALANCED. [0,1]
	 *
	 */
	double getValueBalanceFunction4(std::vector<element::Edge> * adj, CRAAChannel channel) {
		double matrixOverlayChannels[14][14] =
		   {{1.00, 0.77, 0.54, 0.31, 0.09, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
			{0.77, 1.00, 0.77, 0.54, 0.31, 0.09, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
			{0.54, 0.77, 1.00, 0.77, 0.54, 0.31, 0.09, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
			{0.31, 0.54, 0.77, 1.00, 0.77, 0.54, 0.31, 0.09, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00},
			{0.09, 0.31, 0.54, 0.77, 1.00, 0.77, 0.54, 0.31, 0.09, 0.00, 0.00, 0.00, 0.00, 0.00},
			{0.00, 0.09, 0.31, 0.54, 0.77, 1.00, 0.77, 0.54, 0.31, 0.09, 0.00, 0.00, 0.00, 0.00},
			{0.00, 0.00, 0.09, 0.31, 0.54, 0.77, 1.00, 0.77, 0.54, 0.31, 0.09, 0.00, 0.00, 0.00},
			{0.00, 0.00, 0.00, 0.09, 0.31, 0.54, 0.77, 1.00, 0.77, 0.54, 0.31, 0.09, 0.00, 0.00},
			{0.00, 0.00, 0.00, 0.00, 0.09, 0.31, 0.54, 0.77, 1.00, 0.77, 0.54, 0.31, 0.09, 0.00},
			{0.00, 0.00, 0.00, 0.00, 0.00, 0.09, 0.31, 0.54, 0.77, 1.00, 0.77, 0.54, 0.31, 0.09},
			{0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.09, 0.31, 0.54, 0.77, 1.00, 0.77, 0.54, 0.31},
			{0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.09, 0.31, 0.54, 0.77, 1.00, 0.77, 0.54},
			{0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.09, 0.31, 0.54, 0.77, 1.00, 0.77},
			{0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.09, 0.31, 0.54, 0.77, 1.00}};

		double TxChannels = 0.0;

		for(unsigned int j = 0; j < adj->size(); j++) {
			for (unsigned int i = 0; i < (*adj)[j].m_flows.size(); i++) {
				double overlay = matrixOverlayChannels[channel-1][(*adj)[j].channel-1];
				LaboraAppHelper::Class classApp = static_cast<LaboraAppHelper::Class>((*adj)[j].m_flows[i].m_classApp);
				switch (classApp) { //converter a taxa para uma unica unidade Kb/p e somar...
					case LaboraAppHelper::Class::Class_1:
						TxChannels=TxChannels+(320000.0*overlay);
						break;
					case LaboraAppHelper::Class::Class_2:
						TxChannels=TxChannels+(512000.0*overlay);
						break;
					case LaboraAppHelper::Class::Class_3:
						TxChannels=TxChannels+(1000000.0*overlay);
						break;
					default:
						break;
				};
			}
		}
		TxChannels = TxChannels/(RALL::TX_TOTAL_BALANCED);
		return TxChannels;
	}

	double getValueBalanceFunction5(std::vector<element::Edge> * adj, CRAAChannel channel) {
		double TxChannels = 0.0;
		for(unsigned int j = 0; j < adj->size(); j++) {
			for (unsigned int i = 0; i < (*adj)[j].m_flows.size(); i++) {
				LaboraAppHelper::Class classApp = static_cast<LaboraAppHelper::Class>((*adj)[j].m_flows[i].m_classApp);
				switch (classApp) {
					case LaboraAppHelper::Class::Class_1:
						TxChannels=TxChannels + 10;
						break;
					case LaboraAppHelper::Class::Class_2:
						TxChannels=TxChannels + 3 * 1000;
						break;
					case LaboraAppHelper::Class::Class_3:
						TxChannels=TxChannels + 32 * 1000;
						break;
					default:
						break;
				};
			}
		}
		TxChannels = TxChannels/(RALL::TX_TOTAL_BALANCED);
		return TxChannels;
	}

	double RALL::calculateFunctionAmount(element::Edge edge) {
		//int p_const = this->m_graph->listEdges.size();
		double pathVal = this->weightPath * 1;

		//double balanceVal = this->weightBalance * (edge.flowsFunctionAmount / p_const);
		//double balanceVal = this->weightBalance * edge.flowsFunctionAmount;
		std::vector<element::Edge> * adj = &this->m_graph->listEdges[edge.m_source];
		//std::vector<element::Edge> * adj = &this->m_graph->listEdges[edge.m_sink];
		//int af = 0;
		//for(unsigned int j = 0; j < adj->size(); j++) {
		//	af += (*adj)[j].m_flows.size();
		//}
		double af = getValueBalanceFunction1(adj, edge.channel);

		double lqiVal = 0;
		if(edge.quality < RALL::THRESHOLD_QUALITY) {
			double val = 1;
			double MIM = 0.60;
			if (edge.quality > MIM) {
				double delta = RALL::THRESHOLD_QUALITY - MIM;
				val = 1.0 - (edge.quality - MIM)/delta;
			}
			//lqiVal = weightLqi - (weightLqi * edge.quality) / RALL::THRESHOLD_QUALITY;
			lqiVal = (weightLqi * val);
		}
		//if(edge.m_sink == 0)
		//	std::cout <<"[" << edge.m_source << "-" << edge.m_sink << "]" << lqiVal <<"("<< edge.quality <<")" <<" + "<< pathVal <<" + "<< this->weightBalance  * af << std::endl;
		return (lqiVal + pathVal + this->weightBalance * af);// * p_const;
		//return (lqiVal + pathVal) * p_const + this->weightBalance * af;
	}


	/*
	 * method overriding
	 * return the short path considering custom weight
	 */
	path * RALL::ShortPath (int verticeSource, int verticeDestination) {
		return this->dijkstra(verticeSource, verticeDestination);
	}

	/*
	 * return [vd,v1,v2,...,vs]
	 */
	path * RALL::dijkstra(int vs, int vd) {
		//std::cout <<"RALL::dijkstra: " << vs << "->" << vd << std::endl;
		if (vs == vd) {
			path * r = new path(1, vs);
			return r;
		}
		int n = this->m_graph->listEdges.size();

		// Create a vector for distances and initialize all
		// distances as infinite (INF) and vector parents
		std::vector<double> dist(n, INF);
		std::vector<int> parent(n, -1);
		std::priority_queue<Node, std::vector<Node>, std::greater<Node> > pq;

		// Insert source itself in priority queue and initialize
		// its distance as 0.
		pq.push(std::make_pair(0, vs));
		dist[vs] = 0;
		parent[vs] = vs;
		/* Looping till priority queue becomes empty (or all distances are not finalized) */
		while (!pq.empty()) {
			// The first vertex in pair is the minimum distance
			// vertex, extract it from priority queue.
			// vertex label is stored in second of pair (it
			// has to be done this way to keep the vertices
			// sorted distance (distance must be first item
			// in pair)
			int u = pq.top().second;
			pq.pop();

			// 'i' is used to get all adjacent vertices of a vertex
			//list<pair<int, int> >::iterator i;
			std::vector<element::Edge> adj = this->m_graph->listEdges[u];
			for (unsigned int i = 0; i < adj.size(); i++) {
				// Get vertex label and weight of current adjacent
				// of u.
				int v = adj[i].m_sink;
				//int weight = adj[i].functionAmount;
				double weight = this->calculateFunctionAmount(adj[i]);

				//  If there is shorted path to v through u.
				if (dist[v] > dist[u] + weight) {
					// Updating distance of v
					dist[v] = dist[u] + weight;
					pq.push(std::make_pair(dist[v], v));
					parent[v] = u;

//					std::cout << "     ID:" << std::fixed << std::setprecision(2);
//					for (int p = 0; p < n; p++) {
//						std::cout << std::setfill (' ') << std::setw (6) << p;
//					}
//					std::cout << std::endl << " PARENT:";
//					for (int p = 0; p < n; p++) {
//						std::cout << std::setfill (' ') << std::setw (6) << parent[p];
//					}
//					std::cout << std::endl << "   DEST:";
//					for (int p = 0; p < n; p++) {
//						if (dist[p] == INF ) {
//							std::cout << "   INF";
//						} else  {
//							std::cout << std::setfill (' ') << std::setw (6) << dist[p];
//						}
//
//					}
//					std::cout << std::endl;
//					std::cout << std::endl;
				}
			}
		}

		path * r = new std::vector<int>();
		for (int i = vd; i != vs; i = parent[i]) {
			if (i == -1) {
				return NULL;
			}
			r->push_back(i);
		}
		r->push_back(vs);
		//std::cout <<"RALL::dijkstra: FIM!" << std::endl;
		return r;
	}

	void RALL::updateAmountFlows(path * p, element::Flow flow) {
		int sizeOfPath = p->size();
		for (int i = 0; i < sizeOfPath - 1; i++) {
			int vs = p->at(i + 1);
			int vd = p->at(i);
			element::Edge item(vs, vd, 1);
			std::vector<element::Edge> * adj = &this->m_graph->listEdges[vs];
			std::vector<element::Edge>::iterator ed = std::find(adj->begin(), adj->end(), item);
			ed->m_flows.push_back(flow);
			ed->flowsFunctionAmount++; // \FAZER como atribuir fluxos mais pesados...
		}
	}

	void RALL::updateEdgesValues(path * p) {
		int p_const = this->m_graph->listEdges.size();
		int sizeOfPath = p->size();
		for (int i = 0; i < sizeOfPath - 1; i++) {
			int vs = p->at(i + 1);
			int vd = p->at(i);
			int af = 0;
			std::vector<element::Edge> * adj = &this->m_graph->listEdges[vs];
			for(unsigned int j = 0; j < adj->size(); j++) {
				af += (*adj)[j].m_flows.size();
			}
			element::Edge item(vs, vd, 1);
			std::vector<element::Edge>::iterator ed = std::find(adj->begin(), adj->end(), item);
			double lqiVal = 0;
			double pathVal = 1 * this->weightPath;
			if(ed->quality < RALL::THRESHOLD_QUALITY) {
			    lqiVal = weightLqi - (weightLqi * ed->quality) / RALL::THRESHOLD_QUALITY;
				//lqiVal = weightLqi * (ed->quality / RALL::THRESHOLD_QUALITY);
			}
			ed->functionAmount = (lqiVal + pathVal) * p_const + af; // \FAZER acho que não vai mais precisar dessa variavel.
		}
	}

	double RALL::getWeightLqi() const {
		return weightLqi;
	}

	void RALL::setWeightLqi(double weightLqi) {
		this->weightLqi = weightLqi;
	}

	double RALL::getWeightPath() const {
		return weightPath;
	}

	void RALL::setWeightPath(double weightPath) {
		this->weightPath = weightPath;
	}

	void RALL::printExistingRoutes() {
		for(auto flow : this->m_flowsPaths) {
		   std::cout << flow.first << " [";
		   for (auto node : *flow.second ) {
			   std::cout << node << ",";
		   }
		   std::cout << "]\n";
		}
		std::cout << std::endl;
	}

	void RALL::printGraph ( ) {
		printf ("RALL::THRESHOLD_BALANCED:%f\n",RALL::TX_TOTAL_BALANCED);
		printf ("Graph (verticeI -> verticeJ[FunctionWeight]):\n");
		for (unsigned int i = 0; i < this->m_graph->listEdges.size (); i++) {
			printf ("vertice %d -> ", i);
			for (unsigned int j = 0; j < this->m_graph->listEdges[i].size(); j++) {

				double af=0;
				element::Edge ed = this->m_graph->listEdges[i][j];
				for (unsigned int i = 0; i < ed.m_flows.size(); i++) {
					LaboraAppHelper::Class classApp = static_cast<LaboraAppHelper::Class>(ed.m_flows[i].m_classApp);
					switch (classApp) {
						case LaboraAppHelper::Class::Class_1:
							af=af+320*1000;
							break;
						case LaboraAppHelper::Class::Class_2:
							af=af+512*1000;
							break;
						case LaboraAppHelper::Class::Class_3:
							af=af+1000000;
							break;
						default:
							break;
					};
				}

				double lqiVal = 0;
				if(ed.quality < RALL::THRESHOLD_QUALITY) {
					lqiVal = weightLqi - (weightLqi * ed.quality) / RALL::THRESHOLD_QUALITY;
				}
				printf ("%d[1(%.2f), %.2f(%.2f), %.2f(%.2f) = %.2f ch=%d] ", ed.m_sink, weightPath, ed.quality, lqiVal, af, af/RALL::TX_TOTAL_BALANCED, calculateFunctionAmount(ed),ed.channel);
			}
			printf ("\n");
		}
		printf ("\n");
	}


} /* namespace algorithm */


