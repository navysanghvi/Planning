#ifndef PRMAP_HPP
#define PRMAP_HPP

#include "PlanSample.hpp"
#include "Compare.cpp"
#include <time.h>
#include <queue>
using namespace std;

class PRMap: public PlanSample
{
public: 
	PRMap(double* stt, double* end, int n, int x, int y, 
		double* m, double extS, int numS, double neighR, int k);
	PRMap();
	~PRMap();

	vector<double> GetSample();
	vector<pair<int,double>> NeighborIndices(GraphU_t graph, int maxInd, vector<double> qValid);
	vector<bool> HasPathToMe(GraphU_t graph, vector<pair<int,double>> neighs, vector<double> qValid);
	vector<double> ExtendTo(GraphU_t graph, int neighInd, double neighDist, vector<double> qValid);
	void ConnectVertex(GraphU_t* graph, int maxInd, vector<double> qValid, 
		vector<pair<int,double>> neighs, vector<bool> hasPath);
		//,boost::disjoint_sets<GURank_t*,GUParent_t*>* ds);
	void ConstructGraph();
	void Dijkstra();
	vector<vector<double>> BacktrackFromSample(GraphU_t graph, int sample);
	pair<int, double**> GetFinalPlan(vector<vector<double>> backPath, bool reverse);
	pair<int, double**> RunPlan();

	double m_planTime;
	int m_samplesUsed;
	int m_samplesAdded;
	double m_pathQuality;

private:
	GraphU_t m_prmGraph;
	int m_degreeLimit;
	int m_nSamples;
	double m_gNeighRadius;
	double m_extendStep;
};

#endif