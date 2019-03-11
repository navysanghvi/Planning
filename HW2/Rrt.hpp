#ifndef RRT_HPP
#define RRT_HPP

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/incremental_components.hpp>
#include <cmath>
#include <math.h>
#include <time.h>
#include <random>
#include <iostream>
#include "PlanSample.hpp"
#include "GraphVertex.hpp"
#define D_INF numeric_limits<double>::infinity()
using namespace std;

class Rrt: public PlanSample
{
public:
	Rrt(double* stt, double* end, 
		int n, int x, int y, double* m, 
		double extR, double extS, 
		int numS, double goalB);
	Rrt();
	~Rrt();

	vector<double> GetSample(vector<double> goal);
	void InitGraph(vector<GraphVertex*>* graph, vector<double> start_angs);
	void AddChildToGraph(vector<GraphVertex*>* graph, int parentIndex, vector<double> child);
	pair<vector<double>,double> ExtendMax(vector<GraphVertex*> graph, int gIndex, double sDistance, vector<double> qrand);
	vector<double> ExtendTo(vector<GraphVertex*> graph, int gIndex, double toDistance, vector<double> qnew);
	bool IsCloseToGoal(vector<double> qValid, vector<double> goal);
	pair<double,vector<vector<double>>> BacktrackFromSampleGetCost(vector<GraphVertex*> graph, int sample, bool getPath);
	pair<int, double**> GetFinalPlan(vector<vector<double>> backPath, bool reverse);
	bool FindPlan(vector<double> start, vector<double> goal);
	pair<int, double**> RunPlan();

	double m_planTime;
	int m_samplesUsed;
	int m_samplesAdded;
	double m_pathQuality;

protected:
	vector<GraphVertex*> m_rrtGraph;
	double m_extendRadius;
	double m_extendStep;
	int m_nSamples;
	double m_gBias;
	double m_qDistEps;
	double m_gPerturb;

};

#endif