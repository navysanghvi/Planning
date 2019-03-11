#ifndef RRTCONNECT_HPP
#define RRTCONNECT_HPP

#include "Rrt.hpp"
#include <time.h>
using namespace std;

class RrtConnect: public Rrt
{
public: 
	RrtConnect(double* stt, double* end, int n, 
	     int x, int y, double* m, 
	     double extR, double extS, 
	     int numS, double goalB);
	RrtConnect();
	~RrtConnect();

	void SwapGraphs(vector<GraphVertex*>* g1, vector<GraphVertex*>* g2);
	void SwapGoals(vector<double>* goal1, vector<double>* goal2);
	pair<int, double**> GetFinalPlan(vector<vector<double>> backPath1, vector<vector<double>> backPath2);
	pair<int,double**> RunPlan();

	double m_planTime;
	int m_samplesUsed;
	int m_samplesAdded;
	double m_pathQuality;

private:
    vector<GraphVertex*> m_rrtCGraph1;
    vector<GraphVertex*> m_rrtCGraph2;
};

#endif