#ifndef RRTSTAR_HPP
#define RRTSTAR_HPP

#include "Rrt.hpp"
using namespace std;

class RrtStar: public Rrt
{
public: 
	RrtStar(double* stt, double* end, int n, 
	     int x, int y, double* m, double extR, double extS, 
	     int numS, double goalB, double neighR);
    RrtStar();
    ~RrtStar();

    vector<pair<int,double>> NeighborIndices(vector<GraphVertex*> graph, vector<double> qValid);
    vector<double> NeighborCosts(vector<GraphVertex*> graph, vector<pair<int,double>> neighs);
    vector<bool> HasPathToMe(vector<GraphVertex*> graph, int gInd, vector<pair<int,double>> neighs, 
    	                     vector<double> qValid);
    pair<int,double> MyBestParent(vector<GraphVertex*> graph, 
		vector<pair<int,double>> neighs, vector<double> neighCosts, 
		vector<double> qValid, vector<bool> hasPath);
    void AmBetterParent(vector<GraphVertex*> graph, 
		vector<pair<int,double>> neighs, vector<double> neighCosts,
		int myInd, int parInd, double parCost, vector<bool> hasPath);
    pair<bool,int> FindPlan(vector<double> start, vector<double> goal);
    pair<int,double**> RunPlan();

    double m_planTime;
    int m_samplesUsed;
    int m_samplesAdded;
    double m_pathQuality;

private:
    vector<GraphVertex*> m_rrtSGraph;
    double m_gNeighRadius;
};

#endif