#include "RrtStar.hpp"
using namespace std;

RrtStar::RrtStar(double* stt, double* end, int n, 
	     int x, int y, double* m, double extR, double extS, 
	     int numS, double goalB, double neighR): Rrt(stt, end, n, 
	     x, y, m, extR, extS, numS, goalB), m_gNeighRadius(neighR) {}

RrtStar::RrtStar(): Rrt() {}


vector<pair<int,double>> RrtStar::NeighborIndices(vector<GraphVertex*> graph, vector<double> qValid)
{
	vector<pair<int,double>> neighbors;
	double dist;
	for(int i = 0; i < graph.size(); i++)
	{
		dist = sqrt(SquareDistance(graph[i]->qAngs, qValid));
		if(dist <= m_gNeighRadius)
			neighbors.push_back(pair<int,double>(i, dist));
	}
	return neighbors;
}


vector<bool> RrtStar::HasPathToMe(vector<GraphVertex*> graph, int gInd, 
	vector<pair<int,double>> neighs, vector<double> qValid)
{
	vector<double> qCheck;
	vector<bool> hasPath(neighs.size());
	for(int i = 0; i < neighs.size(); i++)
	{
		if(gInd == neighs[i].first){ hasPath[i] = 1; continue; }
		qCheck = ExtendTo(graph, neighs[i].first, neighs[i].second, qValid);
		hasPath[i] = (!qCheck.empty() && sqrt(SquareDistance(qValid, qCheck)) <= 0.1) ? 1 : 0;
	}
	return hasPath;
}

vector<double> RrtStar::NeighborCosts(vector<GraphVertex*> graph, 
	                             vector<pair<int,double>> neighs)
{
	vector<double> neighCosts(neighs.size());
	vector<vector<double>> bp;
	for(int i = 0; i < neighs.size(); i++)
		tie(neighCosts[i],bp) = BacktrackFromSampleGetCost(graph, neighs[i].first, 0);
	return neighCosts;
}

pair<int,double> RrtStar::MyBestParent(vector<GraphVertex*> graph, 
	vector<pair<int,double>> neighs, vector<double> neighCosts, 
	vector<double> qValid, vector<bool> hasPath)
{
	int minInd;
	double minCost = D_INF, pCost;
	
	for(int i = 0; i < neighs.size(); i++)
	{
		pCost = neighCosts[i] + neighs[i].second;
		if(pCost < minCost && hasPath[i])
		{
			minCost = pCost;
			minInd = neighs[i].first;
		}
	}
	return (pair<int,double>(minInd, minCost));
}

void RrtStar::AmBetterParent(vector<GraphVertex*> graph, 
	vector<pair<int,double>> neighs, vector<double> neighCosts,
	int myInd, int parInd, double parCost, vector<bool> hasPath)
{
	double pCost;
	for(int i = 0; i < neighs.size(); i++)
	{
		if(neighs[i].first == parInd){ continue; }
		pCost = parCost + neighs[i].second;
		if(pCost < neighCosts[i] && hasPath[i])
		{
			graph[neighs[i].first]->parCost = neighs[i].second;
			graph[neighs[i].first]->parent = myInd;
		}
	}
}

pair<bool,int> RrtStar::FindPlan(vector<double> start, vector<double> goal)
{
	vector<double> qrand, qnew, qValid;
	vector<pair<int,double>> neighs;
	vector<double> neighCosts;
	vector<bool> hasPath;
	double sDist, toDist;
	bool reached = 0, reached_before = 0;
	int gInd;
	int parInd;
	double parCost;
	int lastSample = -1;

	clock_t begin_time = clock();

	InitGraph(&m_rrtSGraph, start);
	for(int i = 0; i < m_nSamples; i++)
	{
		qrand = GetSample(goal);
		
		tie(gInd, sDist) = NearestNeighbor(m_rrtSGraph, qrand);
		tie(qnew, toDist) = ExtendMax(m_rrtSGraph, gInd, sDist, qrand);
		qValid = ExtendTo(m_rrtSGraph, gInd, toDist, qnew);

		if(qValid.empty()){ i--; continue; }

		if(!reached){ reached = IsCloseToGoal(qValid, goal); }
		if(reached && !reached_before) 
		{ 
			neighs = NeighborIndices(m_rrtSGraph, goal);
			neighCosts = NeighborCosts(m_rrtSGraph, neighs);
			hasPath = HasPathToMe(m_rrtSGraph, gInd, neighs, goal);
			tie(parInd, parCost) = MyBestParent(m_rrtSGraph, neighs, neighCosts, goal, hasPath);
			AddChildToGraph(&m_rrtSGraph, parInd, goal);
			lastSample = m_rrtSGraph.size()-1;
			reached_before = 1;
			//break; 
		}
		else 
		{ 
			neighs = NeighborIndices(m_rrtSGraph, qValid);
			neighCosts = NeighborCosts(m_rrtSGraph, neighs);
			hasPath = HasPathToMe(m_rrtSGraph, gInd, neighs, qValid);
			tie(parInd, parCost) = MyBestParent(m_rrtSGraph, neighs, neighCosts, qValid, hasPath);
			AddChildToGraph(&m_rrtSGraph, parInd, qValid);
			AmBetterParent(m_rrtSGraph, neighs, neighCosts, m_rrtSGraph.size()-1, parInd, parCost, hasPath);
		}
	}
	return pair<bool,int>(reached,lastSample);
}


pair<int,double**> RrtStar::RunPlan()
{
	vector<vector<double>> backPath;
	bool reached, reverse = 0;
	int lastSample;
	double cost;

	clock_t begin_time = clock();

	tie(reached, lastSample) = FindPlan(m_start, m_goal);

	if(reached)
	{
		tie(cost,backPath) = BacktrackFromSampleGetCost(m_rrtSGraph, lastSample, 1);
		//cout << "Reached after " << lastSample << " samples in forward search\n";
		//cout << "Path Cost = " << cost << '\n';
		m_samplesAdded = m_nSamples;
		m_samplesUsed = lastSample;
	}
	else
	{
		lastSample = NearestNeighbor(m_rrtSGraph, m_goal).first;
		tie(cost,backPath) = BacktrackFromSampleGetCost(m_rrtSGraph, lastSample, 1);
		m_samplesAdded = m_nSamples;
		m_samplesUsed = m_nSamples;

		//cout << "Forward path not found, trying backward\n";
		tie(reached,lastSample) = FindPlan(m_goal, m_start);

		if(reached)
		{
			tie(cost, backPath) = BacktrackFromSampleGetCost(m_rrtSGraph, lastSample, 1);

			//cout << "Reached after " << lastSample << " samples in backward search\n";
			//cout << "Path Cost = " << cost << '\n';
			reverse = 1;
			m_samplesAdded += m_nSamples;
			m_samplesUsed += lastSample;
		}
		else { cost = D_INF;/*cout << "Not reached via both forward and backward searches\n";*/ }
	}
	m_planTime = double(clock() - begin_time)/CLOCKS_PER_SEC;
	m_pathQuality = cost;

	return GetFinalPlan(backPath, reverse);

}