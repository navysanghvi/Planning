#include "RrtConnect.hpp"
using namespace std;

RrtConnect::RrtConnect(double* stt, double* end, int n, 
	     int x, int y, double* m, double extR, double extS, 
	     int numS, double goalB): Rrt(stt, end, n, 
	     x, y, m, extR, extS, numS, goalB) {}

RrtConnect::RrtConnect(): Rrt() {}



void RrtConnect::SwapGraphs(vector<GraphVertex*>* g1, vector<GraphVertex*>* g2)
{
	vector<GraphVertex*> temp;
	temp = *g1;
	*g1 = *g2;
	*g2 = temp;
}

void RrtConnect::SwapGoals(vector<double>* goal1, vector<double>* goal2)
{
	vector<double> temp;
	temp = *goal1;
	*goal1 = *goal2;
	*goal2 = temp;
}

pair<int, double**> RrtConnect::GetFinalPlan(vector<vector<double>> backPath1, vector<vector<double>> backPath2)
{
	int planLength1 = backPath1.size(), planLength2 = backPath2.size();
	int planLength = planLength1+planLength2;
	double** finalPlan = (double**) malloc(planLength*sizeof(double*));
	for(int i = 0; i < planLength1; i++)
	{
		finalPlan[i] = (double*) malloc(m_numDOFs*sizeof(double));
		for(int j = 0; j < m_numDOFs; j++)
			finalPlan[i][j] = backPath1[planLength1-i-1][j];
	}
	for(int i = planLength1; i < planLength; i++)
	{
		finalPlan[i] = (double*) malloc(m_numDOFs*sizeof(double));
		for(int j = 0; j < m_numDOFs; j++)
			finalPlan[i][j] = backPath2[i-planLength1][j];
	}

	return pair<int, double**>(planLength, finalPlan);
}

pair<int,double**> RrtConnect::RunPlan()
{
	vector<vector<double>> backPath1, backPath2;
	vector<double> qrand, qnew, qValid1, qValid2; 
	double sDist, toDist;
	bool reached = 0, reverse = 0;
	int gInd;
	int lastSample;
	double cost1, cost2;

	clock_t begin_time = clock();

	vector<double> goal1 = m_goal, goal2 = m_start;
	InitGraph(&m_rrtCGraph1, m_start);
	InitGraph(&m_rrtCGraph2, m_goal);
	for(int i = 0; i < m_nSamples; i++)
	{
		qrand = GetSample(goal1);
		
		tie(gInd, sDist) = NearestNeighbor(m_rrtCGraph1, qrand);
		tie(qnew, toDist) = ExtendMax(m_rrtCGraph1, gInd, sDist, qrand);
		qValid1 = ExtendTo(m_rrtCGraph1, gInd, toDist, qnew);

		if(qValid1.empty()){ i--; continue; }
		AddChildToGraph(&m_rrtCGraph1, gInd, qValid1); 

		tie(gInd, sDist) = NearestNeighbor(m_rrtCGraph2, qValid1);
		qValid2 = ExtendTo(m_rrtCGraph2, gInd, sqrt(sDist), qValid1);
		if(!qValid2.empty() && sqrt(SquareDistance(qValid1, qValid2)) <= 0.1)
		{
			reached = 1;
			AddChildToGraph(&m_rrtCGraph2, gInd, qValid1);
			break;
		}
		else 
		{ 
			SwapGraphs(&m_rrtCGraph1, &m_rrtCGraph2); 
			SwapGoals(&goal1, &goal2); 
			reverse = !reverse; 
		}
	}

	if(reached)
	{
		tie(cost1,backPath1) = BacktrackFromSampleGetCost(m_rrtCGraph1, m_rrtCGraph1.size()-1, 1);
		tie(cost2,backPath2) = BacktrackFromSampleGetCost(m_rrtCGraph2, m_rrtCGraph2.size()-1, 1);
		//cout << "Reached after " << (m_rrtCGraph1.size()-1) << " samples\n";
		//cout << "Path Cost = " << (cost1+cost2) << '\n';
	}

	m_planTime = double(clock() - begin_time)/CLOCKS_PER_SEC;
	m_pathQuality = (cost1+cost2);
	m_samplesUsed = (m_rrtCGraph1.size()-1) + (m_rrtCGraph2.size()-1);
	m_samplesAdded = m_samplesUsed;

	if(!reverse){ return GetFinalPlan(backPath1, backPath2); }
	else{ return GetFinalPlan(backPath2, backPath1); }
}
