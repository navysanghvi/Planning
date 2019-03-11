#include "Rrt.hpp"
using namespace std;

/* Constructor */
Rrt::Rrt(double* stt, double* end, int n, 
	     int x, int y, double* m, 
	     double extR, double extS, 
	     int numS, double goalB): PlanSample(stt, end, n, x, y, m) 
{ 
	m_extendRadius = extR; 
	m_extendStep = extS;
	m_nSamples = numS;
	m_gBias = goalB;
	m_qDistEps = 0.2;
	m_gPerturb = 2*1e-1/sqrt(m_numDOFs);
	for(int i = 0; i < m_numDOFs; i++)
	{
		m_start[i] = AngleInRange(m_start[i]);
		m_goal[i] = AngleInRange(m_goal[i]);
	}
	
}

/* Default Constructor */
Rrt::Rrt(): PlanSample() 
{ 
	m_extendRadius = 0;
	m_extendStep = 0;
	m_nSamples = 0;
	m_gBias = 0;
	m_qDistEps = 1e-1;
}

/* Destructor */
Rrt::~Rrt(){}

void Rrt::InitGraph(vector<GraphVertex*>* graph, vector<double> start_angs)
{
	(*graph) = vector<GraphVertex*>();
	(*graph).push_back(new GraphVertex(start_angs, 0.0));
}

/* Add child to the graph from parent */
void Rrt::AddChildToGraph(vector<GraphVertex*>* graph, int parentIndex, vector<double> child)
{
	double pCost = sqrt(SquareDistance((*graph)[parentIndex]->qAngs, child));
	(*graph).push_back(new GraphVertex(child, parentIndex, pCost));
	//graph[boost::add_vertex(graph)] = new GraphVertex(child, parentIndex);
	//pair<GDEdge,bool> p = boost::add_edge(parentIndex, boost::num_vertices(graph)-1, graph);
}

/* Furthest possible point and distance to extend to from tree toward qrand*/
pair<vector<double>,double> Rrt::ExtendMax(vector<GraphVertex*> graph, int gIndex, double sDistance, 
	                                       vector<double> qrand)
{
	sDistance = sqrt(sDistance);
	if(sDistance > m_extendRadius)
	{
		vector<double> qtree = graph[gIndex]->qAngs;
		double fracExtend = m_extendRadius/sDistance;
		for(int i = 0 ; i < m_numDOFs; i++)
		{	qrand[i] = (qtree[i] + (fracExtend*WrapDifference(qtree[i], qrand[i]))); }
	}
	return(pair<vector<double>,double>(qrand, min(sDistance, m_extendRadius)));
}

/* Furthest valid point to extend to from tree toward qnew*/
vector<double> Rrt::ExtendTo(vector<GraphVertex*> graph, int gIndex, double toDistance, vector<double> qnew)
{
	vector<double> qPrev = graph[gIndex]->qAngs;
	vector<double> qValid, qInRange = qPrev;
	vector<double> qDiff(m_numDOFs);
	for(int i = 0; i < m_numDOFs; i++)
		qDiff[i] = WrapDifference(qPrev[i],qnew[i]);//qnew[i] - qPrev[i];

	int step = 0; 
	int numSteps = ceil(toDistance/m_extendStep);
	double fracExtend = 1/((double)(numSteps));
	
	while(step++ < numSteps)
	{
		for(int i = 0; i < m_numDOFs; i++)
		{
			qPrev[i] += fracExtend*qDiff[i];
			qInRange[i] = AngleInRange(qPrev[i]);
		}
		if(IsValidArmConfiguration(qInRange)) { qValid = qInRange; }
		else { break; }
	}
	return qValid;
}

/* Check valid sample's closeness to goal */
bool Rrt::IsCloseToGoal(vector<double> qValid, vector<double> goal)
{
	if(sqrt(SquareDistance(qValid, goal)) <= m_qDistEps) { return 1; }
	else { return 0; }
}

/* Return backtracked path from sample to start, and path's cost */
pair<double,vector<vector<double>>> Rrt::BacktrackFromSampleGetCost(vector<GraphVertex*> graph, int sample, bool getPath)
{
	vector<vector<double>> backPath;
	int currI = sample;
	double cost = 0;

	if(getPath)
	{
		backPath.push_back(graph[currI]->qAngs);
		do
		{   
			currI = graph[currI]->parent;
			backPath.push_back(graph[currI]->qAngs);
		} while( currI != 0 );

	}

	currI = sample;
	while(currI != 0)
	{
		cost += graph[currI]->parCost;  
		currI = graph[currI]->parent;
	}

	return pair<double,vector<vector<double>>>(cost, backPath);
}

/* Get random sample incorporating goal bias */
vector<double> Rrt::GetSample(vector<double> goal)
{
	vector<double> qrand(m_numDOFs);
	if(GetRandomSample(0,1) > m_gBias)
		for(int i = 0; i < m_numDOFs; i++)
			qrand[i] = GetRandomSample(-M_PI, M_PI);
	else 
	{
		if(GetRandomSample(0,1) > 0.5)
			for(int i = 0; i < m_numDOFs; i++)
				qrand[i] = AngleInRange(goal[i] + GetRandomSample(-m_gPerturb, m_gPerturb));
		else { qrand = goal; }
	}
	return qrand;
}


/* Return final plan (possibly reversed) and plan length */
pair<int, double**> Rrt::GetFinalPlan(vector<vector<double>> backPath, bool reverse)
{
	int planLength = backPath.size();
	double** finalPlan = (double**) malloc(planLength*sizeof(double*));
	for(int i = 0; i < planLength; i++)
	{
		finalPlan[i] = (double*) malloc(m_numDOFs*sizeof(double));
		for(int j = 0; j < m_numDOFs; j++)
			finalPlan[i][j] = reverse ? backPath[i][j] : backPath[planLength-i-1][j];
	}

	return pair<int, double**>(planLength, finalPlan);
}

/* Attempt to find plan within m_nSamples number of samples */
bool Rrt::FindPlan(vector<double> start, vector<double> goal)
{
	vector<double> qrand, qnew, qValid; 
	double sDist, toDist;
	bool reached = 0;
	int gInd;

	InitGraph(&m_rrtGraph, start);
	for(int i = 0; i < m_nSamples; i++)
	{
		qrand = GetSample(goal);
		
		tie(gInd, sDist) = NearestNeighbor(m_rrtGraph, qrand);
		tie(qnew, toDist) = ExtendMax(m_rrtGraph, gInd, sDist, qrand);
		qValid = ExtendTo(m_rrtGraph, gInd, toDist, qnew);

		if(qValid.empty()){ i--; continue; }

		reached = IsCloseToGoal(qValid, goal);
		if(reached) { AddChildToGraph(&m_rrtGraph, gInd, goal); break; }
		else { AddChildToGraph(&m_rrtGraph, gInd, qValid); }
	}
	return reached;

}

/* Run the planner - 
(1) try to reach goal from start
(2) if fail, try to reach start from goal 
*/
pair<int,double**> Rrt::RunPlan()
{
	vector<vector<double>> backPath;
	bool reached, reverse = 0;
	int lastSample;
	double cost; 

	clock_t begin_time = clock();
	
	// Attempt to reach goal from start
	reached = FindPlan(m_start, m_goal);

	if(reached)
	{
		lastSample = m_rrtGraph.size()-1;
		tie(cost,backPath) = BacktrackFromSampleGetCost(m_rrtGraph, lastSample, 1);
		m_samplesUsed = lastSample;
	}
	else
	{
		lastSample = NearestNeighbor(m_rrtGraph, m_goal).first;
		tie(cost,backPath) = BacktrackFromSampleGetCost(m_rrtGraph, lastSample, 1);

		// Attempt to reach start from goal
		reached = FindPlan(m_goal, m_start);
		m_samplesUsed = m_nSamples;

		if(reached)
		{
			lastSample = m_rrtGraph.size()-1;
			tie(cost,backPath) = BacktrackFromSampleGetCost(m_rrtGraph, lastSample, 1);
			reverse = 1;
			m_samplesUsed += lastSample;
		}
		else { cost = D_INF;} // Both failed, path not found
	}

	m_planTime = double(clock() - begin_time)/CLOCKS_PER_SEC;
	m_pathQuality = cost;
	m_samplesAdded = m_samplesUsed;

	return GetFinalPlan(backPath, reverse);

}

