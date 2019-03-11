#include "PRMap.hpp"
using namespace std;

PRMap::PRMap(double* stt, double* end, int n, int x, int y, 
	double* m, double extS, int numS, double neighR, int k): PlanSample(stt, end, n, x, y, m) 
{
	m_gNeighRadius = neighR;
	m_extendStep = extS;
	m_degreeLimit = k;
	m_nSamples = numS;
	for(int i = 0; i < m_numDOFs; i++)
	{
		m_start[i] = AngleInRange(m_start[i]);
		m_goal[i] = AngleInRange(m_goal[i]);
	}
}

PRMap::PRMap(): PlanSample() {}

PRMap::~PRMap() {}

vector<double> PRMap::GetSample()
{
	vector<double> qrand(m_numDOFs);
	for(int i = 0; i < m_numDOFs; i++)
		qrand[i] = GetRandomSample(-M_PI, M_PI);
	return qrand;
}


vector<pair<int,double>> PRMap::NeighborIndices(GraphU_t graph, int maxInd, vector<double> qValid)
{
	vector<pair<int,double>> neighbors;
	double dist;
	for(int i = 0; i < maxInd; i++)
	{
		dist = sqrt(SquareDistance(graph[i]->qAngs, qValid));
		if(dist <= m_gNeighRadius)
			neighbors.push_back(pair<int,double>(i, dist));
	}
	return neighbors;
}

vector<bool> PRMap::HasPathToMe(GraphU_t graph,
	vector<pair<int,double>> neighs, vector<double> qValid)
{
	vector<double> qCheck;
	vector<bool> hasPath(neighs.size());
	for(int i = 0; i < neighs.size(); i++)
	{
		qCheck = vector<double>();
		if(boost::out_degree(neighs[i].first, graph) <= m_degreeLimit)
			qCheck = ExtendTo(graph, neighs[i].first, neighs[i].second, qValid);
		hasPath[i] = (!qCheck.empty() && SquareDistance(qValid, qCheck) <= 0.02) ? 1 : 0;
	}
	return hasPath;
}

vector<double> PRMap::ExtendTo(GraphU_t graph, int neighInd, double neighDist, vector<double> qValid)
{
	vector<double> qPrev = graph[neighInd]->qAngs;
	vector<double> qCheck, qInRange = qPrev;
	vector<double> qDiff(m_numDOFs);
	for(int i = 0; i < m_numDOFs; i++)
		qDiff[i] = WrapDifference(qPrev[i],qValid[i]);

	int step = 0; 
	int numSteps = ceil(neighDist/m_extendStep);
	double fracExtend = 1/((double)(numSteps));
	
	while(step++ < numSteps)
	{
		for(int i = 0; i < m_numDOFs; i++)
		{
			qPrev[i] += fracExtend*qDiff[i];
			qInRange[i] = AngleInRange(qPrev[i]);
		}
		if(IsValidArmConfiguration(qInRange)) { qCheck = qInRange; }
		else { break; }
	}
	return qCheck;
}


void PRMap::ConnectVertex(GraphU_t* graph, int maxInd, vector<double> qValid, 
	vector<pair<int,double>> neighs, vector<bool> hasPath)//, boost::disjoint_sets<GURank_t*,GUParent_t*>* ds)
{
	(*graph)[maxInd] = new GraphVertex(qValid);
	pair<GUEdge_t,bool> p;
	for(int i = 0; i < neighs.size(); i++)
		if(hasPath[i] && boost::out_degree(neighs[i].first, *graph) <= m_degreeLimit)
		{
			p = boost::add_edge(neighs[i].first, maxInd, *graph);
			(*graph)[p.first] = new GraphEdge(neighs[i].second);
			//(*ds).union_set(neighs[i].first, maxInd);
		}
}

void PRMap::ConstructGraph()
{
	vector<double> qValid;
	vector<pair<int,double>> neighs;
	vector<bool> hasPath;

	m_prmGraph = GraphU_t(m_nSamples+2);
	// vector<GURank_t> rank(m_nSamples+2); vector<GUParent_t> parent(m_nSamples+2);
	// boost::disjoint_sets<GURank_t*,GUParent_t*> disSet(&rank[0], &parent[0]);
	// boost::initialize_incremental_components(m_prmGraph, disSet);
	// boost::incremental_components(m_prmGraph, disSet);

	// while(!boost::same_component(m_nSamples, m_nSamples+1, disSet))
	{
		for(int i = 0; i < m_nSamples; i++)
		{
			do{ qValid = GetSample(); } while(!IsValidArmConfiguration(qValid));
			neighs = NeighborIndices(m_prmGraph, i, qValid);
			hasPath = HasPathToMe(m_prmGraph, neighs, qValid);
			ConnectVertex(&m_prmGraph, i, qValid, neighs, hasPath);//, &disSet);
		}

		neighs = NeighborIndices(m_prmGraph, m_nSamples, m_start);
		hasPath = HasPathToMe(m_prmGraph, neighs, m_start);
		ConnectVertex(&m_prmGraph, m_nSamples, m_start, neighs, hasPath);//, &disSet);
		//cout << "Start degree: " << boost::out_degree(m_nSamples, m_prmGraph) <<'\n';

		neighs = NeighborIndices(m_prmGraph, m_nSamples+1, m_goal);
		hasPath = HasPathToMe(m_prmGraph, neighs, m_goal);
		ConnectVertex(&m_prmGraph, m_nSamples+1, m_goal, neighs, hasPath);//, &disSet);
		//cout << "Goal degree: " << boost::out_degree(m_nSamples+1, m_prmGraph) <<'\n';

		//cout << "Number of Components: " << disSet.count_sets(boost::vertices(m_prmGraph).first, boost::vertices(m_prmGraph).second) << '\n';
		//cout << "Is same component? " << boost::same_component(m_nSamples, m_nSamples+1, disSet) << '\n';
	}
}

void PRMap::Dijkstra()
{
	pair<int,GraphVertex*>* curr;
	priority_queue<pair<int,GraphVertex*>*, vector<pair<int,GraphVertex*>*>, Compare> open;
	vector<bool> closed(m_nSamples+2, 0);
	GUInEdgeIt_t e1, e2;
	m_prmGraph[m_nSamples]->pathCost = 0;
	double g_curr = 0, g_neigh = D_INF;
	int neigh;

	open.push(new pair<int,GraphVertex*>(m_nSamples,m_prmGraph[m_nSamples]));

	while(!open.empty())
	{
		curr = open.top();
		while(closed[curr->first]) { open.pop(); curr = open.top(); }
		closed[curr->first] = 1;
		open.pop();

		if(curr->first == m_nSamples+1){ break; }

		g_curr = (curr->second)->pathCost;
		tie(e1,e2) = boost::in_edges(curr->first, m_prmGraph);
		for(GUInEdgeIt_t i = e1; i != e2; i++)
		{
			g_neigh = g_curr + m_prmGraph[(*i)]->edgeCost;
			neigh = boost::source(*i, m_prmGraph);
			if(m_prmGraph[neigh]->pathCost > g_neigh)
			{
				m_prmGraph[neigh]->pathCost = g_neigh;
				open.push(new pair<int,GraphVertex*>(neigh,m_prmGraph[neigh]));
			}
		}
	}
}

vector<vector<double>> PRMap::BacktrackFromSample(GraphU_t graph, int sample)
{
	vector<vector<double>> backPath;
	bool reached = 0;
	int curr = sample, next;
	GUInEdgeIt_t e1, e2;
	double neighCost, minCost;
	int neigh;

	backPath.push_back(graph[sample]->qAngs);
	if(curr == m_nSamples){ reached = 1; }
	while(!reached)
	{
		minCost = D_INF; next = -1;
		tie(e1,e2) = boost::in_edges(curr, graph);
		for(GUInEdgeIt_t i = e1; i != e2; i++)
		{
			neigh = boost::source(*i, graph);
			neighCost = graph[neigh]->pathCost + graph[(*i)]->edgeCost;
			if(neighCost < minCost)
			{
				minCost = neighCost;
				next = neigh;
			}
		}
		backPath.push_back(graph[next]->qAngs);
		curr = next;
		if(curr == m_nSamples){ reached = 1; }
	}
	return backPath;
}

pair<int, double**> PRMap::GetFinalPlan(vector<vector<double>> backPath, bool reverse)
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


pair<int, double**> PRMap::RunPlan()
{
	vector<vector<double>> backPath;

	clock_t begin_time = clock();

	ConstructGraph();
	Dijkstra();
	if(m_prmGraph[m_nSamples+1]->pathCost < D_INF)
	{
		backPath = BacktrackFromSample(m_prmGraph, m_nSamples+1);
		//cout << "Path cost = "<< m_prmGraph[m_nSamples+1]->pathCost << '\n';
		//cout << "Time taken = " << double(clock() - begin_time)/CLOCKS_PER_SEC << " s\n";

		m_samplesUsed = m_nSamples;
		m_samplesAdded = m_samplesUsed;
		m_planTime = double(clock() - begin_time)/CLOCKS_PER_SEC;
		m_pathQuality = m_prmGraph[m_nSamples+1]->pathCost;
		return GetFinalPlan(backPath, 0);
	}
	else
	{ 
		m_samplesUsed = m_nSamples;
		m_planTime = double(clock() - begin_time)/CLOCKS_PER_SEC;
		m_pathQuality = m_prmGraph[m_nSamples+1]->pathCost;
		//cout << "No path found.\n"; 
		//cout << "Time taken = " << double(clock() - begin_time)/CLOCKS_PER_SEC << " s\n";
		return (pair<int, double**>());}


}