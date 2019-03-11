#include "GraphVertex.hpp"
using namespace std;

GraphVertex::GraphVertex(double* a, int n)
{
	qAngs = vector<double>(a, a+n);
	parent = -1;
	parCost = 0;
	pathCost = D_INF;
	m_angs = qAngs;
	m_numDOFs = n;
}

GraphVertex::GraphVertex(vector<double> a)
{
	qAngs = a;
	parent = -1;
	parCost = 0;
	pathCost = D_INF;
	m_angs = a;
	m_numDOFs = qAngs.size();
}

GraphVertex::GraphVertex(vector<double> a, double pCost) 
{ 
	qAngs = a; 
	parent = -1;
	parCost = pCost;
	pathCost = D_INF;
	m_angs = a; 
	m_numDOFs = qAngs.size(); 
}

GraphVertex::GraphVertex(vector<double> a, int p, double pCost) 
{ 
	qAngs = a; 
	parent = p;
	parCost = pCost;
	pathCost = D_INF;
	m_angs = a; 
	m_numDOFs = qAngs.size(); 
}

GraphVertex::~GraphVertex(){}