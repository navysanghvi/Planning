#ifndef GRAPHVERTEX_HPP
#define GRAPHVERTEX_HPP

#include <vector>
#include <limits>
#define D_INF numeric_limits<double>::infinity()
using namespace std;

/* Class for vertex in graph */
class GraphVertex
{
public:
	GraphVertex(double* a, int n);
	GraphVertex(vector<double> a);
	GraphVertex(vector<double> a, double pCost);
	GraphVertex(vector<double> a, int p, double pCost);
	~GraphVertex();
	vector<double> qAngs; 	/* joint angles */
	int parent; 			/* index of parent 
								(for tree structures) */
	double parCost;			/* cost from parent to vertex 
								(for tree structures) */
	double pathCost;		/* cost from start to vertex */

private:
	vector<double> m_angs;	/* member: joint angles */
	int m_numDOFs;			/* member: number of DOFs*/
};

#endif