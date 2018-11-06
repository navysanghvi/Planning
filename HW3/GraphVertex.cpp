#include "GraphVertex.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <math.h>

#define D_INF numeric_limits<double>::infinity()
using namespace std;


GraphVertex::GraphVertex(GState_t* state, double pathCost, GraphVertex* parent, GAction_t* parAction)
{
	m_state = state;
	m_pCost = pathCost;
	m_parents.push_back(parent);
	m_parActions.push_back(parAction);
}


/* GraphVertex constructor
   Input: (1) state (pointer to unordered set of grounded conditions)
          (2) path cost from start to vertex (double) 
*/
GraphVertex::GraphVertex(GState_t* state, double pathCost)
{
	m_state = state;
	m_pCost = pathCost;
}


GraphVertex::GraphVertex(GState_t* state)
{
	m_state = state;
	m_pCost = D_INF;
}