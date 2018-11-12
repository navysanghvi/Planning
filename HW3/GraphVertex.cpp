#include "GraphVertex.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <math.h>

#define D_INF numeric_limits<double>::infinity()
using namespace std;


GraphVertex::GraphVertex(GState_t* state, double pathCost, GraphVertex* parent, GAction_t* parAction, GState_t goal)
{
	m_state = state;
	m_pCost = pathCost;
	m_parents.push_back(parent);
	m_parActions.push_back(parAction);
	m_hVal = 0;
	m_fVal = m_pCost + m_hVal;
	m_goal = goal;
}


/* GraphVertex constructor
   Input: (1) state (pointer to unordered set of grounded conditions)
          (2) path cost from start to vertex (double) 
*/
GraphVertex::GraphVertex(GState_t* state, double pathCost, GState_t goal)
{
	m_state = state;
	m_pCost = pathCost;
	m_hVal = 0;
	m_fVal = m_pCost + m_hVal;
	m_goal = goal;
}


GraphVertex::GraphVertex(GState_t* state)
{
	m_state = state;
	m_pCost = D_INF;
	m_hVal = 0;
	m_fVal = D_INF;
}

double GraphVertex::Set_fVal()
{
	m_fVal = (m_pCost == D_INF || m_hVal == D_INF) ? D_INF : (m_pCost + m_hVal);
}

ostream& operator<<(ostream& os, const GraphVertex& vertex)
{
	os << "\nState Conditions: ";
	for(auto cond: *(vertex.m_state))
    	os << cond.toString() << " ";
    os << "\nG value = " << vertex.m_pCost;
    os << " ;   H value = " << vertex.m_hVal;
    os << " ;   F value = " << vertex.m_fVal << endl;
    return os;
}