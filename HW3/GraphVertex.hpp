#ifndef GRAPHVERTEX_HPP
#define GRAPHVERTEX_HPP

#include "planner_classes.cpp"

using namespace std;

/* Class for vertex in graph */
class GraphVertex
{
public:
	

	typedef unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> GState_t;
    typedef unordered_set<Action, ActionHasher, ActionComparator> Actions_t;
    typedef GroundedAction GAction_t;
    typedef unordered_set<string> Sym_t;

    GraphVertex(GState_t* state, double pathCost, GraphVertex* parent, GAction_t* action, GState_t goal);
    GraphVertex(GState_t* state, double pathCost, GState_t goal);
	GraphVertex(GState_t* state);
	~GraphVertex();

	double Set_fVal();
	friend ostream& operator<<(ostream& os, const GraphVertex& vertex);

	GState_t* m_state;					/* vertex state description */
	double m_pCost;						/* cost from start to vertex */
	vector<GraphVertex*> m_parents;		/* parents in graph */
	vector<GAction_t*> m_parActions;	/* actions leading to vertex from parent vertices */
	double m_hVal;
	double m_fVal;
	GState_t m_goal;
	

};

#endif