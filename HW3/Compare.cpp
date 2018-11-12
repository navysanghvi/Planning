#include "PlanSym.hpp"
#include "GraphVertex.hpp"

using namespace std;

/* Priority of vertex in graph in Dijkstra search
   (according to g-value), using a min-heap 
   Inputs: 2 Graph Vertices 
   Output: boolean after comparing path costs
*/
class Compare
{
public:
	bool operator () (GraphVertex* a, GraphVertex* b) const {
		if(a->m_fVal == b->m_fVal)
		{
			PlanSym p;
			if(p.Contains(a->m_state, a->m_goal)) {return false;}
			else if(p.Contains(b->m_state, b->m_goal)) {return true;}
			else { return(a->m_hVal > b->m_hVal); }
		}
		return (a->m_fVal > b->m_fVal);}
};