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
		return (a->m_fVal > b->m_fVal);}
};