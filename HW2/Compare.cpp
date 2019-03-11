#include "GraphVertex.hpp"
using namespace std;


class Compare
{
public:
	bool operator () (pair<int,GraphVertex*>* a, pair<int,GraphVertex*>* b) const {return ((a->second)->pathCost > (b->second)->pathCost);}
};