#ifndef GRAPHEDGE_HPP
#define GRAPHEDGE_HPP

using namespace std;

/* Structure for edge in graph*/
class GraphEdge
{
public:
	GraphEdge(double e);
	~GraphEdge();
	double edgeCost;
};

#endif