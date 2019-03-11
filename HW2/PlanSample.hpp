#ifndef PLANSAMPLE_HPP
#define PLANSAMPLE_HPP

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/incremental_components.hpp>
#include <cmath>
#include <math.h>
#include <limits>
#include <random>
#include <iostream>
#include "GraphVertex.hpp"
#include "GraphEdge.hpp"
#define D_INF numeric_limits<double>::infinity()
#define LINKLENGTH_CELLS 10
using namespace std;

class PlanSample
{
public:
    PlanSample(double* stt, double* end, int n, int x, int y, double* m);
    PlanSample();
    ~PlanSample();

    typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, GraphVertex*, GraphEdge*> GraphU_t;
    typedef boost::graph_traits<GraphU_t>::vertex_descriptor GUVertex_t;
    typedef boost::graph_traits<GraphU_t>::vertices_size_type GUVertSz_t;
    typedef boost::graph_traits<GraphU_t>::edge_descriptor GUEdge_t;
    typedef boost::graph_traits<GraphU_t>::degree_size_type GUDegree_t;
    typedef GraphU_t::vertex_iterator GUVertexIt_t;
    typedef GraphU_t::edge_iterator GUEdgeIt_t;
    typedef GraphU_t::in_edge_iterator GUInEdgeIt_t;
    typedef GUVertSz_t GURank_t;
    typedef GUVertex_t GUParent_t;

    typedef boost::adjacency_list<boost::setS, boost::vecS, boost::directedS, GraphVertex*, GraphEdge*> GraphD_t;
    typedef boost::graph_traits<GraphD_t>::vertex_descriptor GDVertex_t;
    typedef boost::graph_traits<GraphD_t>::edge_descriptor GDEdge_t;
    typedef GraphD_t::vertex_iterator GDVertexIt_t;
    typedef GraphD_t::edge_iterator GDEdgeIt_t;
    typedef GraphD_t::in_edge_iterator GDInEdgeIt_t;

    typedef struct {
      int X1, Y1;
      int X2, Y2;
      int Increment;
      int UsingYIndex;
      int DeltaX, DeltaY;
      int DTerm;
      int IncrE, IncrNE;
      int XIndex, YIndex;
      int Flipped;
    } bresenham_param_t;


    double GetRandomSample(double lo, double hi);
    double AngleInRange(double q);
    double WrapDifference(double q1, double q2);
    double SquareDistance(vector<double> q1, vector<double> q2);
    pair<int, double> NearestNeighbor(GraphU_t g, vector<double> qrand);
    pair<int, double> NearestNeighbor(GraphD_t g, vector<double> qrand);
    pair<int, double> NearestNeighbor(vector<GraphVertex*> g, vector<double> qrand);

    int GetMapIndex(int x, int y);
    void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY);
    void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params);
    void get_current_point(bresenham_param_t *params, int *x, int *y);
    int get_next_point(bresenham_param_t *params);
    int IsValidLineSegment(double x0, double y0, double x1, double y1);
    bool IsValidArmConfiguration(vector<double> angles);


protected:
    int m_numDOFs;
    int m_xsz;
    int m_ysz;
    double* m_map;
    vector<double> m_start;
    vector<double> m_goal;
    random_device m_randEngine;

};

#endif