#include "PlanSample.hpp"
using namespace std;

/* Constructor */
PlanSample::PlanSample(double* stt, double* end, int n, int x, int y, double* m): m_numDOFs(n), m_xsz(x), m_ysz(y), m_map(m) 
{
	m_start = vector<double>(stt,stt+n);
	m_goal = vector<double>(end,end+n);
}

/* Default Constructor */
PlanSample::PlanSample(): PlanSample(NULL,NULL,0,0,0,NULL) {}

/* Destructor */
PlanSample::~PlanSample(){}

/* Get random double sample between lo and hi */
double PlanSample::GetRandomSample(double lo, double hi)
{
	mt19937 eng(m_randEngine());
	uniform_real_distribution<double> realDist(lo, hi);
	return realDist(eng);
}

/* Normalize angle q to between -pi and pi */
double PlanSample::AngleInRange(double q) 
{ 
	q = WrapDifference(0,q);
	return q;
}

/* Find normalized difference between normalized angles q1, q2 */
double PlanSample::WrapDifference(double q1, double q2)
{
	double diff = q2 - q1;
	//return diff;
	if(diff > M_PI) { diff += ((double)(2))*(-M_PI); }
	else if(diff <= -M_PI) { diff += ((double)(2))*M_PI; }
	if(diff <= -M_PI || diff > M_PI){ throw; }
	return diff;
}

/* Find square of distance between vectors q1, q2 */
double PlanSample::SquareDistance(vector<double> q1, vector<double> q2)
{
	int i = 0;
	double sDistance = 0; 
	for(int i = 0; i < m_numDOFs; i++)
		sDistance += pow(WrapDifference(q1[i], q2[i]),2.0);
	return sDistance;
}

/* Find index and square distance of vertex in undirected graph g closest to qrand */
pair<int,double> PlanSample::NearestNeighbor(GraphU_t g, vector<double> qrand)
{
	int nearest = -1;
	double sDistance;
	double minDistance = D_INF;
	int gSize = boost::num_vertices(g);
	for(int i = 0; i < gSize; i++)
	{
		sDistance = SquareDistance(g[i]->qAngs, qrand);
		if(sDistance < minDistance)
		{
			minDistance = sDistance;
			nearest = i;
		}
	}
	return pair<int,double>(nearest,minDistance);
}

/* Find index and square distance of vertex in undirected graph g closest to qrand */
pair<int,double> PlanSample::NearestNeighbor(GraphD_t g, vector<double> qrand)
{
	int nearest = -1;
	double sDistance;
	double minDistance = D_INF;
	int gSize = boost::num_vertices(g);
	for(int i = 0; i < gSize; i++)
	{
		sDistance = SquareDistance(g[i]->qAngs, qrand);
		if(sDistance < minDistance)
		{
			minDistance = sDistance;
			nearest = i;
		}
	}
	return pair<int,double>(nearest,minDistance);
}

/* Find index and square distance of vertex in vector of vertices g closest to qrand */
pair<int,double> PlanSample::NearestNeighbor(vector<GraphVertex*> g, vector<double> qrand)
{
	int nearest = -1;
	double sDistance;
	double minDistance = D_INF;
	int gSize = g.size();
	for(int i = 0; i < gSize; i++)
	{
		sDistance = SquareDistance(g[i]->qAngs, qrand);
		if(sDistance < minDistance)
		{
			minDistance = sDistance;
			nearest = i;
		}
	}
	return pair<int,double>(nearest,minDistance);
}


/* All the following methods are used to determine if a path is collision-free.
   These were given as part of the question */

void PlanSample::ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY)
{
    double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= m_xsz) *pX = m_xsz-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= m_ysz) *pY = m_ysz-1;
}


void PlanSample::get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
    {
      params->Y1=p1x;
      params->X1=p1y;
      params->Y2=p2x;
      params->X2=p2y;
    }
  else
    {
      params->X1=p1x;
      params->Y1=p1y;
      params->X2=p2x;
      params->Y2=p2y;
    }

   if ((p2x - p1x) * (p2y - p1y) < 0)
    {
      params->Flipped = 1;
      params->Y1 = -params->Y1;
      params->Y2 = -params->Y2;
    }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void PlanSample::get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
        *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
        *y = -*y;
    }
}

int PlanSample::get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}

int PlanSample::GetMapIndex(int x, int y) { return(y*m_xsz + x); }

int PlanSample::IsValidLineSegment(double x0, double y0, double x1, double y1)
{
	bresenham_param_t params;
	int nX, nY; 
    short unsigned int nX0, nY0, nX1, nY1;

    //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
    
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= m_xsz ||
		x1 < 0 || x1 >= m_xsz ||
		y0 < 0 || y0 >= m_ysz ||
		y1 < 0 || y1 >= m_ysz)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0);
	ContXY2Cell(x1, y1, &nX1, &nY1);

    //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(m_map[GetMapIndex(nX,nY)] == 1)
            return 0;
	} while (get_next_point(&params));

	return 1;
}

bool PlanSample::IsValidArmConfiguration(vector<double> angles)
{
    double x0,y0,x1,y1;
    int i;
    
 	//iterate through all the links starting with the base
	x1 = ((double)m_xsz)/2.0;
    y1 = 0;
	for(i = 0; i < m_numDOFs; i++)
	{
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*M_PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*M_PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1))
				return 0;
	}
	return 1;    
}
