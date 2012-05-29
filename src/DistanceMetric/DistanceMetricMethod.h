#ifndef DISTANCEMETRICMETHOD_h
#define DISTANCEMETRICMETHOD_h

#include "MPUtils.h"
#include "Roadmap.h"
#include "CfgTypes.h"
#include <boost/mpl/list.hpp>
using namespace std;

template <class CFG, class WEIGHT> class LocalPlanners;
template <class CFG, class WEIGHT> class LocalPlannerMethod;
class GMSPolyhedron;
class MPProblem;

const double MAX_DIST =  1e10;

const int CS = 0;   ///< Type CS: Configuration space distance metric
const int WS = 1;   ///< Type WS: Workspace distance metric 


/**This is the interface for all distance metric methods(euclidean, 
  *scaledEuclidean, minkowski, manhattan, com, etc.).
  */
class DistanceMetricMethod  : public MPBaseObject {
 public:
  DistanceMetricMethod();
  DistanceMetricMethod(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
  virtual ~DistanceMetricMethod();

  typedef ElementSet<LocalPlannerMethod<CfgType,WeightType> >::MethodPointer LocalPlannerPointer;

  string GetName() const {return m_name;}
  
  virtual void PrintOptions(ostream& _os) const;
  
  template <class CFG, class WEIGHT>
  vector<typename RoadmapGraph<CFG, WEIGHT>::VID> RangeQuery(Roadmap<CFG, WEIGHT>* _rm,
                   typename RoadmapGraph<CFG, WEIGHT>::VID _query, double _radius);
  
  template <class CFG, class WEIGHT>

  vector<typename RoadmapGraph<CFG, WEIGHT>::VID> RangeQuery(Roadmap<CFG, WEIGHT>* _rm,
                         CFG _query, double _radius); 

  virtual double Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) = 0;
  virtual void ScaleCfg(Environment* _env, double _length, Cfg& _o, Cfg& _c, bool _norm=true);

 protected:
  string m_name;
};
ostream& operator<< (ostream& _os, const DistanceMetricMethod& _dm);


template <class CFG, class WEIGHT>
 vector<typename RoadmapGraph<CFG, WEIGHT>::VID> DistanceMetricMethod::
RangeQuery(Roadmap<CFG, WEIGHT>* _rm, typename RoadmapGraph<CFG, WEIGHT>::VID _query, double _radius) {
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  vector<VID> neighbors;
  RoadmapGraph<CFG,WEIGHT>* pMap = _rm->m_pRoadmap;
  Environment* env = _rm->GetEnvironment();
  vector<VID> vids;
  pMap->GetVerticesVID(vids);
  typename vector<VID>::iterator itr;
  for(itr = vids.begin(); itr != vids.end(); ++itr)
  {
    if(_query == *itr) continue;
    double dist = this->Distance(env, (*(pMap->find_vertex(_query))).property(), (*(pMap->find_vertex(*itr))).property());
    if( dist< _radius) {
      neighbors.push_back(*itr);
    }
  }

  return neighbors;
}

template <class CFG, class WEIGHT>
vector<typename RoadmapGraph<CFG, WEIGHT>::VID> DistanceMetricMethod::
RangeQuery(Roadmap<CFG, WEIGHT>* _rm, CFG _query, double _radius) {
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  vector<VID> neighbors;
  RoadmapGraph<CFG,WEIGHT>* pMap = _rm->m_pRoadmap;
  Environment* env = _rm->GetEnvironment();
  vector<VID> vids;
  pMap->GetVerticesVID(vids);
  typename vector<VID>::iterator itr;

  for(itr = vids.begin(); itr != vids.end(); ++itr)
  {
    if(_query == (*(pMap->find_vertex(*itr))).property()) continue;
    if(this->Distance(env, _query, (*(pMap->find_vertex(*itr))).property()) < _radius) {
      neighbors.push_back(*itr);
    }
  }

  return neighbors;
}


#endif
