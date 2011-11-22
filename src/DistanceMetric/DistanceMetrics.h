/**
 * @file DistanceMetrics.h
 *
 * @author Daniel Vallejo
 * @date 8/21/1998
 */

////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DistanceMetrics_h
#define DistanceMetrics_h

#include "DistanceMetricMethod.h"
#include "PMPL_Element_Set.h"
#include "util.h"

//////////////////////////////////////////////////////////////////////////////////////////

class EuclideanDistance;
class ScaledEuclideanDistance;
class MinkowskiDistance;
class ManhattanDistance;
class CenterOfMassDistance;
class RmsdDistance;
class LPSweptDistance;
class BinaryLPSweptDistance;
class KnotTheoryDistance;
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
class ReachableDistance;
#endif


namespace pmpl_detail { //hide DistanceMetricMethodList in pmpl_detail namespace
  typedef boost::mpl::list<
   EuclideanDistance,
   ScaledEuclideanDistance,
   MinkowskiDistance,
   ManhattanDistance,
   CenterOfMassDistance,
   RmsdDistance,
   LPSweptDistance,
   BinaryLPSweptDistance,
   #if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
   ReachableDistance, 
  #endif
   KnotTheoryDistance
    > DistanceMetricMethodList;
}


class DistanceMetric : private element_set<DistanceMetricMethod>, public MPBaseObject 
{
 public:
  typedef element_set<DistanceMetricMethod>::method_pointer DistanceMetricPointer;

 public:

  template<typename MethodList>
  DistanceMetric() : element_set<DistanceMetricMethod>(MethodList()) {}

  DistanceMetric() : element_set<DistanceMetricMethod>(pmpl_detail::DistanceMetricMethodList()) {}
  
  template <typename MethodList>
  DistanceMetric(XMLNodeReader& in_Node, MPProblem* in_pProblem, MethodList const&, bool parse_xml = true)
   : element_set<DistanceMetricMethod>(MethodList()), MPBaseObject(in_pProblem) 
  {
    for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr)
      if(!element_set<DistanceMetricMethod>::add_element(citr->getName(), *citr, in_pProblem))
        citr->warnUnknownNode();
  }
  DistanceMetric(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool parse_xml = true)
   : element_set<DistanceMetricMethod>(pmpl_detail::DistanceMetricMethodList()), MPBaseObject(in_pProblem) 
  {
    for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
      if(!element_set<DistanceMetricMethod>::add_element(citr->getName(), *citr, in_pProblem))
        citr->warnUnknownNode();
    }
  }
  virtual ~DistanceMetric();

  DistanceMetricPointer GetDMMethod(string in_strLabel);
  void AddDMMethod(string in_strLabel, DistanceMetricPointer in_ptr);

  void PrintOptions(ostream& _os) const;

};


/*
#include "Roadmap.h"
template <class CFG, class WEIGHT>
 vector<typename RoadmapGraph<CFG, WEIGHT>::VID> DistanceMetricMethod::
RangeQuery(Roadmap<CFG, WEIGHT>* rm, typename RoadmapGraph<CFG, WEIGHT>::VID in_query, double in_radius) {
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  vector<VID> returnVec;
  RoadmapGraph<CFG,WEIGHT>* pMap = rm->m_pRoadmap;
  Environment* _env = rm->GetEnvironment();


  Clock_Class distance_time;
  distance_time.StartClock("distance_time");

  vector<VID> vec_vids;
  pMap->GetVerticesVID(vec_vids);
  typename vector<VID>::iterator itr;
  for(itr = vec_vids.begin(); itr != vec_vids.end(); ++itr)
  {
    if(in_query == *itr) continue;
  double dist = this->Distance(_env, (*(pMap->find_vertex(in_query))).property(), (*(pMap->find_vertex(*itr))).property());
    //cout << "Distance = " << dist << " Radius = " << in_radius << endl;
    if( dist< in_radius) {
      returnVec.push_back(*itr);
    }
  }
  distance_time.StopClock();
  m_distance_time += distance_time.GetClock_SEC();

  return returnVec;
}


template <class CFG, class WEIGHT>
vector<typename RoadmapGraph<CFG, WEIGHT>::VID> DistanceMetricMethod::
RangeQuery(Roadmap<CFG, WEIGHT>* rm, CFG in_query, double in_radius) {
  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  vector<VID> returnVec;
  RoadmapGraph<CFG,WEIGHT>* pMap = rm->m_pRoadmap;
  Environment* _env = rm->GetEnvironment();
  
  Clock_Class distance_time;
  distance_time.StartClock("distance_time");
  


  vector<VID> vec_vids;
  pMap->GetVerticesVID(vec_vids);
  typename vector<VID>::iterator itr;

  for(itr = vec_vids.begin(); itr != vec_vids.end(); ++itr)
  {
    if(in_query == (*(pMap->find_vertex(*itr))).property()) continue;
    if(this->Distance(_env, in_query, (*(pMap->find_vertex(*itr))).property()) < in_radius) {
      returnVec.push_back(*itr);
    }
  }
  distance_time.StopClock();
  m_distance_time += distance_time.GetClock_SEC();

  return returnVec;
}
*/

#endif
