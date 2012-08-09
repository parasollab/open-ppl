#ifndef BRUTEFORCENF_H_
#define BRUTEFORCENF_H_

#include "NeighborhoodFinderMethod.h"
#include <vector>

using namespace std;

class Environment;

class BruteForceNF: public NeighborhoodFinderMethod {
  public:
    BruteForceNF(string _dmLabel = "", string _label = "", MPProblem* _problem = NULL) :
      NeighborhoodFinderMethod(_dmLabel, _label, _problem) {
        this->SetName("BruteForceNF");
      }

    BruteForceNF(XMLNodeReader& _node, MPProblem* _problem) :
      NeighborhoodFinderMethod(_node, _problem) {
        this->SetName("BruteForceNF");
        if(this->m_debug)
          PrintOptions(cout);
      }

    virtual ~BruteForceNF() {}

    template<typename RDMP, typename InputIterator, typename OutputIterator>
      OutputIterator KClosest(RDMP* _rmp, 
          InputIterator _first, InputIterator _last, typename RDMP::CfgType _cfg, size_t _k, OutputIterator _out);

    // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
    // represent the _kclosest pairs of VIDs between the two ranges.
    template<typename RDMP, typename InputIterator, typename OutputIterator>
      OutputIterator KClosestPairs(RDMP* _rmp,
          InputIterator _first1, InputIterator _last1, 
          InputIterator _first2, InputIterator _last2, 
          size_t _k, OutputIterator _out);
};

template<typename RDMP, typename InputIterator, typename OutputIterator>
OutputIterator 
BruteForceNF::KClosest(RDMP* _rmp, InputIterator _first, InputIterator _last, 
    typename RDMP::CfgType _cfg, size_t _k, OutputIterator _out) {

  typedef typename RDMP::VID VID;
  typedef typename RDMP::CfgType CFG;
  typedef typename RDMP::RoadmapGraphType RoadmapGraphType;
  typedef typename pmpl_detail::GetCfg<RoadmapGraphType> GetCfg;

  IncrementNumQueries();

  // TO DO NOTE: A temporary fix to support parallel runtime. The problem here is that is the way
  // we pass pointer around which is a bit ugly with parallelism. In this particular case
  // the pointer to GetMPProblem became invalid because of the way BruteForceNF is called from
  // Connector, thus call to timing stats below seg fault. One fix is to call BruteForceNF(_node, _problem)
  // constructor and this will be done when parallel code supports all NF. What this means is 
  // that I can not just support BruteForceNF by itself.
#ifndef _PARALLEL
  StartTotalTime();
  StartQueryTime();
#endif

  Environment* env = _rmp->GetEnvironment();
  RoadmapGraphType* pMap = _rmp->m_pRoadmap;
  shared_ptr<DistanceMetricMethod> dmm = GetDMMethod();

  // Keep sorted list of k best so far
  priority_queue<pair<VID, double>, vector<pair<VID, double> >, CompareSecond<VID, double> > pq;
  for(InputIterator it = _first; it != _last; it++) {
    CFG node = GetCfg()(pMap, it);
    double dist = dmm->Distance(env, _cfg, node);

    if(node == _cfg) // Don't connect to self
      continue;
    if(pq.size() < _k)
      pq.push(make_pair(*it, dist));
    // If better than the worst so far, replace worst so far
    else if(dist < pq.top().second) {
      pq.pop();
      pq.push(make_pair(*it, dist));
    }
  }

  // Transfer k closest to vector, sorted greatest to least dist
  vector<VID> closest;
  while(!pq.empty()){
    closest.push_back(pq.top().first);
    pq.pop();
  }

#ifndef _PARALLEL
  EndQueryTime();
  EndTotalTime();
#endif
  
  // Reverse order
  return copy(closest.rbegin(), closest.rend(), _out);
}

template<typename RDMP, typename InputIterator, typename OutputIterator>
OutputIterator 
BruteForceNF::KClosestPairs(RDMP* _rmp,
    InputIterator _first1, InputIterator _last1, 
    InputIterator _first2, InputIterator _last2, size_t _k, OutputIterator _out) {

  typedef typename RDMP::VID VID;
  typedef typename RDMP::CfgType CFG;
  typedef typename RDMP::RoadmapGraphType RoadmapGraphType;
  typedef typename pmpl_detail::GetCfg<RoadmapGraphType> GetCfg;

  Environment* env = _rmp->GetEnvironment();
  RoadmapGraphType* pMap = _rmp->m_pRoadmap;
  shared_ptr<DistanceMetricMethod> dmm = GetDMMethod();
  priority_queue<pair<pair<VID, VID>, double>, vector<pair<pair<VID, VID>, double> >,
    CompareSecond<pair<VID, VID>, double> > pq;

  // Find all pairs
  for(InputIterator it1 = _first1; it1 != _last1; it1++) {
    CFG node1 = GetCfg()(pMap, it1);
    for(InputIterator it2 = _first2; it2 != _last2; it2++) {
      if(*it1 == *it2) // Don't connect to self
        continue;

      CFG node2 = GetCfg()(pMap, it2);
      double dist = dmm->Distance(env, node1, node2);
      if(pq.size() < _k)
        pq.push(make_pair(make_pair(*it1, *it2), dist));
      // If better than worst so far, replace worst so far
      else if(dist < pq.top().second) {
        pq.pop();
        pq.push(make_pair(make_pair(*it1, *it2), dist));
      }
    }
  }

  // Transfer k closest to vector, sorted greatest to least dist
  vector<pair<VID, VID> > closest;
  while(!pq.empty()) {
    closest.push_back(pq.top(). first);
    pq.pop();
  }
  // Reverse order
  return copy(closest.rbegin(), closest.rend(), _out);
}

#endif
