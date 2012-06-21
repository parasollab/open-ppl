#ifndef BFNF_H_
#define BFNF_H_

#include "NeighborhoodFinderMethod.hpp"
#include <vector>

using namespace std;

class Environment;

template<typename CFG, typename WEIGHT>
class BFNF: public NeighborhoodFinderMethod {

public:

  typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
  
  BFNF(XMLNodeReader& _node, MPProblem* _problem) :
      NeighborhoodFinderMethod(_node, _problem) {}
  
  BFNF(shared_ptr<DistanceMetricMethod> _dmm, string _label = "", MPProblem* _problem = NULL) :
      NeighborhoodFinderMethod(_dmm, _label, _problem) {}

  virtual ~BFNF() {}

  virtual const string GetName () const {
    return GetClassName();
  }

  static const string GetClassName() {
    return "BFNF";
  }

  virtual void PrintOptions(ostream& _os) const {
    _os << this->GetClassName() << endl;
  }

  template <typename InputIterator, typename OutputIterator>
  OutputIterator KClosest(Roadmap<CFG, WEIGHT>* _rmp, 
    InputIterator _inFirst, InputIterator _inLast, VID _v, int _k, OutputIterator _out);
  
  // do the work here, and have the function above obtain the CFG and call this one
  template <typename InputIterator, typename OutputIterator>
  OutputIterator KClosest(Roadmap<CFG, WEIGHT>* _rmp, 
    InputIterator _inFirst, InputIterator _inLast, CFG _cfg, int _k, OutputIterator _out);
  
  // KClosest that operate over the entire roadmap to find the _kclosest to a VID or CFG
  // NOTE: These are the prefered methods for _kClosest computations
  template <typename OutputIterator>
  OutputIterator KClosest(Roadmap<CFG, WEIGHT>* _rmp, 
    VID _v, int _k, OutputIterator _out);
  
  template <typename OutputIterator>
  OutputIterator KClosest(Roadmap<CFG, WEIGHT>* _rmp, 
    CFG _cfg, int _k, OutputIterator _out);
  
  // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
  // represent the _kclosest pairs of VIDs between the two ranges.
  template <typename InputIterator, typename OutputIterator>
  OutputIterator KClosestPairs(Roadmap<CFG, WEIGHT>* _rmp,
    InputIterator _in1First, InputIterator _in1Last, 
    InputIterator _in2First, InputIterator _in2Last, 
    int _k, OutputIterator _out);
};

template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator BFNF<CFG, WEIGHT>::KClosest(Roadmap<CFG, WEIGHT>* _rmp, 
    InputIterator _inFirst, InputIterator _inLast, VID _v, int _k, OutputIterator _out) {
  return KClosest(_rmp, _inFirst, _inLast, (_rmp->m_pRoadmap->find_vertex(_v))->property(), _k, _out);
}

template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator BFNF<CFG, WEIGHT>::KClosest(Roadmap<CFG, WEIGHT>* _rmp, 
  InputIterator _inFirst, InputIterator _inLast, CFG _cfg, int _k, OutputIterator _out) {
  
  IncrementNumQueries();
  
  // TO DO NOTE: A temporary fix to support parallel runtime. The problem here is that is the way
  // we pass pointer around which is a bit ugly with parallelism. In this particular case
  // the pointer to GetMPProblem became invalid because of the way BFNF is called from
  // Connector, thus call to timing stats below seg fault. One fix is to call BFNF(_node, _problem)
  // constructor and this will be done when parallel code supports all NF. What this means is 
  // that I can not just support BFNF by itself.
  #ifndef _PARALLEL
  StartTotalTime();
  StartQueryTime();
  #endif

  Environment* env = _rmp->GetEnvironment();
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;

  // Keep sorted list of k best so far
  priority_queue<pair<VID, double>, vector<pair<VID, double> >, compare_second<VID, double> > pq;
  for(InputIterator it = _inFirst; it != _inLast; it++) {
    CFG node = pmpl_detail::GetCfg<InputIterator>(pMap)(it);
    double dist = dmm->Distance(env, _cfg, node);
   
    if(node == _cfg) // Don't connect to self
      continue;
    if(pq.size() < (size_t)_k)
      pq.push(make_pair(*it, dist));
    // If better than the worst so far, replace worst so far
    else if(dist < pq.top().second) {
      pq.pop();
      pq.push(make_pair(*it, dist));
    }
  }
  
  // Transfer k closest to vector, sorted greatest to least dist
  vector<pair<VID, double> > closest;
  while(!pq.empty()) {
    closest.push_back(pq.top());
    pq.pop();
  }
  // Reverse order
  for(typename vector<pair<VID, double> >::reverse_iterator it = closest.rbegin(); it < closest.rend(); it++) {
    *_out = it->first;
    _out++;
  }

  #ifndef _PARALLEL
  EndQueryTime();
  EndTotalTime();
  #endif
  return _out;
}


template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator BFNF<CFG, WEIGHT>::KClosest(Roadmap<CFG, WEIGHT>* _rmp, 
    VID _v, int _k, OutputIterator _out) {
  return KClosest(_rmp, (_rmp->m_pRoadmap->find_vertex(_v))->property(), _k, _out);
}

template<typename CFG, typename WEIGHT>
template<typename OutputIterator>
OutputIterator BFNF<CFG, WEIGHT>::KClosest(Roadmap<CFG, WEIGHT>* _rmp, 
    CFG _cfg, int _k, OutputIterator _out) {
  return KClosest(_rmp, _rmp->m_pRoadmap->descriptor_begin(), _rmp->m_pRoadmap->descriptor_end(), _cfg, _k, _out);
}

template<typename CFG, typename WEIGHT>
template<typename InputIterator, typename OutputIterator>
OutputIterator BFNF<CFG, WEIGHT>::KClosestPairs(Roadmap<CFG, WEIGHT>* _rmp,
  InputIterator _in1First, InputIterator _in1Last, 
  InputIterator _in2First, InputIterator _in2Last, int _k, OutputIterator _out) {
 
  Environment* env = _rmp->GetEnvironment();
  RoadmapGraph<CFG,WEIGHT>* pMap = _rmp->m_pRoadmap;
  priority_queue<pair<pair<VID, VID>, double>, vector<pair<pair<VID, VID>, double> >,
      compare_second<pair<VID, VID>, double> > pq;

  // Find all pairs
  for(InputIterator it1 = _in1First; it1 != _in1Last; it1++) {
    CFG node1 = pmpl_detail::GetCfg<InputIterator>(pMap)(it1);
    for(InputIterator it2 = _in2First; it2 != _in2Last; it2++) {
      if(*it1 == *it2) // Don't connect to self
        continue;
      
      CFG node2 = pmpl_detail::GetCfg<InputIterator>(pMap)(it2);
      double dist = dmm->Distance(env, node1, node2);
      if(pq.size() < (size_t)_k)
        pq.push(make_pair(make_pair(*it1, *it2), dist));
      // If better than worst so far, replace worst so far
      else if(dist < pq.top().second) {
        pq.pop();
        pq.push(make_pair(make_pair(*it1, *it2), dist));
      }
    }
  }

  // Transfer k closest to vector, sorted greatest to least dist
  vector<pair<pair<VID, VID>, double> > closest;
  while(!pq.empty()) {
    closest.push_back(pq.top());
    pq.pop();
  }
  // Reverse order
  for(typename vector<pair<pair<VID, VID>, double> >::
      reverse_iterator it = closest.rbegin(); it < closest.rend(); it++) {
    *_out = it->first;
    _out++;
  }
  return _out;
}

#endif
