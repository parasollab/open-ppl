#ifndef BRUTEFORCENF_H_
#define BRUTEFORCENF_H_

#include "NeighborhoodFinderMethod.h"
#include <vector>

using namespace std;

class Environment;

template<class MPTraits>
class BruteForceNF : public NeighborhoodFinderMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType,WeightType> SequentialGraphType;

    BruteForceNF(MPProblemType* _problem = NULL, string _dmLabel = "", string _label = "") :
      NeighborhoodFinderMethod<MPTraits>(_problem, _dmLabel, _label) {
        this->SetName("BruteForceNF");
      }

    BruteForceNF(MPProblemType* _problem, XMLNodeReader& _node) :
      NeighborhoodFinderMethod<MPTraits>(_problem, _node) {
        this->SetName("BruteForceNF");
      }

    virtual ~BruteForceNF() {}

    template<typename InputIterator, typename OutputIterator>
      OutputIterator KClosest(RoadmapType* _rmp, 
          InputIterator _first, InputIterator _last, CfgType _cfg, size_t _k, OutputIterator _out);

    template<typename InputIterator, typename OutputIterator>
      OutputIterator KClosest(SequentialGraphType* _graph, 
          InputIterator _first, InputIterator _last, CfgType _cfg, size_t _k, OutputIterator _out);
    
    // KClosest that operate over two ranges of VIDS.  K total pair<VID,VID> are returned that
    // represent the _kclosest pairs of VIDs between the two ranges.
    template<typename InputIterator, typename OutputIterator>
      OutputIterator KClosestPairs(RoadmapType* _rmp,
          InputIterator _first1, InputIterator _last1, 
          InputIterator _first2, InputIterator _last2, 
          size_t _k, OutputIterator _out);
};

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator 
BruteForceNF<MPTraits>::KClosest(RoadmapType* _rmp, InputIterator _first, InputIterator _last, 
    CfgType _cfg, size_t _k, OutputIterator _out) {

  this->IncrementNumQueries();

  // TO DO NOTE: A temporary fix to support parallel runtime. The problem here is that is the way
  // we pass pointer around which is a bit ugly with parallelism. In this particular case
  // the pointer to GetMPProblem became invalid because of the way BruteForceNF is called from
  // Connector, thus call to timing stats below seg fault. One fix is to call BruteForceNF(_node, _problem)
  // constructor and this will be done when parallel code supports all NF. What this means is 
  // that I can not just support BruteForceNF by itself.
#ifndef _PARALLEL
  this->StartTotalTime();
  this->StartQueryTime();
#endif

  Environment* env = this->GetMPProblem()->GetEnvironment();
  GraphType* map = _rmp->GetGraph();
  DistanceMetricPointer dmm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);

  // Keep sorted list of k best so far
  priority_queue<pair<VID, double>, vector<pair<VID, double> >, CompareSecond<VID, double> > pq;
  for(InputIterator it = _first; it != _last; it++) {
    CfgType node = map->GetCfg(it);
    double dist = dmm->Distance(env, _cfg, node);

    if(node == _cfg) // Don't connect to self
      continue;
    if(pq.size() < _k){
      VID vid = map->GetVid(it);
      pq.push(make_pair(vid, dist));
    }
    // If better than the worst so far, replace worst so far
    else if(dist < pq.top().second) {
      pq.pop();
      VID vid = map->GetVid(it);
      pq.push(make_pair(vid, dist));
    }
  }

  // Transfer k closest to vector, sorted greatest to least dist
  vector<VID> closest;
  while(!pq.empty()){
    closest.push_back(pq.top().first);
    pq.pop();
  }

#ifndef _PARALLEL
  this->EndQueryTime();
  this->EndTotalTime();
#endif
  
  // Reverse order
  return copy(closest.rbegin(), closest.rend(), _out);
}

template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator 
BruteForceNF<MPTraits>::KClosestPairs(RoadmapType* _rmp,
    InputIterator _first1, InputIterator _last1, 
    InputIterator _first2, InputIterator _last2, size_t _k, OutputIterator _out) {

  Environment* env = this->GetMPProblem()->GetEnvironment();
  GraphType* map = _rmp->GetGraph();
  DistanceMetricPointer dmm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);
  priority_queue<pair<pair<VID, VID>, double>, vector<pair<pair<VID, VID>, double> >,
    CompareSecond<pair<VID, VID>, double> > pq;

  // Find all pairs
  for(InputIterator it1 = _first1; it1 != _last1; it1++) {
    CfgType node1 = map->GetCfg(it1);
    for(InputIterator it2 = _first2; it2 != _last2; it2++) {
      if(*it1 == *it2) // Don't connect to self
        continue;

      CfgType node2 = map->GetCfg(it2);
      double dist = dmm->Distance(env, node1, node2);
      if(pq.size() < _k){
        VID vid1 = map->GetVid(it1);
        VID vid2 = map->GetVid(it2);
        pq.push(make_pair(make_pair(vid1, vid2), dist));
      // If better than worst so far, replace worst so far
      }else if(dist < pq.top().second) {
        pq.pop();
        VID vid1 = map->GetVid(it1);
        VID vid2 = map->GetVid(it2);
        pq.push(make_pair(make_pair(vid1, vid2), dist));
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


// another special case for Radial Blind RRT :)


template<class MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator 
BruteForceNF<MPTraits>::KClosest(SequentialGraphType* _graph, InputIterator _first, InputIterator _last, 
    CfgType _cfg, size_t _k, OutputIterator _out) {

  this->IncrementNumQueries();

  // TO DO NOTE: A temporary fix to support parallel runtime. The problem here is that is the way
  // we pass pointer around which is a bit ugly with parallelism. In this particular case
  // the pointer to GetMPProblem became invalid because of the way BruteForceNF is called from
  // Connector, thus call to timing stats below seg fault. One fix is to call BruteForceNF(_node, _problem)
  // constructor and this will be done when parallel code supports all NF. What this means is 
  // that I can not just support BruteForceNF by itself.
#ifndef _PARALLEL
  this->StartTotalTime();
  this->StartQueryTime();
#endif

  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dmm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel);

  // Keep sorted list of k best so far
  priority_queue<pair<VID, double>, vector<pair<VID, double> >, CompareSecond<VID, double> > pq;
  for(InputIterator it = _first; it != _last; it++) {
    CfgType node = (*(_graph->find_vertex(*it))).property();
    double dist = dmm->Distance(env, _cfg, node);

    if(node == _cfg) // Don't connect to self
      continue;
    if(pq.size() < _k){
      // VID vid = map->GetVid(it);
      VID vid = (*it);
      pq.push(make_pair(vid, dist));
    }
    // If better than the worst so far, replace worst so far
    else if(dist < pq.top().second) {
      pq.pop();
      // VID vid = map->GetVid(it);
      VID vid = (*it);

      pq.push(make_pair(vid, dist));
    }
  }

  // Transfer k closest to vector, sorted greatest to least dist
  vector<VID> closest;
  while(!pq.empty()){
    closest.push_back(pq.top().first);
    pq.pop();
  }

#ifndef _PARALLEL
  this->EndQueryTime();
  this->EndTotalTime();
#endif
  
  // Reverse order
  return copy(closest.rbegin(), closest.rend(), _out);
}



#endif
