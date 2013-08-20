#ifndef _MAX_FLOW_EVALUATION_H
#define _MAX_FLOW_EVALUATION_H

#include "GraphAlgo.h"

template <class CFG, class WEIGHT, class CAPACITY>
class MaxFlowEvaluation : public MapEvaluationMethod<CFG,WEIGHT> {
 public:
  VID source, sink;
  double flow_size;

  virtual bool evaluate(Roadmap<CFG,WEIGHT>* rmap) {
    CAPACITY capacity(*(rmap->m_pRoadmap));
    double flow = max_flow(*(rmap->m_pRoadmap), source, sink, capacity);
    return (flow >= flow_size);
  }
};

template <class InputIterator, class EqualityComparable>
InputIterator
find_first(InputIterator first, InputIterator last,
           const EqualityComparable& value) {
  while((first != last) && !(value == first->first))
    first++;
  return first;
};

template <class GRAPH>
struct ConstantCapacity {
  typedef typename GRAPH::EI EI;
  vector<pair<EI, double> > capacity_map;

  ConstantCapacity(GRAPH& G) {
    vector<VID> vids;
    G.GetVerticesVID(vids);
    typename GRAPH::VI v;
    EI e;
    for(typename vector<VID>::const_iterator V = vids.begin(); V != vids.end();
++V) {
      G.IsVertex(*V, &v);
      for(e = v->edgelist.begin(); e != v->edgelist.end(); ++e)
        capacity_map.push_back(make_pair(e, 1));
    }
  }
  ~ConstantCapacity() {}

  void SetCapacity(EI e, double c) {
    if(c <= 0) {
      typename vector<pair<EI, double> >::iterator CM =
        find_first(capacity_map.begin(), capacity_map.end(), e);
      if(CM != capacity_map.end())
        CM->second = 0;
    }
  }

  double GetCapacity(EI e) const {
    typename vector<pair<EI, double> >::const_iterator CM =
      find_first(capacity_map.begin(), capacity_map.end(), e);
    if(CM != capacity_map.end())
      return CM->second;
  }
};

template <class GRAPH>
struct InverseWeightCapacity {
  typedef typename GRAPH::EI EI;
  vector<pair<EI, double> > capacity_map;

  InverseWeightCapacity(GRAPH& G) {
    vector<VID> vids;
    G.GetVerticesVID(vids);
    typename GRAPH::VI v;
    EI e;
    for(typename vector<VID>::const_iterator V = vids.begin(); V != vids.end();
++V) {
      G.IsVertex(*V, &v);
      for(e = v->edgelist.begin(); e != v->edgelist.end(); ++e)
        capacity_map.push_back(make_pair(e, 1/e->weight.Weight()));
    }
  }
  ~InverseWeightCapacity() {}

  void SetCapacity(EI e, double c) {
    typename vector<pair<EI, double> >::iterator CM =
      find_first(capacity_map.begin(), capacity_map.end(), e);
    if(CM != capacity_map.end())
      if(c <= 0)
        CM->second = 0;
      else
        CM->second = c;
  }

  double GetCapacity(EI e) const {
    typename vector<pair<EI, double> >::const_iterator CM =
      find_first(capacity_map.begin(), capacity_map.end(), e);
    if(CM != capacity_map.end())
      return CM->second;
  }
};

#endif
