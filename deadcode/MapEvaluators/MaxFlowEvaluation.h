#ifndef MAX_FLOW_EVALUATION_H_
#define MAX_FLOW_EVALUATION_H_

#include "GraphAlgo.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
template <class CFG, class WEIGHT, class CAPACITY>
class MaxFlowEvaluation : public MapEvaluationMethod<CFG,WEIGHT> {
 public:
  VID m_source, m_sink;
  double m_flow_size;

  virtual bool evaluate(Roadmap<CFG,WEIGHT>* _rmap) {
    CAPACITY capacity(*(_rmap->m_pRoadmap));
    double flow = max_flow(*(_rmap->m_pRoadmap), m_source, m_sink, capacity);
    return (flow >= m_flow_size);
  }
};

template <class InputIterator, class EqualityComparable>
InputIterator
FindFirst(InputIterator _first, InputIterator _last,
    const EqualityComparable& _value) {
  while((_first != _last) && !(_value == _first->first))
    _first++;
  return _first;
};

template <class GRAPH>
struct ConstantCapacity {
  typedef typename GRAPH::EI EI;
  vector<pair<EI, double> > capacityMap;

  ConstantCapacity(GRAPH& _G) {
    vector<VID> vids;
    _G.GetVerticesVID(vids);
    typename GRAPH::VI v;
    EI e;
    for(typename vector<VID>::const_iterator V = vids.begin(); V != vids.end();
++V) {
      _G.IsVertex(*V, &v);
      for(e = v->edgelist.begin(); e != v->edgelist.end(); ++e)
        capacityMap.push_back(make_pair(e, 1));
    }
  }
  ~ConstantCapacity() {}

  void SetCapacity(EI _e, double _c) {
    if(_c <= 0) {
      typename vector<pair<EI, double> >::iterator CM =
        FindFirst(capacityMap.begin(), capacityMap.end(), _e);
      if(CM != capacityMap.end())
        CM->second = 0;
    }
  }

  double GetCapacity(EI _e) const {
    typename vector<pair<EI, double> >::const_iterator CM =
      FindFirst(capacityMap.begin(), capacityMap.end(), _e);
    if(CM != capacityMap.end())
      return CM->second;
  }
};

template <class GRAPH>
struct InverseWeightCapacity {
  typedef typename GRAPH::EI EI;
  vector<pair<EI, double> > capacityMap;

  InverseWeightCapacity(GRAPH& _G) {
    vector<VID> vids;
    _G.GetVerticesVID(vids);
    typename GRAPH::VI v;
    EI e;
    for(typename vector<VID>::const_iterator V = vids.begin(); V != vids.end();
++V) {
      _G.IsVertex(*V, &v);
      for(e = v->edgelist.begin(); e != v->edgelist.end(); ++e)
        capacityMap.push_back(make_pair(e, 1/e->weight.Weight()));
    }
  }
  ~InverseWeightCapacity() {}

  void SetCapacity(EI _e, double _c) {
    typename vector<pair<EI, double> >::iterator CM =
      FindFirst(capacityMap.begin(), capacityMap.end(), _e);
    if(CM != capacityMap.end())
      if(_c <= 0)
        CM->second = 0;
      else
        CM->second = _c;
  }

  double GetCapacity(EI _e) const {
    typename vector<pair<EI, double> >::const_iterator CM =
      FindFirst(capacityMap.begin(), capacityMap.end(), _e);
    if(CM != capacityMap.end())
      return CM->second;
  }
};

#endif
