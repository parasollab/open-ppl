#ifndef LPOUTPUT_H_
#define LPOUTPUT_H_

template<class MPTraits>
struct LPOutput {
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPTraits::WeightType WeightType;
  typedef pair<WeightType, WeightType> LPEdge;
  typedef pair<pair<CfgType, CfgType>, LPEdge> LPSavedEdge;

  vector<CfgType> m_path;           // Path found by local planner.
  vector<CfgType> m_intermediates;
  LPEdge m_edge;                    // Contains weights of edges defined in path.
  vector<LPSavedEdge> m_savedEdge;  // Failed Edge: savedEdge.second -> position
                                  //   failed.

  void Clear() {
    m_path.clear();
    m_intermediates.clear();
    m_edge.first.SetWeight(0);
    m_edge.second.SetWeight(0);
    m_savedEdge.clear();
  }

  void SetLPLabel(string _l) {
    m_edge.first.SetLPLabel(_l);
    m_edge.second.SetLPLabel(_l);
  }

  void AddIntermediatesToWeights(bool _saveIntermediates) {
    if(_saveIntermediates) {
      m_edge.first.SetIntermediates(m_intermediates);
      vector<CfgType> tmp = m_intermediates;
      reverse(tmp.begin(), tmp.end());
      m_edge.second.SetIntermediates(tmp);
    }
  }
};

#endif
