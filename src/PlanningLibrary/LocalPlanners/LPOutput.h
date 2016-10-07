#ifndef LP_OUTPUT_H_
#define LP_OUTPUT_H_

////////////////////////////////////////////////////////////////////////////////
/// @ingroup LocalPlanners
/// @brief Computed information from a local plan
/// @tparam MPTraits Motion planning universe
///
/// This struct stores all information available from local plan computations.
/// Namely, it stores intermediates along edges (not straight line), the path
/// generated, and the edge weights to be added to the RoadmapGraph.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
struct LPOutput {
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPTraits::WeightType WeightType;
  typedef pair<WeightType, WeightType> LPEdge;

  vector<CfgType> m_path;           // Path found by local planner.
  vector<CfgType> m_intermediates;
  LPEdge m_edge;                    // Contains weights of edges defined in path.

  void Clear() {
    m_path.clear();
    m_intermediates.clear();
    m_edge.first.SetWeight(0);
    m_edge.second.SetWeight(0);
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
