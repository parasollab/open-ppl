#ifndef LP_OUTPUT_H_
#define LP_OUTPUT_H_

#include <string>
#include <utility>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
/// Computed information from a local plan.
///
/// Stores all information available from local plan computations, including
/// intermediates along edges (not straight line), the path
/// generated, and the edge weights to be added to the RoadmapGraph.
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
struct LPOutput {

  ///@name Motion Planning Types
  ///@{

  typedef typename MPTraits::CfgType    CfgType;
  typedef typename MPTraits::WeightType WeightType;

  ///@}
  ///@name Local Types
  ///@{

  typedef std::pair<WeightType, WeightType> LPEdge;

  ///@}
  ///@name Internal State
  ///@{

  std::vector<CfgType> m_path;           // Path found by local planner.
  std::vector<CfgType> m_intermediates;

  LPEdge m_edge;                    // Contains weights of edges defined in path.

  ///@}
  ///@name Interface
  ///@{

  void Clear();

  void SetLPLabel(const std::string& _label);

  void AddIntermediatesToWeights(const bool _saveIntermediates);

  ///@}

};

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
void
LPOutput<MPTraits>::
Clear() {
  m_path.clear();
  m_intermediates.clear();
  m_edge.first.SetWeight(0);
  m_edge.second.SetWeight(0);
}


template <typename MPTraits>
void
LPOutput<MPTraits>::
SetLPLabel(const std::string& _label) {
  m_edge.first.SetLPLabel(_label);
  m_edge.second.SetLPLabel(_label);
}


template <typename MPTraits>
void
LPOutput<MPTraits>::
AddIntermediatesToWeights(const bool _saveIntermediates) {
  if(!_saveIntermediates)
    return;

  // Make a copy of the intermediates in reverse order for the backward edge.
  std::vector<CfgType> tmp;
  tmp.reserve(m_intermediates.size());
  std::copy(m_intermediates.rbegin(), m_intermediates.rend(),
            std::back_inserter(tmp));

  // Set both edges.
  m_edge.first.SetIntermediates(m_intermediates);
  m_edge.second.SetIntermediates(tmp);
}

/*----------------------------------------------------------------------------*/

#endif
