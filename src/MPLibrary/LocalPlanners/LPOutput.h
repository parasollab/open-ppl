#ifndef PMPL_LP_OUTPUT_H_
#define PMPL_LP_OUTPUT_H_

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"

#include <string>
#include <utility>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
/// Computed information from a local plan.
///
/// Stores all information available from local plan computations, including
/// intermediates along edges (not straight line), the path
/// generated, and the edge weights to be added to the RoadmapGraph.
///
/// @todo Destroy this object and have LPs/Extenders work directly with a
///       LocalPlan object, which should replace the DefaultWeight class as our
///       roadmap edge.
///
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
struct LPOutput {

  ///@name Motion Planning Types
  ///@{

  typedef DefaultWeight<Cfg> WeightType;

  ///@}
  ///@name Local Types
  ///@{

  typedef std::pair<WeightType, WeightType> LPEdge;

  ///@}
  ///@name Internal State
  ///@{

  /// The resolution-level path computed by a local planner. Does not include
  /// the start or goal configurations.
  std::vector<Cfg> m_path;

  /// The set of 'intermediate' configurations, between each of which is a
  /// straight-line path in c-space (i.e. these are the vertices of a polygonal
  /// chain). Does not include the start or goal configurations.
  std::vector<Cfg> m_intermediates;

  /// A pair of weight objects. The first is in the forward direction and the
  /// second is its reverse.
  LPEdge m_edge;

  ///@}
  ///@name Construction
  ///@{

  LPOutput();

  ///@}
  ///@name Interface
  ///@{

  void Clear();

  void SetLPLabel(const std::string& _label);

  void AddIntermediatesToWeights(const bool _saveIntermediates);

  ///@}

};

#endif
