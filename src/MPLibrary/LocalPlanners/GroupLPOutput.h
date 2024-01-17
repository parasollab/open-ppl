#ifndef PMPL_GROUP_LP_OUTPUT_H_
#define PMPL_GROUP_LP_OUTPUT_H_

#include <string>
#include <utility>
#include <vector>

#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "MPProblem/RobotGroup/RobotGroup.h"


////////////////////////////////////////////////////////////////////////////////
/// Computed information from a local plan.
///
/// Stores all information available from local plan computations, including
/// intermediates along edges (not straight line), the path
/// generated, and the edge weights to be added to the RoadmapGraph.
///
/// @todo Destroy this object and have LPs/Extenders work directly with a
///       GroupLocalPlan.
///
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
struct GroupLPOutput {

  ///@name Motion Planning Types
  ///@{

  typedef DefaultWeight<Cfg> IndividualEdge;
  typedef GenericStateGraph<Cfg, IndividualEdge> RoadmapType;
  typedef GroupCfg<RoadmapType> GroupCfgType;
  typedef GroupLocalPlan<RoadmapType> GroupWeightType;
  typedef GroupRoadmap<GroupCfgType, GroupWeightType> GroupRoadmapType;
  typedef std::vector<GroupCfgType> GroupCfgPath;

  ///@}
  ///@name Local Types
  ///@{

  typedef std::pair<GroupWeightType, GroupWeightType> LPEdge;

  ///@}
  ///@name Internal State
  ///@{

  GroupRoadmapType* m_groupRoadmap{nullptr}; // Group this local plan is for.

  GroupCfgPath m_path;           // Path found by local planner.
  GroupCfgPath m_intermediates;

  LPEdge m_edge;                   // Contains weights of edges defined in path.

  ///@}
  ///@name Construction
  ///@{

  GroupLPOutput(GroupRoadmapType* const _map = nullptr,
      GroupCfgPath _path = GroupCfgPath(),
      GroupCfgPath _intermediates = GroupCfgPath());

  // Copies _edge in to both members of the edge pair, then reverses the
  // m_intermediates vector of the second element.
  GroupLPOutput(GroupRoadmapType* const _map, const GroupWeightType& _edge);

  ///@}
  ///@name Interface
  ///@{

  void Clear();

  void SetLPLabel(const std::string& _label);

  void AddIntermediatesToWeights(const bool _saveIntermediates);

  void SetFormation(const std::vector<size_t>& _formation);

  void SetIndividualEdges(const std::vector<size_t>& _formation);

  std::vector<size_t> GetFormation() {return m_edge.first.GetFormation();}

  // void SetSkipEdge();

  void SetEdgeWeights(const double _weight);

  ///@}

};

#endif
