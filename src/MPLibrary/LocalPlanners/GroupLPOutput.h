#ifndef GROUP_LP_OUTPUT_H_
#define GROUP_LP_OUTPUT_H_

#include <string>
#include <utility>
#include <vector>

#include "MPProblem/RobotGroup/RobotGroup.h"

////////////////////////////////////////////////////////////////////////////////
/// Computed information from a local plan.
///
/// Stores all information available from local plan computations, including
/// intermediates along edges (not straight line), the path
/// generated, and the edge weights to be added to the RoadmapGraph.
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
struct GroupLPOutput {

  ///@name Motion Planning Types
  ///@{

  typedef typename MPTraits::WeightType       IndividualEdge;

  typedef typename MPTraits::GroupCfgType     GroupCfgType;
  typedef typename MPTraits::GroupWeightType  GroupWeightType;
  typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
  typedef std::vector<GroupCfgType>           GroupCfgPath;

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

  void SetActiveRobots(const std::vector<size_t>& _activeRobots);

  void SetIndividualEdges(const std::vector<size_t>& _activeRobots);

  std::vector<size_t> GetActiveRobots() {return m_edge.first.GetActiveRobots();}

  void SetSkipEdge();

  void SetEdgeWeights(const double _weight);

  ///@}

};

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
GroupLPOutput<MPTraits>::
GroupLPOutput(GroupRoadmapType* const _map, GroupCfgPath _path,
              GroupCfgPath _intermediates) :
      m_groupRoadmap(_map), m_path(_path), m_intermediates(_intermediates),
      m_edge(GroupWeightType(_map), GroupWeightType(_map))
      { }


template <typename MPTraits>
GroupLPOutput<MPTraits>::
GroupLPOutput(GroupRoadmapType* const _map, const GroupWeightType& _edge) :
              m_groupRoadmap(_map), m_edge(_edge, _edge) {
  // Reverse the second edge member's intermediates:
  GroupCfgPath& reversePath = m_edge.second.GetIntermediates();
  std::reverse(reversePath.begin(), reversePath.end());
}


template <typename MPTraits>
void
GroupLPOutput<MPTraits>::
Clear() {
  m_path.clear();
  m_intermediates.clear();
  m_edge.first.SetWeight(0);
  m_edge.second.SetWeight(0);
}


template <typename MPTraits>
void
GroupLPOutput<MPTraits>::
SetLPLabel(const std::string& _label) {
  m_edge.first.SetLPLabel(_label);
  m_edge.second.SetLPLabel(_label);
}


template <typename MPTraits>
void
GroupLPOutput<MPTraits>::
AddIntermediatesToWeights(const bool _saveIntermediates) {
  if(!_saveIntermediates)
    return;

  // Make a copy of the intermediates in reverse order for the backward edge.
  GroupCfgPath tmp;
  tmp.reserve(m_intermediates.size());
  std::copy(m_intermediates.rbegin(), m_intermediates.rend(),
            std::back_inserter(tmp));

  // Set both edges.
  m_edge.first.SetIntermediates(m_intermediates);
  m_edge.second.SetIntermediates(tmp);
}

template <typename MPTraits>
void
GroupLPOutput<MPTraits>::
SetActiveRobots(const std::vector<size_t>& _activeRobots) {
  m_edge.first.SetActiveRobots(_activeRobots);
  m_edge.second.SetActiveRobots(_activeRobots);
}


template <typename MPTraits>
void
GroupLPOutput<MPTraits>::
SetIndividualEdges(const std::vector<size_t>& _activeRobots) {
  //NOTE: Only one direction of individual edges are being supported for now!

  if(!m_edge.first.GetIntermediates().empty())
    std::cerr << "GroupLPOutput Warning: intermediates detected in group edge "
                 "are not being added in the individual edge yet!" << std::endl;

  const std::string label = m_edge.first.GetLPLabel();
  const double weight = m_edge.first.GetWeight();

  for(const size_t robotIndex : _activeRobots) {
    // TODO: Add in support for extracting the individual cfgs from path (make
    //       a Roadmap function to do so for a vector of any group cfgs), which
    //       should be performed in here for each robot.
    m_edge.first.SetEdge(robotIndex, IndividualEdge(label, weight));
    m_edge.second.SetEdge(robotIndex, IndividualEdge(label, weight));
  }
}


template <typename MPTraits>
void
GroupLPOutput<MPTraits>::
SetSkipEdge() {
  m_edge.first.SetSkipEdge();
  m_edge.second.SetSkipEdge();
}


template <typename MPTraits>
void
GroupLPOutput<MPTraits>::
SetEdgeWeights(const double _weight) {
  m_edge.first.SetWeight(_weight);
  m_edge.second.SetWeight(_weight);
}


/*----------------------------------------------------------------------------*/

#endif
