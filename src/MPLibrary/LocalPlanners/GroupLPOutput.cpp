#include "GroupLPOutput.h"

#include <string>
#include <utility>
#include <vector>

/*------------------------------- Construction -------------------------------*/

GroupLPOutput::
GroupLPOutput(GroupRoadmapType* const _map, GroupCfgPath _path,
    GroupCfgPath _intermediates)
    : m_groupRoadmap(_map), m_path(_path), m_intermediates(_intermediates),
      m_edge(GroupWeightType(_map), GroupWeightType(_map))
{ }


GroupLPOutput::
GroupLPOutput(GroupRoadmapType* const _map, const GroupWeightType& _edge) :
              m_groupRoadmap(_map), m_edge(_edge, _edge) {
  // Reverse the second edge member's intermediates:
  GroupCfgPath& reversePath = m_edge.second.GetIntermediates();
  std::reverse(reversePath.begin(), reversePath.end());
}


void
GroupLPOutput::
Clear() {
  m_path.clear();
  m_intermediates.clear();
  m_edge.first.Clear();
  m_edge.second.Clear();
}


void
GroupLPOutput::
SetLPLabel(const std::string& _label) {
  m_edge.first.SetLPLabel(_label);
  m_edge.second.SetLPLabel(_label);
}


void
GroupLPOutput::
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


void
GroupLPOutput::
SetFormation(const std::vector<size_t>& _formation) {
  m_edge.first.SetFormation(_formation);
  m_edge.second.SetFormation(_formation);
}


void
GroupLPOutput::
SetIndividualEdges(const std::vector<size_t>& _formation) {
  /// @todo We need to preserve the intermediates.
  /// @todo This is not a correct edge for each individual robot - they will not
  ///       all have the same weight. This needs to be tracked separately.
  if(!m_edge.first.GetIntermediates().empty())
    std::cerr << "GroupLPOutput Warning: intermediates detected in group edge "
              << "are not being added in the individual edge yet!"
              << std::endl;

  const std::string label = m_edge.first.GetLPLabel();
  const double weight = m_edge.first.GetWeight();

  // If there are no robots in the formation, then we need to set the individual
  // edges for all of them.
  if(_formation.empty()) {
    const size_t numRobots = m_groupRoadmap->GetGroup()->Size();
    for(size_t i = 0; i < numRobots; ++i) {
      m_edge.first.SetEdge(i, IndividualEdge(label, weight));
      m_edge.second.SetEdge(i, IndividualEdge(label, weight));
    }
  }
  else for(const size_t robotIndex : _formation) {
    m_edge.first.SetEdge(robotIndex, IndividualEdge(label, weight));
    m_edge.second.SetEdge(robotIndex, IndividualEdge(label, weight));
  }
}


// void
// GroupLPOutput::
// SetSkipEdge() {
//   m_edge.first.SetSkipEdge();
//   m_edge.second.SetSkipEdge();
// }


void
GroupLPOutput::
SetEdgeWeights(const double _weight) {
  m_edge.first.SetWeight(_weight);
  m_edge.second.SetWeight(_weight);
}
