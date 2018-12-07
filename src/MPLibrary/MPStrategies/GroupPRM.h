#ifndef PMPL_GROUP_PRM_H_
#define PMPL_GROUP_PRM_H_

#include "GroupStrategyMethod.h"

#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "Utilities/XMLNode.h"

#include <iostream>
#include <string>
#include <utility>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// Basic PRM algorithm for multirobot teams using composite c-space.
///
/// @todo Create option for decoupled planning.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GroupPRM : public GroupStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename GroupRoadmapType::VID       VID;

    ///@}
    ///@name Local Types
    ///@{

    /// Settings for a specific sampler.
    struct SamplerSetting {
      std::string label;   ///< The sampler label.
      size_t count;        ///< The number of samples per call.
      size_t attempts;     ///< The number of attempts per sample.
    };

    ///@}
    ///@name Construction
    ///@{

    GroupPRM();

    GroupPRM(XMLNode& _node);

    virtual ~GroupPRM() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    virtual void Initialize() override;

    ///@}

  protected:

    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Iterate() override;

    ///@}
    ///@name Helpers
    ///@{

    /// Sample and add configurations to the roadmap.
    /// @tparam OutputIterator Output iterator on data structure of VIDs
    /// @param _out Data structure of VIDs of added nodes.
    template <typename OutputIterator>
    void Sample(OutputIterator _out);

    /// Connect nodes and CCs of the roadmap
    /// @tparam InputIterator Iterator on data structure of VIDs/graph nodes
    /// @param _first Begin iterator over VIDs/graph nodes
    /// @param _last End iterator over VIDs/graph nodes
    /// @param _labels Connector labels used in connection
    template <typename InputIterator>
    void Connect(InputIterator _first, InputIterator _last,
        const std::vector<std::string>& _labels);

    ///@}
    ///@name Internal State
    ///@{

    /// Sampler labels with number and attempts of sampler.
    std::vector<SamplerSetting> m_samplers;
    /// Connector labels for node-to-node.
    std::vector<std::string> m_connectorLabels;
    /// Connector labels for cc-to-cc.
    std::vector<std::string> m_componentConnectorLabels;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
GroupPRM<MPTraits>::
GroupPRM() {
  this->SetName("GroupPRM");
}


template <typename MPTraits>
GroupPRM<MPTraits>::
GroupPRM(XMLNode& _node) : GroupStrategyMethod<MPTraits>(_node) {
  this->SetName("GroupPRM");

  for(auto& child : _node) {
    if(child.Name() == "Sampler") {
      SamplerSetting s;
      s.label = child.Read("label", true, "", "Sampler Label");
      s.count = child.Read("number", true,
          1, 0, MAX_INT, "Number of samples");
      s.attempts = child.Read("attempts", false,
          1, 0, MAX_INT, "Number of attempts per sample");
      m_samplers.push_back(s);
    }
    else if(child.Name() == "Connector")
      m_connectorLabels.push_back(
          child.Read("label", true, "", "Connector Label"));
    else if(child.Name() == "ComponentConnector")
      m_componentConnectorLabels.push_back(
          child.Read("label", true, "", "Component Connector Label"));
  }
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
GroupPRM<MPTraits>::
Print(std::ostream& _os) const {
  GroupStrategyMethod<MPTraits>::Print(_os);

  _os << "\tSamplers" << std::endl;
  for(const auto& sampler : m_samplers)
    _os << "\t\t" << sampler.label
        << "\tNumber:"   << sampler.count
        << "\tAttempts:" << sampler.attempts
        << std::endl;

  _os << "\tConnectors" << std::endl;
  for(const auto& label : m_connectorLabels)
    _os << "\t\t" << label << std::endl;

  _os << "\tComponentConnectors" << std::endl;
  for(const auto& label : m_componentConnectorLabels)
    _os << "\t\t" << label << std::endl;
}


template <typename MPTraits>
void
GroupPRM<MPTraits>::
Initialize() {
  // Generate start and goal nodes if possible.
  const VID start = this->GenerateStart(m_samplers.front().label);
  std::vector<VID> goals = this->GenerateGoals(m_samplers.front().label);

  // Try to connect the starts/goals to any existing nodes.
  if(start != INVALID_VID)
    goals.push_back(start);
  Connect(goals.begin(), goals.end(), m_connectorLabels);
}

/*------------------------ MPStrategyMethod Overrides ------------------------*/

template <typename MPTraits>
void
GroupPRM<MPTraits>::
Iterate() {
  // Sample.
  std::vector<VID> vids;
  Sample(std::back_inserter(vids));

  // Connect.
  Connect(vids.begin(), vids.end(), m_connectorLabels);

  // Connect CCs.
  auto g = this->GetGroupRoadmap();
  Connect(g->begin(), g->end(), m_componentConnectorLabels);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
template <typename OutputIterator>
void
GroupPRM<MPTraits>::
Sample(OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(), "GroupPRM::Sample");
  if(this->m_debug)
    std::cout << "Sampling new nodes..." << std::endl;

  auto g = this->GetGroupRoadmap();
  const Boundary* const boundary = this->GetEnvironment()->GetBoundary();

  // Generate nodes with each sampler.
  std::vector<GroupCfgType> samples;
  for(auto& sampler : m_samplers) {
    samples.clear();
    samples.reserve(sampler.count);

    auto s = this->GetSampler(sampler.label);
    s->Sample(sampler.count, sampler.attempts, boundary,
        std::back_inserter(samples));

    if(this->m_debug)
      std::cout << "\tSampler '" << sampler.label << "' generated "
                << samples.size() << "/" << sampler.count << " configurations."
                << std::endl;

    // Add valid samples to roadmap.
    for(auto& sample : samples) {
      const VID vid = g->AddVertex(sample);
      *_out++ = vid;
    }
  }
}


template <typename MPTraits>
template <typename InputIterator>
void
GroupPRM<MPTraits>::
Connect(InputIterator _first, InputIterator _last,
    const std::vector<std::string>& _labels) {
  MethodTimer mt(this->GetStatClass(), "GroupPRM::Connect");

  if(_labels.empty())
    return;

  auto g = this->GetGroupRoadmap();

  if(this->m_debug)
    std::cout << "Connecting..." << std::endl;

  for(const auto& label : _labels) {
    if(this->m_debug)
      std::cout << "\tUsing connector '" << label << "'.\n";

    auto c = this->GetConnector(label);
    c->Connect(g, _first, _last);
  }

  if(this->m_debug) {
    std::cout << "\tGraph has "
              << g->get_num_edges() << " edges and "
              << g->GetNumCCs() << " connected components."
              << std::endl;
  }
}

/*----------------------------------------------------------------------------*/

#endif
