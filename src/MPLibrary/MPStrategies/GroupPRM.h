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

    virtual void SetLocalBoundaries(std::map<Robot*,Boundary*> _boundaries) override;

    ///@}

  protected:

    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Iterate() override;

    virtual void Finalize() override;

    ///@}
    ///@name Helpers
    ///@{

    /// Sample and add configurations to the roadmap.
    /// @return The generated VIDs for the successful samples.
    std::vector<VID> Sample();

    /// Connect nodes in the roadmap.
    /// @param _vids A set of node VIDs to connect to the rest of the roadmap.
    void Connect(const std::vector<VID>& _vids);

    ///@}
    ///@name Internal State
    ///@{

    /// Sampler labels with number and attempts of sampler.
    std::vector<SamplerSetting> m_samplers;
    /// Connector labels for node-to-node.
    std::vector<std::string> m_connectorLabels;

    std::map<Robot*,Boundary*> m_localBoundaries;

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

  Connect(goals);
}

/*------------------------ MPStrategyMethod Overrides ------------------------*/

template <typename MPTraits>
void
GroupPRM<MPTraits>::
Iterate() {
  // Sample new configurations.
  const std::vector<VID> vids = Sample();

  // If we sampled any valid configurations, try to connect them to the roadmap.
  if(vids.size())
    Connect(vids);
}

template <typename MPTraits>
void
GroupPRM<MPTraits>::
Finalize() {

  if(!this->m_writeOutput)
    return;

  //TODO: This is output info is temporaly used here for testing the individual
  //    paths extraction from group paths, we may need to move this to the
  //    GroupStrategyMethod class.
  //const std::string base = this->GetBaseFilename();
  const std::string base = this->GetBaseFilename() + "." +
    this->GetNameAndLabel();
  auto path = this->GetGroupPath();
  auto roadmap = this->GetGroupRoadmap();
  roadmap->Write(base + ".map", this->GetEnvironment());
  auto robotGroup = path->GetRoadmap()->GetGroup();
  auto robots = robotGroup->GetRobots();
  for(auto robot : robots) {
    auto label = robot->GetLabel();
    auto robotIndex = robotGroup->GetGroupIndex(robot);
    auto individualRoadmap = roadmap->GetIndividualGraph(robotIndex);
    individualRoadmap->Write(base + "."+ label +".map", this->GetEnvironment());
    auto individualPath = path->GetIndividualPath(robot);
    if(individualPath.Size()) {
      if(this->m_writeOutput)
        ::WritePath(base + "." + label + ".fromGroupPRM.path", individualPath.FullCfgs(this->GetMPLibrary()));
    }
  }
  // Output path vertices. Write both full and roadmap paths for now.
  if(path and path->Size()) {
    //::WritePath(base + ".rdmp.path", path->Cfgs());
    if(this->m_writeOutput)
      ::WritePath(base + ".path", path->FullCfgs(this->GetMPLibrary()));
  }

  // this->ModifyPath();
  // this->GetStatClass()->SetStat("GroupCompositeQuery::TotalCost",path->Length());
  const double timeRes = this->GetEnvironment()->GetTimeRes();
  this->GetStatClass()->SetStat(this->GetNameAndLabel() + "::TotalCost",path->TimeSteps() * timeRes * robots.size());
  this->GetStatClass()->SetStat(this->GetNameAndLabel() + "::NumEdges",roadmap->get_num_edges());
  this->GetStatClass()->SetStat(this->GetNameAndLabel() + "::NumVertices",roadmap->get_num_vertices());

  // Output stats.
  std::ofstream osStat(base + ".stat");
  this->GetStatClass()->PrintAllStats(osStat, roadmap);

}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
std::vector<typename MPTraits::GroupRoadmapType::VID>
GroupPRM<MPTraits>::
Sample() {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::Sample");
  if(this->m_debug)
    std::cout << "Sampling new nodes..." << std::endl;

  auto r = this->GetGroupRoadmap();
  //const Boundary* const boundary = this->GetEnvironment()->GetBoundary();

  // Generate nodes with each sampler.
  std::vector<VID> out;
  std::vector<GroupCfgType> validSamples;
  std::vector<GroupCfgType> invalidSamples;
  for(auto& sampler : m_samplers) {
    validSamples.clear();
    validSamples.reserve(sampler.count);

    invalidSamples.clear();
    invalidSamples.reserve(sampler.count);

    auto s = this->GetSampler(sampler.label);
    s->Sample(sampler.count, sampler.attempts, m_localBoundaries,
        std::back_inserter(validSamples), std::back_inserter(invalidSamples));

    this->GetStatClass()->SetStat("Free samples", validSamples.size());
    this->GetStatClass()->SetStat("Obstacle samples", invalidSamples.size());

    if(this->m_debug)
      std::cout << "\tSampler '" << sampler.label << "' generated "
                << validSamples.size() << " valid configurations and "
                << invalidSamples.size() << " invalid configurations."
                << std::endl;

    // Add valid samples to roadmap.
    out.reserve(out.size() + validSamples.size());
    for(auto& sample : validSamples)
      out.push_back(r->AddVertex(sample));
  }

  return out;
}


template <typename MPTraits>
void
GroupPRM<MPTraits>::
Connect(const std::vector<VID>& _vids) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::Connect");

  if(this->m_debug)
    std::cout << "Connecting..." << std::endl;

  auto r = this->GetGroupRoadmap();
  for(const auto& label : m_connectorLabels) {
    if(this->m_debug)
      std::cout << "\tUsing connector '" << label << "'." << std::endl;

    this->GetConnector(label)->Connect(r, _vids.begin(), _vids.end());
  }

  if(this->m_debug)
    std::cout << "\tGraph has "
              << r->get_num_vertices() << " vertices, "
              << r->get_num_edges() << " edges and "
              << r->GetCCTracker()->GetNumCCs()
              //<< "?" /// @todo Setup CC tracker for groups to fix this.
              << " connected components."
              << std::endl;
}


template<typename MPTraits>
void
GroupPRM<MPTraits>::
SetLocalBoundaries(std::map<Robot*,Boundary*> _boundaries) {
  MPStrategyMethod<MPTraits>::SetLocalBoundaries(_boundaries);
  m_localBoundaries = _boundaries;
}

/*----------------------------------------------------------------------------*/

#endif