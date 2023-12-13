#include "GroupPRM.h"

#include "MPLibrary/MPLibrary.h"

#include <iostream>
#include <string>
#include <utility>
#include <vector>

/*------------------------------- Construction -------------------------------*/

GroupPRM::
GroupPRM() {
  this->SetName("GroupPRM");
}


GroupPRM::
GroupPRM(XMLNode& _node) : GroupStrategyMethod(_node) {
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

void
GroupPRM::
Print(std::ostream& _os) const {
  GroupStrategyMethod::Print(_os);

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


void
GroupPRM::
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

void
GroupPRM::
Iterate() {
  // Sample new configurations.
  const std::vector<VID> vids = Sample();

  // If we sampled any valid configurations, try to connect them to the roadmap.
  if(vids.size())
    Connect(vids);
}

/*--------------------------------- Helpers ----------------------------------*/

std::vector<size_t>
GroupPRM::
Sample() {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::Sample");
  if(this->m_debug)
    std::cout << "Sampling new nodes..." << std::endl;

  auto r = this->GetGroupRoadmap();
  const Boundary* const boundary = this->GetEnvironment()->GetBoundary();

  // Generate nodes with each sampler.
  std::vector<VID> out;
  std::vector<GroupCfgType> samples;
  for(auto& sampler : m_samplers) {
    samples.clear();
    samples.reserve(sampler.count);

    auto s = this->GetMPLibrary()->GetSampler(sampler.label);
    s->Sample(sampler.count, sampler.attempts, boundary,
        std::back_inserter(samples));

    if(this->m_debug)
      std::cout << "\tSampler '" << sampler.label << "' generated "
                << samples.size() << "/" << sampler.count << " configurations."
                << std::endl;

    // Add valid samples to roadmap.
    out.reserve(out.size() + samples.size());
    for(auto& sample : samples)
      out.push_back(r->AddVertex(sample));
  }

  return out;
}


void
GroupPRM::
Connect(const std::vector<VID>& _vids) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::Connect");

  if(this->m_debug)
    std::cout << "Connecting..." << std::endl;

  auto r = this->GetGroupRoadmap();
  for(const auto& label : m_connectorLabels) {
    if(this->m_debug)
      std::cout << "\tUsing connector '" << label << "'." << std::endl;

    this->GetMPLibrary()->GetConnector(label)->Connect(r, _vids.begin(), _vids.end());
  }

  if(this->m_debug)
    std::cout << "\tGraph has "
              << r->get_num_edges() << " edges and "
              //<< r->GetCCTracker()->GetNumCCs()
              << "?" /// @todo Setup CC tracker for groups to fix this.
              << " connected components."
              << std::endl;
}

/*----------------------------------------------------------------------------*/
