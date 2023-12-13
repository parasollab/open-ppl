#include "TogglePRMStrategy.h"

#include "MPLibrary/MPLibrary.h"
#include "Utilities/MetricUtils.h"

#include <deque>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

/*------------------------------ Construction --------------------------------*/

TogglePRMStrategy::
TogglePRMStrategy() {
  this->SetName("TogglePRMStrategy");
}


TogglePRMStrategy::
TogglePRMStrategy(XMLNode& _node) : BasicPRM(_node) {
  this->SetName("TogglePRMStrategy");

  m_vcLabel = _node.Read("vcLabel", true, "",
      "Validity checker for lazy samples.");
  m_priority = _node.Read("priority", false, m_priority,
      "Give priority to valid nodes in the toggle queue?");

  for(auto& child : _node) {
    if(child.Name() == "Sampler") {
      SamplerSetting s;
      s.label = child.Read("label", true, "", "Sampler Label");
      s.count = child.Read("number", true,
          1, 0, MAX_INT, "Number of samples");
      s.attempts = child.Read("attempts", false,
          1, 0, MAX_INT, "Number of attempts per sample");
      this->m_samplers.push_back(s);
    }
    else if(child.Name() == "Connector")
      this->m_connectorLabels.push_back(
          child.Read("label", true, "", "Connector Label"));
    else if(child.Name() == "ColConnector")
        m_colConnectorLabels.push_back(
          child.Read("label", true, "", "Node connection method"));
  }
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

void
TogglePRMStrategy::
Print(std::ostream& _os) const {
  MPStrategyMethod::Print(_os);

  _os << "\tSamplers";
  for(const auto& sampler : this->m_samplers)
    _os << "\n\t\t" << sampler.label
        << "\tNumber:"   << sampler.count
        << "\tAttempts:" << sampler.attempts;

  _os << "\n\tConnectors";
  for(const auto& label : this->m_connectorLabels)
    _os << "\n\t\t" << label;

  _os << "\n\tCollision Connectors";
  for(const auto& label : m_colConnectorLabels)
    _os << "\n\t\t" << label;

  _os << "\n\tvcLabel: " << m_vcLabel
      << "\n\tpriority: " << m_priority
      << std::endl;
}

/*----------------------- MPStrategyMethod Overrides -------------------------*/

void
TogglePRMStrategy::
Initialize() {
  BasicPRM::Initialize();

  m_queue.clear();
}


void
TogglePRMStrategy::
Iterate() {
  // If the toggle queue is empty, generate more nodes.
  if(m_queue.empty())
    GenerateNodes();

  // Extract the next node from the toggle queue.
  std::pair<bool, Cfg> p = m_queue.front();
  m_queue.pop_front();
  const bool valid = p.first;
  const Cfg& cfg = p.second;

  // If the next node is valid, add to free roadmap.
  if(valid) {
    const VID vid = this->GetRoadmap()->AddVertex(cfg);

    ConnectHelper(true, vid);
  }
  // If the next node is invalid, add to obstacle roadmap. Toggle validity
  // while connecting.
  else {
    const VID vid = this->GetBlockRoadmap()->AddVertex(cfg);

    this->GetMPLibrary()->ToggleValidity();
    ConnectHelper(false, vid);
    this->GetMPLibrary()->ToggleValidity();
  }
}

/*--------------------------------- Helpers ----------------------------------*/

void
TogglePRMStrategy::
GenerateNodes() {
  const std::string callee = "TogglePRMStrategy::GenerateNodes";
  MethodTimer mt(this->GetStatClass(), callee);

  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);

  // Use each sampler to generate nodes.
  std::vector<Cfg> outNodes;
  for(auto& sampler : this->m_samplers) {
    outNodes.clear();

    // Generate nodes for this sampler
    auto s = this->GetMPLibrary()->GetSampler(sampler.label);
    s->Sample(sampler.count, sampler.attempts,
        this->GetEnvironment()->GetBoundary(),
        std::back_inserter(outNodes), std::back_inserter(outNodes));

    // Add nodes to queue
    for(auto& sample : outNodes) {
      // If not validated yet, determine validity
      if(!sample.IsLabel("VALID"))
        vc->IsValid(sample, callee);

      Enqueue(sample);
    }
  }
}


void
TogglePRMStrategy::
ConnectHelper(const bool _valid, const VID _vid) {
  MethodTimer mt(this->GetStatClass(), "TogglePRMStrategy::ConnectHelper");

  // Grab correct set of connectors depending on vertex validity
  const auto& connectorLabels = _valid ? this->m_connectorLabels
                                       : m_colConnectorLabels;

  // Loop through each connector
  std::vector<Cfg> collision;
  auto iter = std::back_inserter(collision);
  for(const auto& label : connectorLabels) {
    collision.clear();

    // Try to connect _vid using this connector.
    auto connector = this->GetMPLibrary()->GetConnector(label);
    connector->Connect(
        _valid ? this->GetRoadmap() : this->GetBlockRoadmap(),
        _vid, nullptr, &iter);

    // Add collision witnesses to queue.
    for(const auto& cfg : collision)
      Enqueue(cfg);
  }
}


void
TogglePRMStrategy::
Enqueue(const Cfg& _cfg) {
  // Ensure this configuration was validated.
  if(!_cfg.IsLabel("VALID"))
    throw RunTimeException(WHERE) << "Collision not yet validated, "
                                  << "shouldn't happen.";

  // Note: the 'VALID' label doesn't change when the validity is toggled.
  // Valid always means collision-free here regardless of toggling.
  if(!_cfg.GetLabel("VALID"))
    m_queue.emplace_back(false, _cfg);
  else if(m_priority)
    m_queue.emplace_front(true, _cfg);
  else
    m_queue.emplace_back(true, _cfg);
}

/*----------------------------------------------------------------------------*/
