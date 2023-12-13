#include "BasicPRM.h"

#include "MPLibrary/MPLibrary.h"

#include <iostream>
#include <string>
#include <utility>
#include <vector>

/*------------------------------- Construction -------------------------------*/

BasicPRM::
BasicPRM() {
  this->SetName("BasicPRM");
}


BasicPRM::
BasicPRM(XMLNode& _node) : MPStrategyMethod(_node) {
  this->SetName("BasicPRM");

  m_inputMapFilename = _node.Read("inputMap", false, "",
      "filename of roadmap to start from");

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

  // Temporary hack for fixing the base.
  m_fixBase = _node.Read("fixBase", false, m_fixBase,
      "Fix the robot's base position and orientation to the start cfg?");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void
BasicPRM::
Print(std::ostream& _os) const {
  MPStrategyMethod::Print(_os);
  _os << "\tInput Map: " << m_inputMapFilename << std::endl;

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

/*------------------------ MPStrategyMethod Overrides ------------------------*/

void
BasicPRM::
Initialize() {
  /// @todo Move input map parsing to the MPSolution somehow?
  //read in and reload roadmap
  if(!m_inputMapFilename.empty()) {
    RoadmapType* r = this->GetRoadmap();
    if(this->m_debug)
      std::cout << "Loading roadmap from \"" << m_inputMapFilename << "\"."
                << std::endl;

    //r->Read(m_inputMapFilename);
    Read(r, m_inputMapFilename);

    auto g = r;
    for(auto vi = g->begin(); vi != g->end(); ++vi)
      VDAddNode(g->GetVertex(vi));
    if(this->m_debug)
      std::cout << "Roadmap has " << g->get_num_vertices()
                << " nodes and " << g->get_num_edges() << " edges.\n"
                << "Resetting map evaluator states."
                << std::endl;
  }

  // Generate start and goal nodes if possible.
  const VID start = this->GenerateStart(m_samplers.front().label);
  std::vector<VID> goals = this->GenerateGoals(m_samplers.front().label);

  // Try to connect the starts/goals to any existing nodes.
  if(start != INVALID_VID)
    goals.push_back(start);
  Connect(goals);

  // Hacks for fixing the base.
  if(m_fixBase) {
    auto g = this->GetRoadmap();
    auto goalTracker = this->GetMPLibrary()->GetGoalTracker();
    const auto& startVIDs = goalTracker->GetStartVIDs();
    if(startVIDs.size() != 1)
      throw RunTimeException(WHERE) << "Fix-base option is only supported for "
                                    << "single start nodes, " << startVIDs.size()
                                    << " were found.";

    const auto& start = g->GetVertex(*startVIDs.begin());

    // Create a version of the robot's cspace where the base
    // position/translation are fixed.
    auto robot = this->GetTask()->GetRobot();
    auto mb = robot->GetMultiBody();
    m_samplingBoundary = robot->GetCSpace()->Clone();
    const size_t numDOF = mb->PosDOF() + mb->OrientationDOF();

    // Need to downcast to c-space bounding box to be able to set the range.
    auto bbx = static_cast<CSpaceBoundingBox*>(m_samplingBoundary.get());
    for(size_t i = 0; i < numDOF; ++i)
      bbx->SetRange(i, start[i], start[i]);
  }
}


void
BasicPRM::
Iterate() {
  // Sample new configurations.
  const std::vector<VID> vids = Sample();

  // If we sampled any valid configurations, try to connect them to the roadmap.
  if(vids.size())
    Connect(vids);
}

/*--------------------------------- Helpers ----------------------------------*/

std::vector<size_t>
BasicPRM::
Sample() {
  if(this->m_debug)
    std::cout << "Sampling new nodes..." << std::endl;

  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::Sample");

  //const Boundary* const boundary = this->GetEnvironment()->GetBoundary();
  auto boundary = m_samplingBoundary.get() ? m_samplingBoundary.get()
                                        : this->GetEnvironment()->GetBoundary();

  // Generate nodes with each sampler.
  auto g = this->GetRoadmap();
  std::vector<VID> out;
  std::vector<Cfg> samples;
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
      out.push_back(g->AddVertex(sample));
  }

  return out;
}


void
BasicPRM::
Connect(const std::vector<VID>& _vids) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::Connect");

  if(this->m_debug)
    std::cout << "Connecting..." << std::endl;

  auto r = this->GetRoadmap();
  for(const auto& label : m_connectorLabels) {
    if(this->m_debug)
      std::cout << "\tUsing connector '" << label << "'." << std::endl;

    this->GetMPLibrary()->GetConnector(label)->Connect(r, _vids.begin(), _vids.end());
  }

  if(this->m_debug)
    std::cout << "\tGraph has "
              << r->get_num_edges() << " edges and "
              << r->GetCCTracker()->GetNumCCs() << " connected components."
              << std::endl;
}

/*----------------------------------------------------------------------------*/
