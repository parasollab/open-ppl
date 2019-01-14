#ifndef PMPL_BASIC_PRM_H_
#define PMPL_BASIC_PRM_H_

#include "MPStrategyMethod.h"

#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "Utilities/XMLNode.h"

#include <iostream>
#include <string>
#include <utility>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// Basic PRM algorithm.
///
/// Reference:
///   Lydia E. Kavraki and Petr Svestka and Jean-Claude Latombe and Mark H.
///   Overmars. "Probabilistic Roadmaps for Path Planning in High-Dimensional
///   Configuration Spaces". TRO 1996.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class BasicPRM : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Local Types
    ///@{

    enum Start {Sampling, Connecting, ConnectingComponents, Evaluating};

    /// Settings for a specific sampler.
    struct SamplerSetting {
      std::string label;
      size_t count;
      size_t attempts;
    };

    ///@}
    ///@name Construction
    ///@{

    BasicPRM();

    BasicPRM(XMLNode& _node);

    virtual ~BasicPRM() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:

    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;
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
    template <class InputIterator>
    void Connect(InputIterator _first, InputIterator _last,
        const std::vector<std::string>& _labels);

    /// Iterate over range and check nodes to be within narrow passage
    /// @tparam InputIterator Iterator on data structure of VIDs
    /// @param _first Begin iterator over VIDs
    /// @param _last End iterator over VIDs
    template <class InputIterator>
    void CheckNarrowPassageSamples(InputIterator _first, InputIterator _last);

    ///@}
    ///@name Internal State
    ///@{

    /// Sampler labels with number and attempts of sampler.
    std::vector<SamplerSetting> m_samplers;
    /// Connector labels for node-to-node.
    std::vector<std::string> m_connectorLabels;
    /// Connector labels for cc-to-cc.
    std::vector<std::string> m_componentConnectorLabels;

    std::string m_inputMapFilename; ///< Input roadmap to initialize map

    /// When inputting a roadmap, specifies where in algorithm to start.
    Start m_startAt{Sampling};

    ///@}
    ///@name Fix-base hacks
    ///@{
    /// To be removed when we properly implement path constraints or perhaps a
    /// sampler-method option.

    bool m_fixBase{false};  ///< Keep the base fixed to the start cfg?
    /// An optional sampling boundary for fixing the base.
    std::unique_ptr<Boundary> m_samplingBoundary;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
BasicPRM<MPTraits>::
BasicPRM() {
  this->SetName("BasicPRM");
}


template <typename MPTraits>
BasicPRM<MPTraits>::
BasicPRM(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("BasicPRM");

  m_inputMapFilename = _node.Read("inputMap", false, "",
      "filename of roadmap to start from");
  std::string startAt = _node.Read("startAt", false, "sampling",
      "point of algorithm where to begin at: \
      \"sampling\" (default), \"connecting\", \
      \"connectingcomponents\", \"evaluating\"");
  if(startAt == "sampling")
    m_startAt = Sampling;
  else if(startAt == "connecting")
    m_startAt = Connecting;
  else if(startAt == "connectingcomponents")
    m_startAt = ConnectingComponents;
  else if(startAt == "evaluating")
    m_startAt = Evaluating;
  else
    throw ParseException(_node.Where()) << "Start at is '" << startAt << "'. "
                                        << "Choices are 'sampling', "
                                        << "'connecting', "
                                        << "'connectingComponents', "
                                        << "'evaluating'.";

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

  // Temporary hack for fixing the base.
  m_fixBase = _node.Read("fixBase", false, m_fixBase,
      "Fix the robot's base position and orientation to the start cfg?");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
BasicPRM<MPTraits>::
Print(std::ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);
  _os << "\tInput Map: " << m_inputMapFilename << std::endl;

  _os << "\tStart At: ";
  switch(m_startAt) {
    case Sampling: _os << "sampling"; break;
    case Connecting: _os << "connecting"; break;
    case ConnectingComponents: _os << "connectingcomponents"; break;
    case Evaluating: _os << "evaluating"; break;
  }
  std::cout << std::endl;

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

/*------------------------ MPStrategyMethod Overrides ------------------------*/

template <typename MPTraits>
void
BasicPRM<MPTraits>::
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

  // Hacks for fixing the base.
  if(m_fixBase) {
    auto g = this->GetRoadmap();
    auto goalTracker = this->GetGoalTracker();
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

  // Generate start and goal nodes if possible.
  const VID start = this->GenerateStart(m_samplers.front().label);
  std::vector<VID> goals = this->GenerateGoals(m_samplers.front().label);

  // Try to connect the starts/goals to any existing nodes.
  if(start != INVALID_VID)
    goals.push_back(start);
  Connect(goals.begin(), goals.end(), m_connectorLabels);
}


template <typename MPTraits>
void
BasicPRM<MPTraits>::
Iterate() {
  std::vector<VID> vids;

  switch(m_startAt) {

    case Sampling:
      Sample(std::back_inserter(vids));

    case Connecting:
      {
        if(m_startAt == Connecting) {
          auto g = this->GetRoadmap();
          Connect(g->begin(), g->end(), m_connectorLabels);
          //For spark prm to grow RRT at difficult nodes
          CheckNarrowPassageSamples(g->begin(), g->end());
        }
        else {
          Connect(vids.begin(), vids.end(), m_connectorLabels);
          //For spark prm to grow RRT at difficult nodes
          CheckNarrowPassageSamples(vids.begin(), vids.end());
        }
      }

    case ConnectingComponents:
      {
        auto g = this->GetRoadmap();
        Connect(g->begin(), g->end(), m_componentConnectorLabels);
      }

    default:
      break;
  }
  m_startAt = Sampling;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
template<typename OutputIterator>
void
BasicPRM<MPTraits>::
Sample(OutputIterator _out) {
  if(this->m_debug)
    std::cout << "Sampling new nodes..." << std::endl;

  MethodTimer mt(this->GetStatClass(), "BasicPRM::Sample");
  auto g = this->GetRoadmap();
  //const Boundary* const boundary = this->GetEnvironment()->GetBoundary();
  auto boundary = m_samplingBoundary.get() ? m_samplingBoundary.get()
                                        : this->GetEnvironment()->GetBoundary();

  // Generate nodes with each sampler.
  std::vector<CfgType> samples;
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
template<class InputIterator>
void
BasicPRM<MPTraits>::
Connect(InputIterator _first, InputIterator _last,
    const std::vector<std::string>& _labels) {
  MethodTimer mt(this->GetStatClass(), "BasicPRM::Connect");

  if(this->m_debug)
    std::cout << "Connecting..." << std::endl;

  for(const auto& label : _labels) {
    if(this->m_debug)
      std::cout << "\tUsing connector '" << label << "'.\n";

    auto c = this->GetConnector(label);
    c->Connect(this->GetRoadmap(), _first, _last);
  }

  if(this->m_debug) {
    auto g = this->GetRoadmap();
    std::cout << "\tGraph has "
              << g->get_num_edges() << " edges and "
              << g->GetNumCCs() << " connected components."
              << std::endl;
  }
}


template <typename MPTraits>
template<class InputIterator>
void
BasicPRM<MPTraits>::
CheckNarrowPassageSamples(InputIterator _first, InputIterator _last) {
  MethodTimer mt(this->GetStatClass(), "BasicPRM::CheckNarrowPassageSamples");

  /// @todo This adds O(n) to each iteration regardless of whether it is used,
  ///       try to rework the design.
  if(this->m_debug)
    std::cout << this->GetName() << "::CheckNarrowPassageSamples"
              << std::endl;

  for(; _first != _last; _first++) {
    const VID vid = this->GetRoadmap()->GetVID(_first);
    if(this->CheckNarrowPassageSample(vid))
      break;
  }
}

/*----------------------------------------------------------------------------*/

#endif
