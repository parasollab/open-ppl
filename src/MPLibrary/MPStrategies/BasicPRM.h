#ifndef BASIC_PRM_H_
#define BASIC_PRM_H_

#include "MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Basic PRM approach
///
/// BasicPRM essentially combines samplers and connectors to iteratively
/// construct a roadmap until planning is "done"
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class BasicPRM : public MPStrategyMethod<MPTraits> {
  public:

    enum Start {Sampling, Connecting, ConnectingComponents, Evaluating};

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename RoadmapType::VID       VID;

    BasicPRM(
        const map<string, pair<size_t, size_t> >& _samplerLabels = map<string, pair<size_t, size_t> >(),
        const vector<string>& _connectorLabels = vector<string>(),
        const vector<string>& _componentConnectorLabels = vector<string>(),
        const vector<string>& _evaluatorLabels = vector<string>(),
        string _inputMapFilename = "",
        Start _startAt = Sampling);
    BasicPRM(XMLNode& _node);
    virtual ~BasicPRM() {}

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    virtual void Initialize();
    virtual void Iterate();
    virtual void Finalize();

  protected:
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Sample and add configurations to the roadmap.
    /// @tparam OutputIterator Output iterator on data structure of VIDs
    /// @param[out] _thisIterationOut Data structure of VIDs of added nodes.
    template <typename OutputIterator>
      void Sample(OutputIterator _thisIterationOut);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Connect nodes and CCs of the roadmap
    /// @tparam InputIterator Iterator on data structure of VIDs/graph nodes
    /// @param _first Begin iterator over VIDs/graph nodes
    /// @param _last End iterator over VIDs/graph nodes
    /// @param _labels Connector labels used in connection
    template<class InputIterator>
      void Connect(InputIterator _first, InputIterator _last,
          const vector<string>& _labels);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Iterate over range and check nodes to be within narrow passage
    /// @tparam InputIterator Iterator on data structure of VIDs
    /// @param _first Begin iterator over VIDs
    /// @param _last End iterator over VIDs
    template<class InputIterator>
      void CheckNarrowPassageSamples(InputIterator _first, InputIterator _last);

    map<string, pair<size_t, size_t> > m_samplerLabels; ///< Sampler labels with number and attempts of sampler
    vector<string> m_connectorLabels; ///< Connector labels for node-to-node
    vector<string> m_componentConnectorLabels; ///< Connector labels for cc-to-cc
    size_t m_currentIteration; ///< Current iteration of while-loop of Run function
    string m_inputMapFilename; ///< Input roadmap to initialize map
    Start m_startAt; ///< When inputting a roadmap, specifies where in algorithm to start
};

template <typename MPTraits>
BasicPRM<MPTraits>::
BasicPRM(const map<string, pair<size_t, size_t> >& _samplerLabels,
    const vector<string>& _connectorLabels,
    const vector<string>& _componentConnectorLabels,
    const vector<string>& _evaluatorLabels,
    string _inputMapFilename, Start _startAt) :
    m_samplerLabels(_samplerLabels), m_connectorLabels(_connectorLabels),
    m_componentConnectorLabels(_componentConnectorLabels),
    m_currentIteration(0), m_inputMapFilename(_inputMapFilename),
    m_startAt(_startAt) {
  this->m_meLabels = _evaluatorLabels;
  this->SetName("BasicPRM");
}

template <typename MPTraits>
BasicPRM<MPTraits>::
BasicPRM(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node),
    m_currentIteration(0), m_inputMapFilename(""), m_startAt(Sampling) {
  this->SetName("BasicPRM");
  ParseXML(_node);
}

template <typename MPTraits>
void
BasicPRM<MPTraits>::
ParseXML(XMLNode& _node) {
  m_inputMapFilename = _node.Read("inputMap", false, "",
      "filename of roadmap to start from");
  string startAt = _node.Read("startAt", false, "sampling",
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
  else  {
    string message = "Start at is '" + startAt +
      "'. Choices are 'sampling', 'connecting', 'connectingComponents', 'evaluating'.";
    throw ParseException(_node.Where(), message);
  }

  for(auto& child : _node) {
    if(child.Name() == "Sampler") {
      string s = child.Read("method", true, "", "Sampler Label");
      size_t num = child.Read("number", true,
          1, 0, MAX_INT, "Number of samples");
      size_t attempts = child.Read("attempts", false,
          1, 0, MAX_INT, "Number of attempts per sample");
      m_samplerLabels[s] = make_pair(num, attempts);
    }
    else if(child.Name() == "Connector")
      m_connectorLabels.push_back(
          child.Read("method", true, "", "Connector Label"));
    else if(child.Name() == "ComponentConnector")
      m_componentConnectorLabels.push_back(
          child.Read("method", true, "", "Component Connector Label"));
    else if(child.Name() == "Evaluator")
      this->m_meLabels.push_back(
          child.Read("method", true, "", "Evaluator Label"));
  }
}

template <typename MPTraits>
void
BasicPRM<MPTraits>::
Print(ostream& _os) const {
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
  for(const auto& label : m_samplerLabels)
    _os << "\t\t" << label.first
        << "\tNumber:"   << label.second.first
        << "\tAttempts:" << label.second.second
        << std::endl;

  _os << "\tConnectors" << std::endl;
  for(const auto& label : m_connectorLabels)
    _os << "\t\t" << label << std::endl;

  _os << "\tComponentConnectors" << std::endl;
  for(const auto& label : m_componentConnectorLabels)
    _os << "\t\t" << label << std::endl;

  _os<<"\tMapEvaluators" << std::endl;
  for(const auto& label : this->m_meLabels)
    _os << "\t\t" << label << std::endl;
}

template <typename MPTraits>
void
BasicPRM<MPTraits>::
Initialize() {
  if(this->m_debug)
    std::cout << this->GetName() << "::Initialize"
              << std::endl;

  //read in and reload roadmap and evaluators
  if(!m_inputMapFilename.empty()) {
    RoadmapType* r = this->GetRoadmap();
    if(this->m_debug)
      std::cout << "Loading roadmap from \"" << m_inputMapFilename << "\"."
                << std::endl;

    r->Read(m_inputMapFilename.c_str());

    GraphType* g = r->GetGraph();
    for(typename GraphType::VI vi = g->begin(); vi != g->end(); ++vi)
      VDAddNode(g->GetVertex(vi));
    if(this->m_debug)
      std::cout << "Roadmap has " << g->get_num_vertices()
                << " nodes and " << g->get_num_edges() << " edges.\n"
                << "Resetting map evaluator states."
                << std::endl;

    /// @TODO This cannot be different for PRM than for any other strategy. It
    ///       must be removed or moved to the base class if needed.
    for(const auto& label: this->m_meLabels) {
      auto evaluator = this->GetMapEvaluator(label);
      if(evaluator->HasState())
        evaluator->operator()();
    }
  }
}


template <typename MPTraits>
void
BasicPRM<MPTraits>::
Iterate() {
  m_currentIteration++;
  if(this->m_debug)
    std::cout << this->GetName() << ":: starting iteration "
              << m_currentIteration
              << std::endl;

  vector<VID> vids;

  switch(m_startAt) {

    case Sampling:
      Sample(back_inserter(vids));

    case Connecting:
      {
        if(m_startAt == Connecting) {
          GraphType* g = this->GetRoadmap()->GetGraph();
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
        GraphType* g = this->GetRoadmap()->GetGraph();
        Connect(g->begin(), g->end(), m_componentConnectorLabels);
      }

    default:
      break;
  }
  m_startAt = Sampling;
}


template <typename MPTraits>
void
BasicPRM<MPTraits>::
Finalize() {
  // Output final map.
  this->GetRoadmap()->Write(this->GetBaseFilename() + ".map",
      this->GetEnvironment());

  // Output stats.
  ofstream  osStat(this->GetBaseFilename() + ".stat");
  StatClass* stats = this->GetStatClass();
  stats->PrintAllStats(osStat, this->GetRoadmap());
}


template <typename MPTraits>
template<typename OutputIterator>
void
BasicPRM<MPTraits>::
Sample(OutputIterator _thisIterationOut) {
  if(this->m_debug)
    std::cout << this->GetName() << "::Sample"
              << std::endl;

  StatClass* stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetName() + "::Sample");

  // Generate nodes with each sampler.
  vector<CfgType> samples;
  for(auto&  sampler : m_samplerLabels) {
    auto s = this->GetSampler(sampler.first);

    s->Sample(sampler.second.first, sampler.second.second,
        this->GetEnvironment()->GetBoundary(), back_inserter(samples));
  }

  if(this->m_debug)
    std::cout << "\tGenerated " << samples.size() << " samples."
              << std::endl;

  // Add valid samples to roadmap.
  GraphType* g = this->GetRoadmap()->GetGraph();
  for(auto& sample: samples) {
    const VID vid = g->AddVertex(sample);
    *_thisIterationOut++ = vid;
  }
}


template <typename MPTraits>
template<class InputIterator>
void
BasicPRM<MPTraits>::
Connect(InputIterator _first, InputIterator _last,
    const std::vector<string>& _labels) {
  if(this->m_debug)
    std::cout << this->GetNameAndLabel() << "::Connect()"
              << std::endl;

  StatClass* stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetName() + "::Connect");

  for(const auto& label : _labels) {
    auto c = this->GetConnector(label);
    c->Connect(this->GetRoadmap(), _first, _last);
  }

  if(this->m_debug) {
    GraphType* g = this->GetRoadmap()->GetGraph();
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
  if(this->m_debug)
    std::cout << this->GetName() << "::CheckNarrowPassageSamples"
              << std::endl;

  for(; _first != _last; _first++) {
    VID vid = this->GetRoadmap()->GetGraph()->GetVID(_first);
    if(this->CheckNarrowPassageSample(vid))
      break;
  }
}

#endif
