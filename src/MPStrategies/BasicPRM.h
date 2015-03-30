#ifndef BASICPRM_H_
#define BASICPRM_H_

#include "MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Basic PRM approach
///
/// BasicPRM essentially combines samplers and connectors to iteratively
/// construct a roadmap until planning is "done"
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class BasicPRM : public MPStrategyMethod<MPTraits> {
  public:

    enum Start {Sampling, Connecting, ConnectingComponents, Evaluating};

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::SamplerPointer SamplerPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename MPProblemType::MapEvaluatorPointer MapEvaluatorPointer;

    BasicPRM(
        const map<string, pair<size_t, size_t> >& _samplerLabels = map<string, pair<size_t, size_t> >(),
        const vector<string>& _connectorLabels = vector<string>(),
        const vector<string>& _componentConnectorLabels = vector<string>(),
        const vector<string>& _evaluatorLabels = vector<string>(),
        string _inputMapFilename = "",
        Start _startAt = Sampling);
    BasicPRM(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~BasicPRM() {}

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void Print(ostream& _os) const;

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

  protected:
    ////////////////////////////////////////////////////////////////////////////
    /// @breif Sample and add configurations to the roadmap.
    /// @tparam OutputIterator Output iterator on data structure of VIDs
    /// @param[out] _thisIterationOut Data structure of VIDs of added nodes.
    template <typename OutputIterator>
      void Sample(OutputIterator _thisIterationOut);

    ////////////////////////////////////////////////////////////////////////////
    /// @breif Connect nodes and CCs of the roadmap
    /// @tparam InputIterator Iterator on data structure of VIDs/graph nodes
    /// @param _first Begin iterator over VIDs/graph nodes
    /// @param _last End iterator over VIDs/graph nodes
    /// @param _labels Connector labels used in connection
    template<class InputIterator>
      void Connect(InputIterator _first, InputIterator _last,
          const vector<string>& _labels);

    ////////////////////////////////////////////////////////////////////////////
    /// @breif Iterate over range and check nodes to be within narrow passage
    /// @tparam InputIterator Iterator on data structure of VIDs
    /// @param _first Begin iterator over VIDs
    /// @param _last End iterator over VIDs
    template<class InputIterator>
      void CheckNarrowPassageSamples(InputIterator _first, InputIterator _last);

    map<string, pair<size_t, size_t> > m_samplerLabels; ///< Sampler labels with number and attempts of sampler
    vector<string> m_connectorLabels; ///< Connector labels for node-to-node
    vector<string> m_componentConnectorLabels; ///< Connector labels for cc-to-cc
    vector<string> m_evaluatorLabels; ///< Evaluator labels
    size_t m_currentIteration; ///< Current iteration of while-loop of Run function
    string m_inputMapFilename; ///< Input roadmap to initialize map
    Start m_startAt; ///< When inputting a roadmap, specifies where in algorithm to start
};

template<class MPTraits>
BasicPRM<MPTraits>::
BasicPRM(
    const map<string, pair<size_t, size_t> >& _samplerLabels,
    const vector<string>& _connectorLabels,
    const vector<string>& _componentConnectorLabels,
    const vector<string>& _evaluatorLabels,
    string _inputMapFilename,
    Start _startAt)
  : m_samplerLabels(_samplerLabels),
  m_connectorLabels(_connectorLabels), m_componentConnectorLabels(_componentConnectorLabels),
  m_evaluatorLabels(_evaluatorLabels), m_currentIteration(0),
  m_inputMapFilename(_inputMapFilename), m_startAt(_startAt){
    this->SetName("BasicPRM");
  }

template<class MPTraits>
BasicPRM<MPTraits>::
BasicPRM(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node), m_currentIteration(0),
  m_inputMapFilename(""), m_startAt(Sampling){
    this->SetName("BasicPRM");
    ParseXML(_node);
  }

template<class MPTraits>
void
BasicPRM<MPTraits>::
ParseXML(XMLNodeReader& _node) {
  m_inputMapFilename = _node.stringXMLParameter("inputMap", false, "",
      "filename of roadmap to start from");

  string startAt = _node.stringXMLParameter("startAt", false, "sampling",
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
    throw ParseException(WHERE, message);
  }

  typedef XMLNodeReader::childiterator CIT;
  for(CIT cit = _node.children_begin(); cit != _node.children_end(); ++cit) {
    if(cit->getName() == "Sampler") {
      string s = cit->stringXMLParameter("method", true, "", "Sampler Label");
      size_t num = cit->numberXMLParameter("number", true,
          1, 0, MAX_INT, "Number of samples");
      size_t attempts = cit->numberXMLParameter("attempts", false,
          1, 0, MAX_INT, "Number of attempts per sample");
      m_samplerLabels[s] = make_pair(num, attempts);
    }
    else if(cit->getName() == "Connector"){
      string c = cit->stringXMLParameter("method", true, "", "Connector Label");
      m_connectorLabels.push_back(c);
    }
    else if(cit->getName() == "ComponentConnector"){
      string c = cit->stringXMLParameter("method", true, "", "Component Connector Label");
      m_componentConnectorLabels.push_back(c);
    }
    else if(cit->getName() == "Evaluator"){
      string e = cit->stringXMLParameter("method", true, "", "Evaluator Label");
      m_evaluatorLabels.push_back(e);
    }
    else
      cit->warnUnknownNode();
    cit->warnUnrequestedAttributes();
  }
}

template<class MPTraits>
void
BasicPRM<MPTraits>::
Print(ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);
  _os << "\tInput Map: " << m_inputMapFilename << endl;

  _os << "\tStart At: ";
  switch(m_startAt){
    case Sampling: _os << "sampling"; break;
    case Connecting: _os << "connecting"; break;
    case ConnectingComponents: _os << "connectingcomponents"; break;
    case Evaluating: _os << "evaluating"; break;
  }
  cout << endl;

  typedef map<string, pair<size_t, size_t> >::const_iterator MIT;
  _os << "\tSamplers" << endl;
  for(MIT mit = m_samplerLabels.begin(); mit != m_samplerLabels.end(); ++mit)
    _os << "\t\t" << mit->first
      << "\tNumber:" << mit->second.first
      << "\tAttempts:" << mit->second.second
      << endl;

  typedef vector<string>::const_iterator SIT;
  _os << "\tConnectors" << endl;
  for(SIT sit = m_connectorLabels.begin(); sit != m_connectorLabels.end(); ++sit)
    _os << "\t\t" << *sit << endl;

  _os << "\tComponentConnectors" << endl;
  for(SIT sit = m_componentConnectorLabels.begin(); sit != m_componentConnectorLabels.end(); ++sit)
    _os << "\t\t" << *sit << endl;

  _os<<"\tMapEvaluators" << endl;
  for(SIT sit = m_evaluatorLabels.begin(); sit != m_evaluatorLabels.end(); ++sit)
    _os << "\t\t" << *sit << endl;
}

template<class MPTraits>
void
BasicPRM<MPTraits>::
Initialize() {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Initialize()" << endl;

  //read in and reload roadmap and evaluators
  if(!m_inputMapFilename.empty()) {
    RoadmapType* r = this->GetRoadmap();
    if(this->m_debug)
      cout << "Loading roadmap from \"" << m_inputMapFilename << "\".";

    r->Read(m_inputMapFilename.c_str());

    GraphType* g = r->GetGraph();
    for(typename GraphType::VI vi = g->begin(); vi != g->end(); ++vi)
      VDAddNode(g->GetVertex(vi));
    if(this->m_debug) {
      cout << "Roadmap has " << g->get_num_vertices() << " nodes and "
        << g->get_num_edges() << " edges." << endl;
      cout << "Resetting map evaluator states." << endl;
    }

    typedef vector<string>::const_iterator SIT;
    for(SIT sit = m_evaluatorLabels.begin(); sit != m_evaluatorLabels.end(); ++sit) {
      MapEvaluatorPointer evaluator = this->GetMapEvaluator(*sit);
      if(evaluator->HasState())
        evaluator->operator()();
    }
  }
}

template<class MPTraits>
void
BasicPRM<MPTraits>::
Run(){

  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Run()"<<endl;

  this->GetStatClass()->StartClock(this->GetNameAndLabel());

  bool done = this->EvaluateMap(m_evaluatorLabels);
  while(!done) {
    m_currentIteration++;
    vector<VID> vids;

    switch(m_startAt) {

      case Sampling:
        Sample(back_inserter(vids));

      case Connecting:
        {
          if(m_startAt == Connecting){
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

      case Evaluating:
        done = this->EvaluateMap(m_evaluatorLabels);
    }
    m_startAt = Sampling;
  }

  this->GetStatClass()->StopClock(this->GetNameAndLabel());
  if(this->m_debug)
    this->GetStatClass()->PrintClock(this->GetNameAndLabel(), cout);
}

template<class MPTraits>
void
BasicPRM<MPTraits>::
Finalize() {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Finalize()" << endl;

  //output final map
  this->GetRoadmap()->Write(this->GetBaseFilename() + ".map", this->GetEnvironment());

  //output stats
  string str = this->GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  StatClass* stats = this->GetStatClass();
  stats->PrintAllStats(osStat, this->GetRoadmap());
  stats->PrintClock("Map Generation", osStat);
}

template<class MPTraits>
template<typename OutputIterator>
void
BasicPRM<MPTraits>::
Sample(OutputIterator _thisIterationOut) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Sample()";

  StatClass* stats = this->GetStatClass();
  string clockName = "Total Node Generation";
  stats->StartClock(clockName);

  //For each sampler generate nodes into samples
  typedef map<string, pair<size_t, size_t> >::const_iterator SIT;
  vector<CfgType> samples;
  for(SIT sit = m_samplerLabels.begin(); sit != m_samplerLabels.end(); ++sit) {
    SamplerPointer s = this->GetSampler(sit->first);

    stats->StartClock(s->GetNameAndLabel());

    s->Sample(this->GetEnvironment(), this->m_boundary, *this->GetStatClass(),
        sit->second.first,
        sit->second.second, back_inserter(samples));

    stats->StopClock(s->GetNameAndLabel());
  }

  if(this->m_debug && samples.empty())
    cout << "No samples generated." << endl;

  //add valid samples to roadmap
  GraphType* g = this->GetRoadmap()->GetGraph();
  typedef typename vector<CfgType>::iterator CIT;
  for(CIT cit=samples.begin(); cit != samples.end(); ++cit) {
    VID vid = g->AddVertex(*cit);
    *_thisIterationOut++ = vid;
  }

  stats->StopClock(clockName);
  if(this->m_debug) {
    cout << this->GetNameAndLabel() << " has "
      << g->get_num_vertices() << " total vertices. Time: " << endl;
    this->GetStatClass()->PrintClock(clockName, cout);
  }
}

template<class MPTraits>
template<class InputIterator>
void
BasicPRM<MPTraits>::
Connect(InputIterator _first, InputIterator _last, const vector<string>& _labels) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::Connect()";

  StatClass* stats = this->GetStatClass();
  string clockName = "Total Connection";
  stats->StartClock(clockName);

  typename GraphType::ColorMap cmap;
  typedef vector<string>::const_iterator SIT;
  for(SIT sit = _labels.begin(); sit != _labels.end(); ++sit){
    ConnectorPointer c = this->GetConnector(*sit);

    stats->StartClock(c->GetNameAndLabel());

    c->Connect(this->GetRoadmap(), *this->GetStatClass(), cmap, _first, _last);

    stats->StopClock(c->GetNameAndLabel());
  }

  stats->StopClock(clockName);
  if(this->m_debug) {
    GraphType* g = this->GetRoadmap()->GetGraph();
    cmap.reset();
    cout << this->GetNameAndLabel() << " has "
      << g->get_num_edges() << " edges and "
      << g->GetNumCCs() << " connected components. Time: " << endl;
    stats->PrintClock(clockName, cout);
  }
}

template<class MPTraits>
template<class InputIterator>
void
BasicPRM<MPTraits>::
CheckNarrowPassageSamples(InputIterator _first, InputIterator _last) {
  if(this->m_debug)
    cout << this->GetNameAndLabel() << "::CheckNarrowPassageSamples()";

  for(; _first != _last; _first++) {
    VID vid = this->GetRoadmap()->GetGraph()->GetVID(_first);
    if(this->CheckNarrowPassageSample(vid))
      break;
  }
}

#endif
