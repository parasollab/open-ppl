#ifndef BASICPRM_H_
#define BASICPRM_H_

#include "MPStrategyMethod.h"

template<class MPTraits>
class BasicPRM : public MPStrategyMethod<MPTraits> {
  public:
    enum Start {NODE_GENERATION, NODE_CONNECTION, COMPONENT_CONNECTION, MAP_EVALUATION};

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::SamplerPointer SamplerPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename MPProblemType::MapEvaluatorPointer MapEvaluatorPointer;

    BasicPRM(const map<string, pair<int, int> >& _samplerLabels = map<string, pair<int, int> >(),
        const vector<string>& _connectorLabels = vector<string>(),
        const vector<string>& _componentConnectorLabels = vector<string>(),
        const vector<string>& _evaluatorLabels = vector<string>(),
        string _vcLabel = "",
        string _inputMapFilename = "",
        Start _startAt = NODE_GENERATION);
    BasicPRM(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~BasicPRM();

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void Print(ostream& _os) const;

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

  protected:
    //helper functions for operator()
    template<class InputIterator>
      void ConnectNodes(InputIterator _first, InputIterator _last);
    void ConnectComponents();

    //data
    map<string, pair<int, int> > m_samplerLabels;
    vector<string> m_connectorLabels;
    vector<string> m_componentConnectorLabels;
    vector<string> m_evaluatorLabels;
    int m_currentIteration;
    string m_vcLabel;
    string m_inputMapFilename;
    Start m_startAt;

  private:
    template <typename OutputIterator>
      void GenerateNodes(OutputIterator _thisIterationOut);
};

template<class MPTraits>
BasicPRM<MPTraits>::BasicPRM(const map<string, pair<int, int> >& _samplerLabels,
    const vector<string>& _connectorLabels,
    const vector<string>& _componentConnectorLabels,
    const vector<string>& _evaluatorLabels,
    string _vcLabel,
    string _inputMapFilename,
    Start _startAt)
  : m_samplerLabels(_samplerLabels),
  m_connectorLabels(_connectorLabels), m_componentConnectorLabels(_componentConnectorLabels),
  m_evaluatorLabels(_evaluatorLabels), m_currentIteration(0), m_vcLabel(_vcLabel),
  m_inputMapFilename(_inputMapFilename), m_startAt(_startAt){
    this->SetName("BasicPRM");
  }

template<class MPTraits>
BasicPRM<MPTraits>::BasicPRM(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node), m_currentIteration(0),
  m_inputMapFilename(""), m_startAt(NODE_GENERATION){
    this->SetName("BasicPRM");
    ParseXML(_node);
  }

template<class MPTraits>
BasicPRM<MPTraits>::~BasicPRM(){
}

template<class MPTraits>
void
BasicPRM<MPTraits>::ParseXML(XMLNodeReader& _node) {
  m_inputMapFilename = _node.stringXMLParameter("inputMap", false, "",
      "filename of roadmap to start from");
  m_vcLabel = _node.stringXMLParameter("vcLabel", false, "", "Validity Checker in case Sampler does not verify validity of nodes.");

  string startAt = _node.stringXMLParameter("startAt", false, "node generation",
      "point of algorithm where to begin at: \"node generation\" (default), \"node connection\", \"component connection\", \"map evaluation\"");
  if(startAt == "node generation")
    m_startAt = NODE_GENERATION;
  else if(startAt == "node connection")
    m_startAt = NODE_CONNECTION;
  else if(startAt == "component connection")
    m_startAt = COMPONENT_CONNECTION;
  else if(startAt == "map evaluation")
    m_startAt = MAP_EVALUATION;
  else  {
    cerr << "\n\ndo not understand m_startAt = \"" << startAt
      << "\", choices are: 'node generation', 'node connection', 'component connection', and 'map evaluation', exiting.\n";
    exit(-1);
  }

  for(XMLNodeReader::childiterator cIter = _node.children_begin();
      cIter != _node.children_end(); ++cIter){
    if(cIter->getName() == "node_generation_method") {
      string generationMethod = cIter->stringXMLParameter("Method", true, "",
          "Node Generation Method");
      int numPerIteration = cIter->numberXMLParameter("Number", true, 1, 0,
          MAX_INT, "Number of samples");
      int attemptsPerIteration = cIter->numberXMLParameter("Attempts", false,
          1, 0, MAX_INT, "Number of attempts per sample");
      m_samplerLabels[generationMethod] = make_pair(numPerIteration,
          attemptsPerIteration);
      cIter->warnUnrequestedAttributes();
    }
    else if(cIter->getName() == "node_connection_method"){
      string connectMethod = cIter->stringXMLParameter("Method", true, "",
          "Node Connection Method");
      m_connectorLabels.push_back(connectMethod);
      cIter->warnUnrequestedAttributes();
    }
    else if(cIter->getName() == "component_connection_method"){
      string connectMethod = cIter->stringXMLParameter("Method", true, "",
          "Component Connection Method");
      m_componentConnectorLabels.push_back(connectMethod);
      cIter->warnUnrequestedAttributes();
    }
    else if(cIter->getName() == "evaluation_method"){
      string evalMethod = cIter->stringXMLParameter("Method", true, "",
          "Evaluation Method");
      m_evaluatorLabels.push_back(evalMethod);
      cIter->warnUnrequestedAttributes();
    }
    else
      cIter->warnUnknownNode();
  }
}

template<class MPTraits>
void
BasicPRM<MPTraits>::Print(ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);
  _os << "\tValidity Checker: " << m_vcLabel << endl;
  _os << "\tInput Map Filename: " << m_inputMapFilename << endl;
  _os << "\tm_startAt: ";

  switch(m_startAt){
    case NODE_GENERATION: _os << "node generation\n"; break;
    case NODE_CONNECTION: _os << "node connection\n"; break;
    case COMPONENT_CONNECTION: _os << "component connection\n"; break;
    case MAP_EVALUATION: _os << "map evaluation\n"; break;
  }

  typedef map<string, pair<int,int> >::const_iterator MIter;
  typedef vector<string>::const_iterator StringIter;
  _os<<"\nSamplers\n";
  for(MIter mIter=m_samplerLabels.begin();
      mIter!=m_samplerLabels.end(); mIter++){
    _os<<"\t"<<mIter->first<<"\tNumber:"<<mIter->second.first
      <<"\tAttempts:"<<mIter->second.second;
  }

  _os<<"\nNodeConnectors\n";
  for(StringIter sIter=m_connectorLabels.begin(); sIter!=m_connectorLabels.end(); sIter++){
    _os<<"\t"<<*sIter;
  }
  _os<<"\nComponentConnectors\n";
  for(StringIter sIter=m_componentConnectorLabels.begin(); sIter!=m_componentConnectorLabels.end(); sIter++){
    _os<<"\t"<<*sIter;
  }

  _os<<"\nMapEvaluators\n";
  for(StringIter sIter=m_evaluatorLabels.begin(); sIter!=m_evaluatorLabels.end(); sIter++){
    _os<<"\t"<<*sIter;
  }
}

template<class MPTraits>
void
BasicPRM<MPTraits>::Initialize(){
  if(this->m_debug) cout<<"\nInitializing BasicPRM::"<<endl;

  //read in and reload roadmap and evaluators
  if(m_inputMapFilename != "") {
    RoadmapType* rMap = this->GetMPProblem()->GetRoadmap();
    if(this->m_debug) cout << "\tLoading roadmap from \"" << m_inputMapFilename << "\"...";
    rMap->Read(m_inputMapFilename.c_str());
    for(typename GraphType::VI vi = rMap->GetGraph()->begin(); vi != rMap->GetGraph()->end(); ++vi)
      VDAddNode(rMap->GetGraph()->GetVertex(vi));
    if(this->m_debug) {
      cout << "\troadmap has " << rMap->GetGraph()->get_num_vertices() << " nodes and " << rMap->GetGraph()->get_num_edges() << " edges\n";
      cout << "\n\tResetting map evaluator states...\n";
    }
    for(vector<string>::iterator evalIter = m_evaluatorLabels.begin(); evalIter != m_evaluatorLabels.end(); ++evalIter) {
      MapEvaluatorPointer evaluator = this->GetMPProblem()->GetMapEvaluator(*evalIter);
      if(evaluator->HasState())
        evaluator->operator()();
    }
    if(this->m_debug) cout << "\tdone.\n";
  }

  if(this->m_debug) cout<<"\nEnding Initializing BasicPRM"<<endl;
}

template<class MPTraits>
void
BasicPRM<MPTraits>::Run(){

  if(this->m_debug) cout<<"\nRunning BasicPRM::"<<endl;

  //setup variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  stats->StartClock("Map Generation");

  //Pre-evaluate map in case Query is run and start/goal are added to the
  //roadmap. This pre-evaluation should not affect the start value of
  //mapPassedEvaluation.
  this->EvaluateMap(m_evaluatorLabels);
  bool mapPassedEvaluation = false;
  while(!mapPassedEvaluation){
    m_currentIteration++;
    vector<VID> vids;
    if(m_startAt <= NODE_GENERATION) {
      if(this->m_debug) cout << "\ngenerating nodes: ";
      GenerateNodes(back_insert_iterator<vector<VID> >(vids));
    }
    if(m_startAt <= NODE_CONNECTION) {
      if(this->m_debug) cout << "\nconnecting nodes: ";
      if(m_startAt == NODE_CONNECTION){
        GraphType* g = this->GetMPProblem()->GetRoadmap()->GetGraph();
        ConnectNodes(g->begin(), g->end());
      }
      else
        ConnectNodes(vids.begin(), vids.end());
    }
    if(m_startAt <= COMPONENT_CONNECTION) {
      if(this->m_debug) cout << "\nconnecting components: ";
      ConnectComponents();
    }
    if(m_startAt <= MAP_EVALUATION) {
      if(this->m_debug) cout << "\nevaluating roadmap: ";
      mapPassedEvaluation = this->EvaluateMap(m_evaluatorLabels);
    }
    m_startAt = NODE_GENERATION;
  }

  stats->StopClock("Map Generation");
  if(this->m_debug) {
    stats->PrintClock("Map Generation", cout);
    cout<<"\nEnd Running BasicPRM::"<<endl;
  }
}

template<class MPTraits>
void
BasicPRM<MPTraits>::Finalize(){
  if(this->m_debug) cout<<"\nFinalizing BasicPRM::"<<endl;

  //setup variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  string str;

  //output final map
  str = this->GetBaseFilename() + ".map";
  ofstream osMap(str.c_str());
  this->GetMPProblem()->GetRoadmap()->Write(osMap, this->GetMPProblem()->GetEnvironment());
  osMap.close();

  //output stats
  str = this->GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  osStat << "NodeGen+Connection Stats" << endl;
  stats->PrintAllStats(osStat, this->GetMPProblem()->GetRoadmap());
  stats->PrintClock("Map Generation", osStat);

  osStat.close();

  if(this->m_debug) cout<<"\nEnd Finalizing BasicPRM"<<endl;
}

template<class MPTraits>
template<class InputIterator>
void
BasicPRM<MPTraits>::ConnectNodes(InputIterator _first, InputIterator _last) {
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  string connectorClockName = "Total Node Connection";
  stats->StartClock(connectorClockName);
  stapl::sequential::vector_property_map<typename GraphType::GRAPH,size_t > cmap;

  for(vector<string>::iterator nodeLabelIter = m_connectorLabels.begin();
      nodeLabelIter != m_connectorLabels.end(); ++nodeLabelIter){

    ConnectorPointer pConnection = this->GetMPProblem()->GetConnector(*nodeLabelIter);

    string connectorSubClockName = "Node Connection::" + pConnection->GetNameAndLabel();
    stats->StartClock(connectorSubClockName);

    if(this->m_debug) cout << "\n\t";
    pConnection->Connect(this->GetMPProblem()->GetRoadmap(), *(this->GetMPProblem()->GetStatClass()), cmap, _first, _last);
    if(this->m_debug) {
      cmap.reset();
      cout << this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_edges() << " edges, "
        << get_cc_count(*(this->GetMPProblem()->GetRoadmap()->GetGraph()), cmap) << " connected components"
        << endl;
      cout << "\t";
    }

    stats->StopClock(connectorSubClockName);
    if(this->m_debug) stats->PrintClock(connectorSubClockName, cout);
  }

  for(; _first != _last; _first++) {
    VID vid = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVID(_first);
    if(this->CheckNarrowPassageSample(vid))
      break;
  }

  stats->StopClock(connectorClockName);
  if(this->m_debug) stats->PrintClock(connectorClockName, cout);
}

template<class MPTraits>
void
BasicPRM<MPTraits>::ConnectComponents() {
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  string clockName = "Total Connect Components";
  stats->StartClock(clockName);
  stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> cmap;

  for(vector<string>::iterator compConnLabelIter = m_componentConnectorLabels.begin();
      compConnLabelIter != m_componentConnectorLabels.end(); ++compConnLabelIter){
    ConnectorPointer pConnection = this->GetMPProblem()->GetConnector(*compConnLabelIter);

    string connectorClockName = "Connect Component::" + pConnection->GetNameAndLabel();
    stats->StartClock(connectorClockName);

    if(this->m_debug) cout << "\n\t";
    pConnection->Connect(this->GetMPProblem()->GetRoadmap(), *(this->GetMPProblem()->GetStatClass()), cmap);

    if(this->m_debug){
      cmap.reset();
      cout << this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_edges() << " edges, "
        << get_cc_count(*(this->GetMPProblem()->GetRoadmap()->GetGraph()), cmap) << " connected components"<< endl;
      cout << "\t";
    }
    stats->StopClock(connectorClockName);
    if(this->m_debug) stats->PrintClock(connectorClockName, cout);
  }
  stats->StopClock(clockName);
  if(this->m_debug) stats->PrintClock(clockName, cout);
}

template<class MPTraits>
template<typename OutputIterator>
void
BasicPRM<MPTraits>::GenerateNodes(OutputIterator _thisIterationOut){
  StatClass* pStatClass = this->GetMPProblem()->GetStatClass();
  string clockName = "Total Node Generation";
  pStatClass->StartClock(clockName);
  string callee("BasicPRM::GenerateNodes");

  typedef map<string, pair<int, int> >::iterator NodeGenIter;
  vector<CfgType> outNodes;
  for(NodeGenIter gIter = m_samplerLabels.begin(); gIter != m_samplerLabels.end(); ++gIter){
    SamplerPointer pNodeGenerator = this->GetMPProblem()->GetSampler(gIter->first);
    if(this->m_debug) pNodeGenerator->Print(cout);
    vector<CfgType> inNodes(gIter->second.first);

    //generate nodes for this node generator method
    string generatorClockName = "Sampler::" + gIter->first;
    pStatClass->StartClock(generatorClockName);

    if(this->m_debug) cout << "\n\t";

    do{
      pNodeGenerator->Sample(this->GetMPProblem()->GetEnvironment(), this->m_boundary,*pStatClass,
          inNodes.begin(),inNodes.end(),gIter->second.second, back_inserter(outNodes));
    }while(outNodes.size()<=0 && m_currentIteration==1);

    if(this->m_debug) {
      cout << this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_vertices() << " vertices " << endl;
      cout << "\n\t";
    }
    pStatClass->StopClock(generatorClockName);
    if(this->m_debug) pStatClass->PrintClock(generatorClockName, cout);
  }

  //add valid nodes to roadmap
  typedef typename vector<CfgType>::iterator CIT;
  for(CIT cit=outNodes.begin(); cit!=outNodes.end(); ++cit){
    if(cit->IsLabel("Lazy") && cit->GetLabel("Lazy")){
      if(!this->GetMPProblem()->GetRoadmap()->GetGraph()->IsVertex(*cit)) {
        VID vid = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(*cit);
        //store value and increment iterator
        *_thisIterationOut++ = vid;
      }
    }
    else{
      if(!cit->IsLabel("VALID"))
        !(this->GetMPProblem()->GetValidityChecker(m_vcLabel)->IsValid(*cit, callee));

      if(cit->IsLabel("VALID") && cit->GetLabel("VALID")) {
        if(!this->GetMPProblem()->GetRoadmap()->GetGraph()->IsVertex(*cit)) {
          VID vid = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(*cit);
          //store value and increment iterator
          *_thisIterationOut++ = vid;
        }
      }
    }
  }
  pStatClass->StopClock(clockName);
  if(this->m_debug) pStatClass->PrintClock(clockName, cout);
}

#endif
