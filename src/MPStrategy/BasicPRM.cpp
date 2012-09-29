#include "BasicPRM.h"
#include "MPStrategy.h"
#include "MapEvaluator.h"
#include "Sampler.h"
#include "Connector.h"
#include "Roadmap.h"
#include "RoadmapGraph.h"
#include "MetricUtils.h"
#include "ValidityChecker.h"

BasicPRM::BasicPRM(XMLNodeReader& _node, MPProblem* _problem) :
  MPStrategyMethod(_node, _problem), m_currentIteration(0), 
  m_useProbability(false), m_inputMapFilename(""), m_startAt(NODE_GENERATION){
    //read input
    ParseXML(_node);
}
BasicPRM::~BasicPRM(){
}

void BasicPRM::ParseXML(XMLNodeReader& _node) {
  m_inputMapFilename = _node.stringXMLParameter("inputMap", false, "", 
    "filename of roadmap to start from");
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

  for(XMLNodeReader::childiterator citr = _node.children_begin(); 
      citr != _node.children_end(); ++citr){
    if(citr->getName() == "node_generation_method") {
      string generationMethod = citr->stringXMLParameter("Method", true, "", 
        "Node Connection Method");
      int numPerIteration = citr->numberXMLParameter("Number", true, 1, 0, 
        MAX_INT, "Number of samples");
      int attemptsPerIteration = citr->numberXMLParameter("Attempts", false, 
        1, 0, MAX_INT, "Number of attempts per sample");
      double probPerIteration = 0;
      if(numPerIteration == 0){
        probPerIteration = citr->numberXMLParameter("Probability", true, 0.0, 
          0.0, 1.0, "Number of samples");
        m_useProbability = true;
      }
      m_nodeGenerationLabels[generationMethod] = make_pair(numPerIteration, 
        attemptsPerIteration);
      m_probGenerationLabels[generationMethod] = make_pair(probPerIteration, 
        attemptsPerIteration);
      citr->warnUnrequestedAttributes();
    } 
    else if(citr->getName() == "node_connection_method"){
      string connectMethod = citr->stringXMLParameter("Method", true, "", 
        "Node Connection Method");
      m_nodeConnectionLabels.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    } 
    else if(citr->getName() == "component_connection_method"){
      string connectMethod = citr->stringXMLParameter("Method", true, "", 
        "Component Connection Method");
      m_componentConnectionLabels.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    } 
    else if(citr->getName() == "evaluation_method"){
      string evalMethod = citr->stringXMLParameter("Method", true, "", 
        "Evaluation Method");
      m_evaluatorLabels.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName()=="vc_method"){
      m_vcMethod =citr->stringXMLParameter("Method",true,"",
        "ValidityCheckerMethod");
      citr->warnUnrequestedAttributes();
    }
    else
      citr->warnUnknownNode();
  }
  if (m_debug) PrintOptions(cout);
}

void BasicPRM::PrintOptions(ostream& _os) {
  _os << "BasicPRM::PrintOptions" << endl;
  _os << "\tValidity Checker: " << m_vcMethod << endl;
  _os << "\tInput Map Filename: " << m_inputMapFilename << endl;
  _os << "\tm_startAt: ";
  
  switch(m_startAt){
    case NODE_GENERATION: _os << "node generation\n"; break;
    case NODE_CONNECTION: _os << "node connection\n"; break;
    case COMPONENT_CONNECTION: _os << "component connection\n"; break;
    case MAP_EVALUATION: _os << "map evaluation\n"; break;
  }

  typedef map<string, pair<double,int> >::iterator PIT;
  typedef map<string, pair<int,int> >::iterator MIT;
  typedef vector<string>::iterator SIT;
  _os<<"\nNodeGenerators\n";
  if(!m_useProbability){
    for(MIT mit=m_nodeGenerationLabels.begin(); 
        mit!=m_nodeGenerationLabels.end(); mit++){
      _os<<"\t"<<mit->first<<"\tNumber:"<<mit->second.first
         <<"\tAttempts:"<<mit->second.second<<"\tOptions:\n";
      GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(mit->first)->PrintOptions(_os);
    }
  }
  else{
    for(PIT pit=m_probGenerationLabels.begin(); 
        pit!=m_probGenerationLabels.end(); pit++){
      _os<<"\t"<<pit->first<<"\tProbability:"<<pit->second.first
      <<"\tAttempts:"<<pit->second.second<<"\tOptions:\n";
      GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(pit->first)->PrintOptions(_os);
    }
  }

  _os<<"\nNodeConnectors\n";
  for(SIT sit=m_nodeConnectionLabels.begin(); sit!=m_nodeConnectionLabels.end(); sit++){
    _os<<"\t"<<*sit<<"\tOptions:\n";
    GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*sit)->PrintOptions(_os);
  }
  _os<<"\nComponentConnectors\n";
  for(SIT sit=m_componentConnectionLabels.begin(); sit!=m_componentConnectionLabels.end(); sit++){
    _os<<"\t"<<*sit<<"\tOptions:\n";
    GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*sit)->PrintOptions(_os);
  }

  _os<<"\nMapEvaluators\n";
  for(SIT sit=m_evaluatorLabels.begin(); sit!=m_evaluatorLabels.end(); sit++){
    _os<<"\t"<<*sit<<"\tOptions:\n";
    GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetMethod(*sit)->PrintOptions(_os);
  }
}

void BasicPRM::Initialize(){
  if (m_debug) cout<<"\nInitializing BasicPRM::"<<endl;

  //read in and reload roadmap and evaluators
  if(m_inputMapFilename != "") {
    Roadmap<CfgType, WeightType>* rmap = GetMPProblem()->GetRoadmap();
    if (m_debug) cout << "\tLoading roadmap from \"" << m_inputMapFilename << "\"...";
    rmap->ReadRoadmapGRAPHONLY(m_inputMapFilename.c_str());
    for(RoadmapGraph<CfgType, WeightType>::VI vi = rmap->m_pRoadmap->begin(); vi != rmap->m_pRoadmap->end(); ++vi)
      VDAddNode(vi->property());
    if (m_debug) {
      cout << "\troadmap has " << rmap->m_pRoadmap->get_num_vertices() << " nodes and " << rmap->m_pRoadmap->get_num_edges() << " edges\n";
      cout << "\n\tResetting map evaluator states...\n";
    }
    for(vector<string>::iterator I = m_evaluatorLabels.begin(); I != m_evaluatorLabels.end(); ++I) {
      MapEvaluator<CfgType, WeightType>::MapEvaluationPointer evaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetMethod(*I);
      if(evaluator->HasState())
        evaluator->operator()();
    }
    if (m_debug) cout << "\tdone.\n";
  }

  if (m_debug) cout<<"\nEnding Initializing BasicPRM"<<endl;
}

void BasicPRM::Run(){
 
  if (m_debug) cout<<"\nRunning BasicPRM::"<<endl;

  //setup variables
  StatClass* stats = GetMPProblem()->GetStatClass();
  vector<VID> allNodesVID;
  GetMPProblem()->GetRoadmap()->m_pRoadmap->GetVerticesVID(allNodesVID);

  stats->StartClock("Map Generation");
  
  bool mapPassedEvaluation = false;
  while(!mapPassedEvaluation){
    if(allNodesVID.size() != GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_vertices()){
      allNodesVID.clear();
      GetMPProblem()->GetRoadmap()->m_pRoadmap->GetVerticesVID(allNodesVID);
      if(m_debug)
        cout << "\nMy roadmap is not equal to my vids.\nnum nodes = " << allNodesVID.size() << endl;
    }
    m_currentIteration++;
    vector<VID> thisIterationNodesVID;
    if(m_startAt <= NODE_GENERATION) {
      if (m_debug) cout << "\ngenerating nodes: ";
      GenerateNodes(back_insert_iterator<vector<VID> >(allNodesVID), 
          back_insert_iterator<vector<VID> >(thisIterationNodesVID));
    }
    else
      GetMPProblem()->GetRoadmap()->m_pRoadmap->GetVerticesVID(thisIterationNodesVID);
    if(m_startAt <= NODE_CONNECTION) {
      if (m_debug) cout << "\nconnecting nodes: ";
      ConnectNodes(allNodesVID, thisIterationNodesVID);
    }
    if(m_startAt <= COMPONENT_CONNECTION) {
      if (m_debug) cout << "\nconnecting components: ";
      ConnectComponents();
    }
    if(m_startAt <= MAP_EVALUATION) {
      if (m_debug) cout << "\nevaluating roadmap: ";
      mapPassedEvaluation = EvaluateMap(m_evaluatorLabels);
    }
    m_startAt = NODE_GENERATION;
  }

  stats->StopClock("Map Generation");
  if (m_debug) stats->PrintClock("Map Generation", cout);

  if (m_debug) cout<<"\nEnd Running BasicPRM::"<<endl;
}

void BasicPRM::Finalize(){
  if (m_debug) cout<<"\nFinalizing BasicPRM::"<<endl;

  //setup variables
  StatClass* stats = GetMPProblem()->GetStatClass();

  string str;

  //output final map
  str = GetBaseFilename() + ".map";
  ofstream osMap(str.c_str());
  GetMPProblem()->WriteRoadmapForVizmo(osMap);
  osMap.close();

  //output stats
  str = GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  osStat << "NodeGen+Connection Stats" << endl;
  stats->PrintAllStats(osStat, GetMPProblem()->GetRoadmap());
  stats->PrintClock("Map Generation", osStat);
  
  osStat.close();

  if (m_debug) cout<<"\nEnd Finalizing BasicPRM"<<endl;
}

void 
BasicPRM::ConnectNodes(vector<VID>& allNodesVID, vector<VID>& thisIterationNodesVID) {
  StatClass* stats = GetMPProblem()->GetStatClass();
  string connectorClockName = "Total Node Connection";
  stats->StartClock(connectorClockName);
  stapl::sequential::vector_property_map< RoadmapGraph<CfgType, WeightType>::GRAPH,size_t > cmap;

  for(vector<string>::iterator I = m_nodeConnectionLabels.begin(); 
      I != m_nodeConnectionLabels.end(); ++I){

    Connector<CfgType, WeightType>::ConnectionPointer pConnection;
    pConnection = GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*I);    

    string connectorSubClockName = "Node Connection::" + pConnection->GetName();
    stats->StartClock(connectorSubClockName);

    if(m_debug) cout << "\n\t";
    vector<VID> nodesVID(thisIterationNodesVID.begin(), thisIterationNodesVID.end());
    pConnection->Connect(GetMPProblem()->GetRoadmap(), *(GetMPProblem()->GetStatClass()), cmap,
        nodesVID.begin(), nodesVID.end(), allNodesVID.begin(), allNodesVID.end());
    if (m_debug) {
      cmap.reset();
      cout << GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_edges() << " edges, " 
        << get_cc_count(*(GetMPProblem()->GetRoadmap()->m_pRoadmap), cmap) << " connected components"
        << endl;
      cout << "\t";
    }

    stats->StopClock(connectorSubClockName);
    if (m_debug) stats->PrintClock(connectorSubClockName, cout);
  }
  stats->StopClock(connectorClockName);
  if (m_debug) stats->PrintClock(connectorClockName, cout);
}

void BasicPRM::ConnectComponents() {
  StatClass* stats = GetMPProblem()->GetStatClass();
  string clockName = "Total Connect Components";
  stats->StartClock(clockName);
  stapl::sequential::vector_property_map< RoadmapGraph<CfgType, WeightType>::GRAPH,size_t > cmap;

  for(vector<string>::iterator I = m_componentConnectionLabels.begin(); 
      I != m_componentConnectionLabels.end(); ++I){
    Connector<CfgType, WeightType>::ConnectionPointer pConnection;
    pConnection = GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*I);

    string connectorClockName = "Connect Component::" + pConnection->GetName();
    stats->StartClock(connectorClockName);

    if(m_debug) cout << "\n\t";
    pConnection->Connect(GetMPProblem()->GetRoadmap(), *(GetMPProblem()->GetStatClass()), cmap);

    if (m_debug){
      cmap.reset();
      cout << GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_edges() << " edges, " 
      << get_cc_count(*(GetMPProblem()->GetRoadmap()->m_pRoadmap), cmap) << " connected components"<< endl;
      cout << "\t";
    }
    stats->StopClock(connectorClockName);
    if (m_debug) stats->PrintClock(connectorClockName, cout);
  }
   stats->StopClock(clockName);
  if(m_debug) stats->PrintClock(clockName, cout);
}

string BasicPRM::PickNextSampler(){
  double rand = DRand();
  double total = 1.0;
  string index = "";
  typedef map<string, pair<double,int> >::iterator GIT;
  for(GIT git = m_probGenerationLabels.begin(); git!=m_probGenerationLabels.end(); git++){
    if(rand>=total){
      break;
    }
    else{
      index = git->first;
      total-=git->second.first;
    }
  }
  if (m_debug) cout<<"Choosing "<<index<<" as the next node generator"<<endl;
  return index;
}

template <typename OutputIterator> void 
BasicPRM::GenerateNodes(OutputIterator _allOut, OutputIterator _thisIterationOut){
  CDInfo cdInfo;
  StatClass* pStatClass = GetMPProblem()->GetStatClass();
  string clockName = "Total Node Generation"; 
  pStatClass->StartClock(clockName);
  string callee("BasicPRM::GenerateNodes");

  typedef map<string, pair<int, int> >::iterator GIT;
  vector<CfgType> outNodes;
  if(!m_useProbability){
    for(GIT git = m_nodeGenerationLabels.begin(); git != m_nodeGenerationLabels.end(); ++git){
      Sampler<CfgType>::SamplerPointer pNodeGenerator = GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(git->first);
     if(m_debug) pNodeGenerator->PrintOptions(cout);
      vector<CfgType> inNodes(git->second.first);

      //generate nodes for this node generator method
      string generatorClockName = "Sampler::" + git->first; 
      pStatClass->StartClock(generatorClockName);

      if(m_debug) cout << "\n\t";

      do{
        pNodeGenerator->Sample(GetMPProblem()->GetEnvironment(),m_boundary,*pStatClass,
            inNodes.begin(),inNodes.end(),git->second.second, back_inserter(outNodes));
      }while(outNodes.size()<=0&&m_currentIteration==1);

      if (m_debug) {
        cout << GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_vertices() << " vertices " << endl;
        cout << "\n\t";
      }
      pStatClass->StopClock(generatorClockName);
      if(m_debug) pStatClass->PrintClock(generatorClockName, cout);

    }
  }
  else{
    string nextNodeGen = PickNextSampler();
    Sampler<CfgType>::SamplerPointer pNodeGenerator; 
    pNodeGenerator = GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(nextNodeGen);
    if(m_debug) pNodeGenerator->PrintOptions(cout);
    vector<CfgType> inNodes(1);

    //generate nodes for this node generator method
    string generatorClockName = "Sampler::" + pNodeGenerator->GetName(); 
    pStatClass->StartClock(generatorClockName);

    if(m_debug) cout << "\n\t";
    pNodeGenerator->Sample(GetMPProblem()->GetEnvironment(),m_boundary,
      *pStatClass,inNodes.begin(),inNodes.end(),m_probGenerationLabels[nextNodeGen].second, back_inserter(outNodes));

    if (m_debug) {
      cout << GetMPProblem()->GetRoadmap()->m_pRoadmap->get_num_vertices() << " vertices " << endl;
      cout << "\n\t";
    }
    pStatClass->StopClock(generatorClockName);
    if(m_debug) pStatClass->PrintClock(generatorClockName, cout);
  }
  //add valid nodes to roadmap
  typedef vector<CfgType>::iterator CIT;
  for(CIT cit=outNodes.begin(); cit!=outNodes.end(); ++cit){
    if(cit->IsLabel("Lazy") && cit->GetLabel("Lazy")){
      if(!GetMPProblem()->GetRoadmap()->m_pRoadmap->IsVertex(*cit)) {
        VID vid = GetMPProblem()->GetRoadmap()->m_pRoadmap->AddVertex(*cit);
        //store value and increment iterator
        *_thisIterationOut++ = vid;
        *_allOut++ = vid;
      }
    }
    else{ 
      if(!cit->IsLabel("VALID")){
        !(GetMPProblem()->GetValidityChecker()->GetMethod(m_vcMethod)->IsValid(
              *cit, GetMPProblem()->GetEnvironment(), *(GetMPProblem()->GetStatClass()), cdInfo, &callee));
      }
      if(cit->IsLabel("VALID") && cit->GetLabel("VALID")) {
        if(!GetMPProblem()->GetRoadmap()->m_pRoadmap->IsVertex(*cit)) {
          VID vid = GetMPProblem()->GetRoadmap()->m_pRoadmap->AddVertex(*cit);
          //store value and increment iterator
          *_thisIterationOut++ = vid;
          *_allOut++ = vid;
        }
      }
    }
  }
  pStatClass->StopClock(clockName);
  if(m_debug) pStatClass->PrintClock(clockName, cout);
}

