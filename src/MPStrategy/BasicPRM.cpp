#include "BasicPRM.h"
#include "MPRegion.h"
#include "MPStrategy.h"
#include "MapEvaluator.h"
#include "Sampler.h"
#include "ConnectMap.h"
#include "Roadmap.h"
#include "RoadmapGraph.h"
#include "MetricUtils.h"
#include "ValidityChecker.hpp"

BasicPRM::BasicPRM(XMLNodeReader& _node, MPProblem* _problem) :
  MPStrategyMethod(_node, _problem), m_currentIteration(0), m_useProbability(false), m_inputMapFilename(""), m_startAt(NODE_GENERATION) {
    //read input
    ParseXML(_node);
  }

BasicPRM::~BasicPRM(){
}

void BasicPRM::ParseXML(XMLNodeReader& _node) {
  m_inputMapFilename = _node.stringXMLParameter("inputMap", false, "", "filename of roadmap to start from");
  string startAt = _node.stringXMLParameter("startAt", false, "node generation", "point of algorithm where to begin at: \"node generation\" (default), \"node connection\", \"component connection\", \"map evaluation\"");
  if(startAt == "node generation")
    m_startAt = NODE_GENERATION;
  else if(startAt == "node connection")
    m_startAt = NODE_CONNECTION;
  else if(startAt == "component connection")
    m_startAt = COMPONENT_CONNECTION;
  else if(startAt == "map evaluation")
    m_startAt = MAP_EVALUATION;
  else  {
    cerr << "\n\ndo not understand m_startAt = \"" << startAt << "\", choices are: 'node generation', 'node connection', 'component connection', and 'map evaluation', exiting.\n";
    exit(-1);
  }

  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr){
    if(citr->getName() == "node_generation_method") {
      string generationMethod = citr->stringXMLParameter("Method", true, "", "Node Connection Method");
      int numPerIteration = citr->numberXMLParameter("Number", true, 1, 0, MAX_INT, "Number of samples");
      int attemptsPerIteration = citr->numberXMLParameter("Attempts", false, 1, 0, MAX_INT, "Number of attempts per sample");
      double probPerIteration = 0;
      if(numPerIteration == 0){
        probPerIteration = citr->numberXMLParameter("Probability", true, 0.0, 0.0, 1.0, "Number of samples");
        m_useProbability = true;
      }
      m_nodeGenerationLabels[generationMethod] = make_pair(numPerIteration, attemptsPerIteration);
      m_probGenerationLabels[generationMethod] = make_pair(probPerIteration, attemptsPerIteration);
      citr->warnUnrequestedAttributes();
    } 
    else if(citr->getName() == "node_connection_method"){
      string connectMethod = citr->stringXMLParameter("Method", true, "", "Node Connection Method");
      m_nodeConnectionLabels.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    } 
    else if(citr->getName() == "component_connection_method"){
      string connectMethod = citr->stringXMLParameter("Method", true, "", "Component Connection Method");
      m_componentConnectionLabels.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    } 
    else if(citr->getName() == "evaluation_method"){
      string evalMethod = citr->stringXMLParameter("Method", true, "", "Evaluation Method");
      m_evaluatorLabels.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName()=="vc_method"){
      m_vcMethod =citr->stringXMLParameter("Method",true,"","ValidityCheckerMethod");
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
  switch(m_startAt)
  {
    case NODE_GENERATION: cout << "node generation\n"; break;
    case NODE_CONNECTION: cout << "node connection\n"; break;
    case COMPONENT_CONNECTION: cout << "component connection\n"; break;
    case MAP_EVALUATION: cout << "map evaluation\n"; break;
  }

  typedef map<string, pair<double,int> >::iterator PIT;
  typedef map<string, pair<int,int> >::iterator MIT;
  typedef vector<string>::iterator SIT;
  _os<<"\nNodeGenerators\n";
  if(!m_useProbability){
    for(MIT mit=m_nodeGenerationLabels.begin(); mit!=m_nodeGenerationLabels.end(); mit++){
      _os<<"\t"<<mit->first<<"\tNumber:"<<mit->second.first<<"\tAttempts:"<<mit->second.second<<"\tOptions:\n";
      GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(mit->first)->PrintOptions(_os);
    }
  }
  else{
    for(PIT pit=m_probGenerationLabels.begin(); pit!=m_probGenerationLabels.end(); pit++){
      _os<<"\t"<<pit->first<<"\tProbability:"<<pit->second.first<<"\tAttempts:"<<pit->second.second<<"\tOptions:\n";
      GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(pit->first)->PrintOptions(_os);
    }
  }
  _os<<"\nNodeConnectors\n";
  for(SIT sit=m_nodeConnectionLabels.begin(); sit!=m_nodeConnectionLabels.end(); sit++){
    _os<<"\t"<<*sit<<"\tOptions:\n";
    GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetNodeMethod(*sit)->PrintOptions(_os);
  }
  _os<<"\nComponentConnectors\n";
  for(SIT sit=m_componentConnectionLabels.begin(); sit!=m_componentConnectionLabels.end(); sit++){
    _os<<"\t"<<*sit<<"\tOptions:\n";
    GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetComponentMethod(*sit)->PrintOptions(_os);
  }
  _os<<"\nMapEvaluators\n";
  for(SIT sit=m_evaluatorLabels.begin(); sit!=m_evaluatorLabels.end(); sit++){
    _os<<"\t"<<*sit<<"\tOptions:\n";
    GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*sit)->PrintOptions(_os);
  }
}

void BasicPRM::Initialize(int _regionID){
  if (m_debug) cout<<"\nInitializing BasicPRM::"<<_regionID<<endl;

  //read in and reload roadmap and evaluators
  if(m_inputMapFilename != "") {
    Roadmap<CfgType, WeightType>* rmap = GetMPProblem()->GetMPRegion(_regionID)->GetRoadmap();
    if (m_debug) cout << "\tLoading roadmap from \"" << m_inputMapFilename << "\"...";
    rmap->ReadRoadmapGRAPHONLY(m_inputMapFilename.c_str());
    for(RoadmapGraph<CfgType, WeightType>::VI vi = rmap->m_pRoadmap->begin(); vi != rmap->m_pRoadmap->end(); ++vi)
      VDAddNode(vi->property());
    if (m_debug) {
      cout << "\troadmap has " << rmap->m_pRoadmap->get_num_vertices() << " nodes and " << rmap->m_pRoadmap->get_num_edges() << " edges\n";
      cout << "\n\tResetting map evaluator states...\n";
    }
    for(vector<string>::iterator I = m_evaluatorLabels.begin(); I != m_evaluatorLabels.end(); ++I) {
      MapEvaluator<CfgType, WeightType>::conditional_type pEvaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*I);
      if(pEvaluator->HasState())
        pEvaluator->operator()(_regionID);
    }
    if (m_debug) cout << "\tdone.\n";
  }

  if (m_debug) cout<<"\nEnding Initializing BasicPRM"<<endl;
}

void BasicPRM::Run(int _regionID){
  if (m_debug) cout<<"\nRunning BasicPRM::"<<_regionID<<endl;

  //setup region variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);

  vector<VID> allNodesVID;
  region->GetRoadmap()->m_pRoadmap->GetVerticesVID(allNodesVID);

  m_strategyClock.StartClock("Map Generation");

  bool mapPassedEvaluation = false;
  while(!mapPassedEvaluation){
    m_currentIteration++;
    vector<VID> thisIterationNodesVID;
    if(m_startAt <= NODE_GENERATION) {
      if (m_debug) cout << "\ngenerating nodes: ";
      GenerateNodes(region, 
          back_insert_iterator<vector<VID> >(allNodesVID), 
          back_insert_iterator<vector<VID> >(thisIterationNodesVID));
    }
    else
      region->GetRoadmap()->m_pRoadmap->GetVerticesVID(thisIterationNodesVID);
    if(m_startAt <= NODE_CONNECTION) {
      if (m_debug) cout << "\nconnecting nodes: ";
      ConnectNodes(region, allNodesVID, thisIterationNodesVID);
    }
    if(m_startAt <= COMPONENT_CONNECTION) {
      if (m_debug) cout << "\nconnecting components: ";
      ConnectComponents(region);
    }
    if(m_startAt <= MAP_EVALUATION) {
      if (m_debug) cout << "\nevaluating roadmap: ";
      mapPassedEvaluation = EvaluateMap(_regionID);
    }
    m_startAt = NODE_GENERATION;
  }

  m_strategyClock.StopClock();
  if (m_debug) m_strategyClock.PrintClock();

  if (m_debug) cout<<"\nEnd Running BasicPRM::"<<_regionID<<endl;
}

void BasicPRM::Finalize(int _regionID){
  if (m_debug) cout<<"\nFinalizing BasicPRM::"<<_regionID<<endl;

  //setup region variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
  StatClass* regionStats = region->GetStatClass();

  string str;

  //output final map
  str = GetBaseFilename() + ".map";
  ofstream osMap(str.c_str());
  region->WriteRoadmapForVizmo(osMap);
  osMap.close();

  //output stats
  str = GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  streambuf* sbuf = cout.rdbuf(); // to be restored later
  cout.rdbuf(osStat.rdbuf());   // redirect destination of std::cout
  cout << "NodeGen+Connection Stats" << endl;
  regionStats->PrintAllStats(region->GetRoadmap());
  m_strategyClock.PrintClock();
  cout.rdbuf(sbuf);  // restore original stream buffer
  osStat.close();

  if (m_debug) cout<<"\nEnd Finalizing BasicPRM"<<endl;
}

void BasicPRM::ConnectNodes(MPRegion<CfgType, WeightType>* _region, 
    vector<VID>& allNodesVID, vector<VID>& thisIterationNodesVID)
{
  ClockClass nodeConnClock;
  stringstream clockName; clockName << "Iteration " << m_currentIteration << ", Node Connection";
  nodeConnClock.StartClock(clockName.str().c_str());
  stapl::vector_property_map< RoadmapGraph<CfgType, WeightType>::GRAPH,size_t > cmap;

  for(vector<string>::iterator I = m_nodeConnectionLabels.begin(); 
      I != m_nodeConnectionLabels.end(); ++I){

    ConnectMap<CfgType, WeightType>::NodeConnectionPointer pConnection;
    pConnection = GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetNodeMethod(*I);    

    ClockClass nodeConnSubClock;
    stringstream connectorClockName; connectorClockName << "Iteration " << m_currentIteration << ", " << pConnection->GetName();
    nodeConnSubClock.StartClock(connectorClockName.str().c_str());

    if(m_debug) cout << "\n\t";
    vector<VID> nodesVID(thisIterationNodesVID.begin(), thisIterationNodesVID.end());
    GetMPProblem()->GetMPStrategy()->
      GetConnectMap()->ConnectNodes(pConnection,
          _region->GetRoadmap(), *(_region->GetStatClass()),
          GetMPProblem()->GetMPStrategy()->addPartialEdge, 
          GetMPProblem()->GetMPStrategy()->addAllEdges,
          nodesVID.begin(), nodesVID.end(), 
          allNodesVID.begin(), allNodesVID.end());
    cmap.reset();
    if (m_debug) {
      cout << _region->GetRoadmap()->m_pRoadmap->get_num_edges() << " edges, " 
        << get_cc_count(*(_region->GetRoadmap()->m_pRoadmap), cmap) << " connected components"
        << endl;
      cout << "\t";
    }

    nodeConnSubClock.StopClock();
    if (m_debug) nodeConnSubClock.PrintClock();
  }
  nodeConnClock.StopClock();
  if (m_debug) nodeConnClock.PrintClock();
}

void BasicPRM::ConnectComponents(MPRegion<CfgType, WeightType>* _region) {
  ClockClass componentConnClock;
  stringstream clockName; clockName << "Iteration " << m_currentIteration << ", Component Connection";
  componentConnClock.StartClock(clockName.str().c_str());
  stapl::vector_property_map< RoadmapGraph<CfgType, WeightType>::GRAPH,size_t > cmap;

  for(vector<string>::iterator I = m_componentConnectionLabels.begin(); 
      I != m_componentConnectionLabels.end(); ++I){
    ConnectMap<CfgType, WeightType>::ComponentConnectionPointer pConnection;
    pConnection = GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetComponentMethod(*I);

    ClockClass componentConnSubClock;
    stringstream connectorClockName; connectorClockName << "Iteration " << m_currentIteration << ", " << pConnection->GetName();
    componentConnSubClock.StartClock(connectorClockName.str().c_str());

    if(m_debug) cout << "\n\t";
    GetMPProblem()->GetMPStrategy()->
      GetConnectMap()->ConnectComponents(pConnection,
          _region->GetRoadmap(), 
          *(_region->GetStatClass()),
          GetMPProblem()->GetMPStrategy()->addPartialEdge, 
          GetMPProblem()->GetMPStrategy()->addAllEdges);

    cmap.reset();
    if (m_debug){
      cout << _region->GetRoadmap()->m_pRoadmap->get_num_edges() << " edges, " 
      << get_cc_count(*(_region->GetRoadmap()->m_pRoadmap), cmap) << " connected components"<< endl;
      cout << "\t";
    }
    componentConnSubClock.StopClock();
    if (m_debug) componentConnSubClock.PrintClock();
  }
  componentConnClock.StopClock();
  if(m_debug) componentConnClock.PrintClock();
}


bool BasicPRM::EvaluateMap(int _regionID)
{
  bool mapPassedEvaluation = false;
  if(!m_evaluatorLabels.empty()){
    ClockClass evalClock;
    stringstream clockName; clockName << "Iteration " << m_currentIteration << ", Map Evaluation"; 
    evalClock.StartClock(clockName.str().c_str());

    mapPassedEvaluation = true;
    for(vector<string>::iterator I = m_evaluatorLabels.begin(); 
        I != m_evaluatorLabels.end(); ++I){
      MapEvaluator<CfgType, WeightType>::conditional_type pEvaluator;
      pEvaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*I);
      ClockClass evalSubClock;
      stringstream evaluatorClockName; evaluatorClockName << "Iteration " << m_currentIteration << ", " << pEvaluator->GetName();
      evalSubClock.StartClock(evaluatorClockName.str().c_str());

      if(m_debug) cout << "\n\t";
      mapPassedEvaluation = pEvaluator->operator()(_regionID);

      if(m_debug) cout << "\t";
      evalSubClock.StopClock();
      if(m_debug) evalSubClock.PrintClock();
      if(mapPassedEvaluation){
        if (m_debug) cout << "\t  (passed)\n";
      }
      else {
        if (m_debug) cout << "\t  (failed)\n";
      }
      if(!mapPassedEvaluation)
        break;
    }
    evalClock.StopClock();
    if(m_debug) evalClock.PrintClock();
  }
  else{mapPassedEvaluation=true;}//avoid the infinite loop
  return mapPassedEvaluation;
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

template <typename OutputIterator>
void BasicPRM::GenerateNodes(MPRegion<CfgType, WeightType>* _region, 
    OutputIterator _allOut, OutputIterator _thisIterationOut){
  ClockClass nodeGenClock;
  CDInfo cdInfo;
  StatClass * pStatClass = _region->GetStatClass();
  stringstream clockName; 
  clockName << "Iteration " << m_currentIteration << ", Node Generation"; 
  nodeGenClock.StartClock(clockName.str().c_str());
  string Callee("BasicPRM::GenerateNodes");

  typedef map<string, pair<int, int> >::iterator GIT;
  vector<CfgType> outNodes;
  if(!m_useProbability){
    for(GIT git = m_nodeGenerationLabels.begin(); git != m_nodeGenerationLabels.end(); ++git){
      Sampler<CfgType>::SamplerPointer pNodeGenerator = GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(git->first);
     if(m_debug) pNodeGenerator->PrintOptions(cout);
      vector<CfgType> inNodes(git->second.first);

      //generate nodes for this node generator method
      ClockClass nodeGenSubClock;
      stringstream generatorClockName; 
      generatorClockName << "Iteration " << m_currentIteration << ", " << git->first;
      nodeGenSubClock.StartClock(generatorClockName.str().c_str());

      if(m_debug) cout << "\n\t";

      do{
        pNodeGenerator->Sample(GetMPProblem()->GetEnvironment(),m_boundary,*pStatClass,
            inNodes.begin(),inNodes.end(),git->second.second, back_inserter(outNodes));
      }while(outNodes.size()<=0&&m_currentIteration==1);

      if (m_debug) {
        cout << _region->GetRoadmap()->m_pRoadmap->get_num_vertices() << " vertices " << endl;
        cout << "\n\t";
      }
      nodeGenSubClock.StopClock();
      if(m_debug) nodeGenSubClock.PrintClock();

    }
  }
  else{
    string NextNodeGen = PickNextSampler();
    Sampler<CfgType>::SamplerPointer pNodeGenerator; 
    pNodeGenerator = GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(NextNodeGen);
    if(m_debug) pNodeGenerator->PrintOptions(cout);
    vector<CfgType> inNodes(1);

    //generate nodes for this node generator method
    ClockClass nodeGenSubClock;
    stringstream generatorClockName; 
    generatorClockName << "Iteration " << m_currentIteration << ", " << NextNodeGen;
    nodeGenSubClock.StartClock(generatorClockName.str().c_str());

    if(m_debug) cout << "\n\t";
    pNodeGenerator->Sample(GetMPProblem()->GetEnvironment(),m_boundary,*pStatClass,inNodes.begin(),inNodes.end(),m_probGenerationLabels[NextNodeGen].second, back_inserter(outNodes));

    if (m_debug) {
      cout << _region->GetRoadmap()->m_pRoadmap->get_num_vertices() << " vertices " << endl;
      cout << "\n\t";
    }
    nodeGenSubClock.StopClock();
    if(m_debug) nodeGenSubClock.PrintClock();
  }
  //add valid nodes to roadmap
  typedef vector<CfgType>::iterator CIT;
  for(CIT cit=outNodes.begin(); cit!=outNodes.end(); ++cit){
    if(!(*cit).IsLabel("VALID")){
      !(GetMPProblem()->GetValidityChecker()->IsValid(GetMPProblem()->GetValidityChecker()->GetVCMethod(m_vcMethod),
            *cit, GetMPProblem()->GetEnvironment(), *(_region->GetStatClass()), cdInfo, true, &Callee));
    }
    if((*cit).IsLabel("VALID") && ((*cit).GetLabel("VALID"))) {
      if(!_region->GetRoadmap()->m_pRoadmap->IsVertex(*cit)) {
        VID vid = _region->GetRoadmap()->m_pRoadmap->AddVertex(*cit);
        //store value and increment iterator
        *_thisIterationOut++ = vid;
        *_allOut++ = vid;
      }
    }
  }
  nodeGenClock.StopClock();
  if(m_debug) nodeGenClock.PrintClock();
}

