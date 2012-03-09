#include "TogglePRMStrategy.h"
#include "MPRegion.h"
#include "MPStrategy.h"
#include "MapEvaluator.h"
#include "Sampler.h"
#include "boost/lambda/lambda.hpp"

TogglePRMStrategy::TogglePRMStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
  MPStrategyMethod(in_Node, in_pProblem), m_CurrentIteration(0), priority(false){
    //read input
    ParseXML(in_Node);
  }

TogglePRMStrategy::~TogglePRMStrategy(){
}

void TogglePRMStrategy::ParseXML(XMLNodeReader& in_Node) {
  for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr){
    if(citr->getName() == "node_generation_method") {
      string generationMethod = citr->stringXMLParameter("Method", true, "", "Node Connection Method");
      int numPerIter = citr->numberXMLParameter("Number", true, 1, 0, MAX_INT, "Number of samples");
      int attemptsPerIter = citr->numberXMLParameter("Attempts", false, 1, 0, MAX_INT, "Number of attempts");
      m_NodeGenerationLabels[generationMethod] = make_pair(numPerIter, attemptsPerIter);
      citr->warnUnrequestedAttributes();
    } 
    else if(citr->getName() == "node_connection_method"){
      string connectMethod = citr->stringXMLParameter("Method", true, "", "Node Connection Method");
      m_NodeConnectionLabels.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    } 
    /*else if(citr->getName() == "component_connection_method"){
      string connectMethod = citr->stringXMLParameter("Method", true, "", "Component Connection Method");
      m_ComponentConnectionLabels.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    } */
    else if(citr->getName() == "col_node_connection_method"){
      string connectMethod = citr->stringXMLParameter("Method", true, "", "Node Connection Method");
      m_ColNodeConnectionLabels.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    } 
    /*else if(citr->getName() == "col_component_connection_method"){
      string connectMethod = citr->stringXMLParameter("Method", true, "", "Component Connection Method");
      m_ColComponentConnectionLabels.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    } */
    else if(citr->getName() == "evaluation_method"){
      string evalMethod = citr->stringXMLParameter("Method", true, "", "Evaluation Method");
      m_EvaluatorLabels.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName()=="vc_method"){
      vcMethod =citr->stringXMLParameter("Method", true, "", "ValidityCheckerMethod");
      citr->warnUnrequestedAttributes();
    }
    else
      citr->warnUnknownNode();
  }

  priority = in_Node.boolXMLParameter("priority", false, false, "Priority Queue");
  in_Node.warnUnrequestedAttributes();

  PrintOptions(cout);
}

void TogglePRMStrategy::PrintOptions(ostream& out_os) {
  using boost::lambda::_1;
  out_os << "\nTogglePRMStrategy::ParseXML:\n";
  out_os << "\tnode_generation_methods: "; 
  for(map<string, pair<int, int> >::iterator I = m_NodeGenerationLabels.begin();I!=m_NodeGenerationLabels.end();I++) 
    out_os<<I->first<<" Number:"<<I->second.first<<" Attempts:"<<I->second.second<<endl; 
  out_os << endl;
  out_os << endl;
  out_os << "\tnode_connection_methods: "; 
  for_each(m_NodeConnectionLabels.begin(), m_NodeConnectionLabels.end(), out_os << _1 << " "); 
  out_os << endl;
  out_os << "\tcol_node_connection_methods: "; 
  for_each(m_ColNodeConnectionLabels.begin(), m_ColNodeConnectionLabels.end(), out_os << _1 << " "); 
  out_os << endl;
  out_os << "\tevaluator_methods: "; 
  for_each(m_EvaluatorLabels.begin(), m_EvaluatorLabels.end(), out_os << _1 << " "); 
  out_os << endl;
  out_os << "\tvcMethod: " << vcMethod;
  out_os << "\tpriority: " << priority;
  out_os << endl;
}

void TogglePRMStrategy::Initialize(int in_RegionID){
  cout<<"\nInitializing TogglePRMStrategy::"<<in_RegionID<<endl;

  cout<<"\nEnding Initializing TogglePRMStrategy"<<endl;
}

void TogglePRMStrategy::Run(int in_RegionID){
  cout<<"\nRunning TogglePRMStrategy::"<<in_RegionID<<endl;

  //setup region variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
  StatClass* stats = region->GetStatClass();

  vector<VID> allNodesVID, allCollisionNodesVID;
  region->GetRoadmap()->m_pRoadmap->GetVerticesVID(allNodesVID);
  vector<VID> thisIterationNodesVID, thisIterationCollisionNodesVID;

  deque<pair<string, CfgType> > queue;

  stats->StartClock("Map Generation");

  bool done=false;

  while(!EvaluateMap(in_RegionID)&&!done){
    m_CurrentIteration++;
    cout << "\ngenerating nodes: ";
    GenerateNodes(region, 
        back_insert_iterator<vector<VID> >(allNodesVID), 
        back_insert_iterator<vector<VID> >(thisIterationNodesVID),
        back_insert_iterator<vector<VID> >(allCollisionNodesVID),
        back_insert_iterator<vector<VID> >(thisIterationCollisionNodesVID),
        queue);

    while(!EvaluateMap(in_RegionID) && queue.size()>0 && !done){
      //if(region->GetRoadmap()->m_pRoadmap->get_num_vertices()+
        //region->GetBlockRoadmap()->m_pRoadmap->get_num_vertices()>=1000){done=true;continue;}
      pair<string, CfgType> p = queue.front();
      queue.pop_front();
      string validity = p.first;
      CfgType cfg = p.second;
      cout<<"validity - " << validity<<endl;
      if(validity=="valid"){
        VID vid = region->GetRoadmap()->m_pRoadmap->AddVertex(cfg);
        allNodesVID.push_back(vid);
        thisIterationNodesVID.push_back(vid);
        Connect(region, make_pair("valid", vid), allNodesVID, allNodesVID, allCollisionNodesVID, queue);
      }
      else if(validity=="invalid"){
        VID vid = region->GetBlockRoadmap()->m_pRoadmap->AddVertex(cfg);
        allCollisionNodesVID.push_back(vid);
        thisIterationCollisionNodesVID.push_back(vid);
        GetMPProblem()->GetValidityChecker()->ToggleValidity();
        Connect(region, make_pair("invalid", vid), allCollisionNodesVID, allNodesVID, allCollisionNodesVID, queue);
        GetMPProblem()->GetValidityChecker()->ToggleValidity();
      }
    }
  }

  stats->StopPrintClock("Map Generation", cout);

  cout<<"\nEnd Running TogglePRMStrategy::"<<in_RegionID<<endl;
}

void TogglePRMStrategy::Finalize(int in_RegionID){
  cout<<"\nFinalizing TogglePRMStrategy::"<<in_RegionID<<endl;

  //setup region variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
  StatClass* regionStats = region->GetStatClass();

  string str;

  //output final map
  str = GetBaseFilename() + ".map";
  ofstream osMap(str.c_str());
  region->WriteRoadmapForVizmo(osMap);
  osMap.close();

  str = GetBaseFilename() + ".block.map";
  ofstream osMap2(str.c_str());
  region->WriteRoadmapForVizmo(osMap2, NULL, true);
  osMap2.close();

  //output stats
  str = GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  osStat << "NodeGen+Connection Stats" << endl;
  regionStats->PrintAllStats(osStat, region->GetRoadmap());
  regionStats->PrintClock("Map Generation", osStat);
  osStat.close();

  cout<<"\nEnd Finalizing TogglePRMStrategy"<<endl;
}

template <typename OutputIterator>
void TogglePRMStrategy::GenerateNodes(MPRegion<CfgType, WeightType>* region, 
    OutputIterator allOut, OutputIterator thisIterationOut,
    OutputIterator allCollisionOut, OutputIterator thisIterationCollisionOut,
    deque<pair<string, CfgType> >& queue){
  CDInfo cdInfo;
  StatClass * pStatClass = region->GetStatClass();
  stringstream clockName; 
  clockName << "Iteration " << m_CurrentIteration << ", Node Generation"; 
  pStatClass->StartClock(clockName.str());

  typedef map<string, pair<int,int> >::iterator GIT;
  for(GIT git = m_NodeGenerationLabels.begin(); git != m_NodeGenerationLabels.end(); ++git){
    Sampler<CfgType>::SamplerPointer pNodeGenerator = GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(git->first);
    vector<CfgType> outNodes, outCollisionNodes;
    vector<CfgType> inNodes(git->second.first);

    string Callee = "TogglePRM::GenerateNodes";
    //generate nodes for this node generator method
    stringstream generatorClockName; 
    generatorClockName << "Iteration " << m_CurrentIteration << ", " << git->first;
    pStatClass->StartClock(generatorClockName.str());

    cout << "\n\t";

    do{
      pNodeGenerator->Sample(GetMPProblem()->GetEnvironment(),*pStatClass,
          inNodes.begin(),inNodes.end(),git->second.second,
          back_inserter(outNodes), back_inserter(outCollisionNodes));
    }while(outNodes.size()<=0&&m_CurrentIteration==1);

    cout << region->GetRoadmap()->m_pRoadmap->get_num_vertices() << " vertices " << endl;
    cout << "\n\t";
    pStatClass->StopPrintClock(generatorClockName.str(), cout);

    //add nodes to queue
    typedef vector<CfgType>::iterator CIT;
    for(CIT cit=outNodes.begin(); cit!=outNodes.end(); ++cit){
      if(!(*cit).IsLabel("VALID")){
        !(GetMPProblem()->GetValidityChecker()->IsValid(GetMPProblem()->GetValidityChecker()->GetVCMethod(vcMethod), 
              *cit, GetMPProblem()->GetEnvironment(), *(region->GetStatClass()), cdInfo, true, &Callee));
      }
      //out nodes mean valid then add them to the real roadmap
      if((*cit).IsLabel("VALID") && ((*cit).GetLabel("VALID"))) {
        if(priority)
          queue.push_front(make_pair("valid", *cit));
        else
          queue.push_back(make_pair("valid", *cit));
      }
      //else invalid add to block map
      else{
        queue.push_back(make_pair("invalid", *cit));
      }
    }
    for(CIT cit=outCollisionNodes.begin(); cit!=outCollisionNodes.end(); ++cit){
      if(!(*cit).IsLabel("VALID")){
        !(GetMPProblem()->GetValidityChecker()->IsValid(GetMPProblem()->GetValidityChecker()->GetVCMethod(vcMethod), 
              *cit, GetMPProblem()->GetEnvironment(), *(region->GetStatClass()), cdInfo, true, &Callee));
      }
      //outCollisionNodes mean INVALID then add to block map
      if((*cit).IsLabel("VALID") && !((*cit).GetLabel("VALID"))) {
        queue.push_back(make_pair("invalid", *cit));
      }
      //else valid add to real map
      else{
        if(priority)
          queue.push_front(make_pair("valid", *cit));
        else
          queue.push_back(make_pair("valid", *cit));
      }  
    }
  }
 pStatClass->StopPrintClock(clockName.str(), cout);
}

void TogglePRMStrategy::Connect(MPRegion<CfgType, WeightType>* region, pair<string, VID> pvid, 
    vector<VID>& allVID, vector<VID>& allNodesVID, vector<VID>&
    allCollisionNodesVID, deque<pair<string, CfgType> >& queue){
  StatClass* pStatClass = region->GetStatClass();
  stringstream clockName; clockName << "Iteration " << m_CurrentIteration << ", Node Connection";
  pStatClass->StartClock(clockName.str());
  stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
  vector<string> NodeConnectionLabels;
  if(pvid.first=="valid")
    NodeConnectionLabels = m_NodeConnectionLabels;
  else
    NodeConnectionLabels = m_ColNodeConnectionLabels;
  for(vector<string>::iterator I = NodeConnectionLabels.begin(); I != NodeConnectionLabels.end(); ++I){

    Connector<CfgType, WeightType>::ConnectionPointer pConnection;
    pConnection = GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*I);    

    stringstream connectorClockName; 
    connectorClockName<<"Iteration "<<m_CurrentIteration<<", "<<pConnection->GetName();
    pStatClass->StartClock(connectorClockName.str());

    cout << "\n\t";
    vector<VID> nodesVID;
    nodesVID.push_back(pvid.second);
    vector<CfgType> collision, valid;
    if(pvid.first=="valid")
      GetMPProblem()->GetMPStrategy()->GetConnector()->Connect(pConnection,
          region->GetRoadmap(), *(region->GetStatClass()),
          nodesVID.begin(), nodesVID.end(), 
          allVID.begin(), allVID.end(),
          back_inserter(collision));
    else
      GetMPProblem()->GetMPStrategy()->GetConnector()->Connect(pConnection,
          region->GetBlockRoadmap(), *(region->GetStatClass()),
          nodesVID.begin(), nodesVID.end(), 
          allVID.begin(), allVID.end(),
          back_inserter(collision));

    cout<<"\nCollision Nodes from connecting::"<<collision.size()<<endl;
    cout<<"Adding "<<collision.size()<<" collision nodes"<<endl;
    typedef vector<CfgType>::iterator CIT;
    for(CIT cit=collision.begin(); cit!=collision.end(); ++cit){
      if(cit->IsLabel("VALID") && cit->GetLabel("VALID")){
        if(priority)
          queue.push_front(make_pair("valid", *cit));
        else
          queue.push_back(make_pair("valid", *cit));
      }
      else if(cit->IsLabel("VALID") && !cit->GetLabel("VALID")) {
        queue.push_back(make_pair("invalid", *cit));
      }
    }
    cmap.reset();
    cout << region->GetRoadmap()->m_pRoadmap->get_num_edges() << " edges, " 
      << get_cc_count(*(region->GetRoadmap()->m_pRoadmap), cmap) << " connected components"
      << endl;

    cout << "\t";
    pStatClass->StopPrintClock(connectorClockName.str(), cout);
  }
  pStatClass->StopPrintClock(clockName.str(), cout);
}

bool TogglePRMStrategy::EvaluateMap(int in_RegionID)
{
  bool mapPassedEvaluation = false;
  if(!m_EvaluatorLabels.empty()){
    stringstream clockName; clockName << "Iteration " << m_CurrentIteration << ", Map Evaluation";
   StatClass* stats = GetMPProblem()->GetMPRegion(in_RegionID)->GetStatClass();
   stats->StartClock(clockName.str());

    mapPassedEvaluation = true;
    for(vector<string>::iterator I = m_EvaluatorLabels.begin(); I != m_EvaluatorLabels.end(); ++I){
      MapEvaluator<CfgType, WeightType>::MapEvaluationMethodPtr pEvaluator;
      pEvaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*I);
      stringstream evaluatorClockName; 
      evaluatorClockName << "Iteration " << m_CurrentIteration << ", " << pEvaluator->GetName();
      stats->StartClock(evaluatorClockName.str());

      cout << "\n\t";
      mapPassedEvaluation = pEvaluator->operator()(in_RegionID);

      cout << "\t";
      stats->StopPrintClock(evaluatorClockName.str(), cout);
      if(mapPassedEvaluation){
        return true;
        cout << "\t  (passed)\n";
      }
      else
        cout << "\t  (failed)\n";
      //if(!mapPassedEvaluation)
        //break;
    }
    stats->StopPrintClock(clockName.str(), cout);
  }
  else{mapPassedEvaluation=true;}//avoid the infinite loop
  return mapPassedEvaluation;
}

