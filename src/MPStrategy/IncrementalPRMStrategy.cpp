#include "MPStrategy/IncrementalPRMStrategy.h"
#include "MPProblem/MPRegion.h"
#include "NodeGenerator/NodeGeneratorMethod.h"
#include "MPStrategy/MPStrategy.h"

IncrementalPRMStrategy::
IncrementalPRMStrategy(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
  MPStrategyMethod(in_Node, in_pProblem),
  m_current_iteration(0)
{
  //read input
  ParseXML(in_Node);
}


IncrementalPRMStrategy::
~IncrementalPRMStrategy() 
{
}


#include "boost/lambda/lambda.hpp"

void 
IncrementalPRMStrategy::
ParseXML(XMLNodeReader& in_Node) 
{
  for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr)
  {
    if(citr->getName() == "node_generation_method") 
    {
      string generation_method = citr->stringXMLParameter(string("Method"), true, string(""), string("Node Connection Method"));
      m_vecStrNodeGenerationLabels.push_back(generation_method);
      citr->warnUnrequestedAttributes();
    } else if(citr->getName() == "node_connection_method")
    {
      string connect_method = citr->stringXMLParameter(string("Method"), true, string(""), string("Node Connection Method"));
      m_vecStrNodeConnectionLabels.push_back(connect_method);
      citr->warnUnrequestedAttributes();
    } else if(citr->getName() == "component_connection_method")
    {
      string connect_method = citr->stringXMLParameter(string("Method"), true, string(""), string("Node Connection Method"));
      m_vecStrComponentConnectionLabels.push_back(connect_method);
      citr->warnUnrequestedAttributes();
    } else if(citr->getName() == "evaluation_method")
    {
      string eval_method = citr->stringXMLParameter(string("Method"), true, string(""), string("Node Connection Method"));
      m_vecStrEvaluatorLabels.push_back(eval_method);
      citr->warnUnrequestedAttributes();
    } else
      citr->warnUnknownNode();
  }

  //set m_strBaseFilename
  stringstream seed_stream;
  seed_stream << getSeed();
  string seed_str;
  seed_stream >> seed_str;
  m_strBaseFilename = getBaseFilename() + "." + seed_str;
  
  //output for debugging
  using boost::lambda::_1;
  cout << "\nIncrementalPRMStrategy::ParseXML:\n";
  cout << "\tbase_filename: " << m_strBaseFilename << endl;
  cout << "\tnode_generation_methods: "; for_each(m_vecStrNodeGenerationLabels.begin(), m_vecStrNodeGenerationLabels.end(), cout << _1 << " "); cout << endl;
  cout << "\tnode_connection_methods: "; for_each(m_vecStrNodeConnectionLabels.begin(), m_vecStrNodeConnectionLabels.end(), cout << _1 << " "); cout << endl;
  cout << "\tcomponent_connection_methods: "; for_each(m_vecStrComponentConnectionLabels.begin(), m_vecStrComponentConnectionLabels.end(), cout << _1 << " "); cout << endl;
  cout << "\tevaluator_methods: "; for_each(m_vecStrEvaluatorLabels.begin(), m_vecStrEvaluatorLabels.end(), cout << _1 << " "); cout << endl;
  cout << endl;
}


void 
IncrementalPRMStrategy::
PrintOptions(ostream& out_os) 
{
  out_os << "IncrementalPRMStrategy::PrintOptions: not implemented, nothing done\n";
}


void 
IncrementalPRMStrategy::
operator()(int in_RegionID)
{
  cout << "\n\nBeginning IncrementalPRMStratgy::operator(" << in_RegionID << ")\n";
  
  //seed random number generator
  OBPRM_srand(getSeed()); 
  
  //setup region variables
  MPRegion<CfgType,WeightType>* region = GetMPProblem()->GetMPRegion(in_RegionID);
  Stat_Class* region_stats = region->GetStatClass();
  
  vector<VID> all_nodes_VID;
  region->GetRoadmap()->m_pRoadmap->GetVerticesVID(all_nodes_VID);
  
  Clock_Class MapGenClock;
  MapGenClock.StartClock("Map Generation");
  
  bool map_passed_evaluation = false;
  do {
    vector<VID> this_iteration_nodes_VID;
    cout << "\ngenerating nodes: ";
    generate_nodes(region, 
                   back_insert_iterator<vector<VID> >(all_nodes_VID), 
                   back_insert_iterator<vector<VID> >(this_iteration_nodes_VID));
    cout << "\nconnecting nodes: ";
    connect_nodes(region, all_nodes_VID, this_iteration_nodes_VID);
    connect_components(region);
    cout << "\nevaluating roadmap: ";
    map_passed_evaluation = evaluate_map(in_RegionID);
  } while(++m_current_iteration < m_iterations && !map_passed_evaluation);
  
  MapGenClock.StopPrintClock();
  
  string str;
  
  //output final map
  str = m_strBaseFilename + ".map";
  ofstream os_map(str.c_str());
  region->WriteRoadmapForVizmo(os_map);
  os_map.close();
  
  //output stats
  str = m_strBaseFilename + ".stat";
  ofstream  os_stat(str.c_str());
  streambuf* sbuf = cout.rdbuf(); // to be restored later
  cout.rdbuf(os_stat.rdbuf());   // redirect destination of std::cout
  cout << "NodeGen+Connection Stats" << endl;
  region_stats->PrintAllStats(region->GetRoadmap());
  MapGenClock.PrintClock();
  //region_stats->PrintFeatures();
  std::cout.rdbuf(sbuf);  // restore original stream buffer
  os_stat.close();

  cout << "Ending IncrementalPRMStratgy::operator(" << in_RegionID << ")\n\n";
}


void
IncrementalPRMStrategy::
connect_nodes(MPRegion<CfgType, WeightType>* region, vector<VID>& all_nodes_VID, vector<VID>& this_iteration_nodes_VID)
{
  Clock_Class NodeConnClock;
  stringstream clock_name; clock_name << "Iteration " << m_current_iteration << ", Node Connection";
  NodeConnClock.StartClock(clock_name.str().c_str());
  
  for(vector<string>::iterator I = m_vecStrNodeConnectionLabels.begin(); I != m_vecStrNodeConnectionLabels.end(); ++I) //SLT:: should be const_iterator
  {
    NodeConnectionMethod<CfgType, WeightType>* pNodeConnector = GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetNodeMethod(*I);
        
    Clock_Class NodeConnSubClock;
    stringstream connector_clock_name; connector_clock_name << "Iteration " << m_current_iteration << ", " << pNodeConnector->GetName();
    NodeConnSubClock.StartClock(connector_clock_name.str().c_str());
    
    cout << "\n\t";
    vector<VID> nodes_VID(this_iteration_nodes_VID.begin(), this_iteration_nodes_VID.end());
    pNodeConnector->Connect(region->GetRoadmap(), 
                            *(region->GetStatClass()),
                //            GetMPProblem()->GetCollisionDetection(),
                            GetMPProblem()->GetDistanceMetric(), 
                            GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                            GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                            GetMPProblem()->GetMPStrategy()->addAllEdges,
                            nodes_VID, 
                            all_nodes_VID);
    cout << region->GetRoadmap()->m_pRoadmap->GetEdgeCount() << " edges, " 
         << GetCCcount(*(region->GetRoadmap()->m_pRoadmap)) << " connected components"
         << endl;
    
    cout << "\t";
    NodeConnSubClock.StopPrintClock();
  }
  NodeConnClock.StopPrintClock();
}


void
IncrementalPRMStrategy::
connect_components(MPRegion<CfgType, WeightType>* region)
{
  Clock_Class ComponentConnClock;
  stringstream clock_name; clock_name << "Iteration " << m_current_iteration << ", Component Connection";
  ComponentConnClock.StartClock(clock_name.str().c_str());
  
  for(vector<string>::iterator I = m_vecStrComponentConnectionLabels.begin(); I != m_vecStrComponentConnectionLabels.end(); ++I) //SLT:: should be const_iterator
  {
    ComponentConnectionMethod<CfgType, WeightType>* pComponentConnector = GetMPProblem()->GetMPStrategy()->GetConnectMap()->GetComponentMethod(*I);
    
    Clock_Class ComponentConnSubClock;
    stringstream connector_clock_name; connector_clock_name << "Iteration " << m_current_iteration << ", " << pComponentConnector->GetName();
    ComponentConnSubClock.StartClock(connector_clock_name.str().c_str());
    
    cout << "\n\t";
    pComponentConnector->Connect(region->GetRoadmap(), 
                                 *(region->GetStatClass()), //*region_stats,
//                                 GetMPProblem()->GetCollisionDetection(),
                                 GetMPProblem()->GetDistanceMetric(), 
                                 GetMPProblem()->GetMPStrategy()->GetLocalPlanners(),
                                 GetMPProblem()->GetMPStrategy()->addPartialEdge, 
                                 GetMPProblem()->GetMPStrategy()->addAllEdges);
    cout << region->GetRoadmap()->m_pRoadmap->GetEdgeCount() << " edges, " 
         << GetCCcount(*(region->GetRoadmap()->m_pRoadmap)) << " connected components"
         << endl;
    
    cout << "\t";
    ComponentConnSubClock.StopPrintClock();
  }
  ComponentConnClock.StopPrintClock();
}


bool
IncrementalPRMStrategy::
evaluate_map(int in_RegionID)
{
  bool map_passed_evaluation = false;
  if(!m_vecStrEvaluatorLabels.empty())
  {
    Clock_Class EvalClock;
    stringstream clock_name; clock_name << "Iteration " << m_current_iteration << ", Map Evaluation"; 
    EvalClock.StartClock(clock_name.str().c_str());
    
    map_passed_evaluation = true;
    for(vector<string>::iterator I = m_vecStrEvaluatorLabels.begin(); I != m_vecStrEvaluatorLabels.end(); ++I) //SLT: should be const_iterator
    {
      MapEvaluator<CfgType, WeightType>::conditional_type pEvaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetConditionalMethod(*I);
      Clock_Class EvalSubClock;
      stringstream evaluator_clock_name; evaluator_clock_name << "Iteration " << m_current_iteration << ", " << pEvaluator->GetName();
      EvalSubClock.StartClock(evaluator_clock_name.str().c_str());
      
      cout << "\n\t";
      map_passed_evaluation = pEvaluator->operator()(in_RegionID);
     
      cout << "\t";
      EvalSubClock.StopPrintClock();
      if(map_passed_evaluation)
        cout << "\t  (passed)\n";
      else
        cout << "\t  (failed)\n";
      if(!map_passed_evaluation)
        break;
    }
    EvalClock.StopPrintClock();
  }
  return map_passed_evaluation;
}

