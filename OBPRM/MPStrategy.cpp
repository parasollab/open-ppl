#include "MPStrategy.h"


MPStrategy::
MPStrategy(TiXmlNode* in_pNode, MPProblem* in_pproblem) {

    LOG_MSG( "MPStrategy::MPStrategy()" , DEBUG_MSG);
    m_pProblem = in_pproblem;
    addPartialEdge= true;
    addAllEdges=true;
    if(!in_pNode) {
      LOG_MSG("MPStrategy::MPStrategy() error xml input",ERROR_MSG); exit(-1);
    }
    if(string(in_pNode->Value()) != "MPStrategy") {
      LOG_MSG("MPStrategy::MPStrategy() error xml input",ERROR_MSG); exit(-1);
    }
 
    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
      if(string(pChild->Value()) == "node_generation_methods") {
        m_pNodeGeneration = new GenerateMapNodes<CfgType>(pChild);
      } else if(string(pChild->Value()) == "connection_methods") {
        m_pConnection = new ConnectMap<CfgType, WeightType>(pChild);
      } else if(string(pChild->Value()) == "lp_methods") {
        m_pLocalPlanners = new LocalPlanners<CfgType, WeightType>(pChild);
      } else {
        LOG_MSG("MPStrategy::  I don't know: "<< endl << *pChild,WARNING_MSG);
      }
    }
    LOG_MSG( "~MPStrategy::MPStrategy()" , DEBUG_MSG);
}


void MPStrategy::
GenerateMap() {
  LOG_MSG("MPStrategy::GenerateMap()",DEBUG_MSG);
  Stat_Class Stats;
  
  Clock_Class        NodeGenClock;
  Clock_Class        ConnectionClock;
  vector<CfgType> nodes;
  nodes.erase(nodes.begin(),nodes.end());

  //---------------------------
  // Generate roadmap nodes
  //---------------------------
  NodeGenClock.StartClock("Node Generation");
  cout << "Starting Generate Nodes" << endl;
  m_pNodeGeneration->GenerateNodes(m_pProblem->GetRoadmap(),Stats,
                                    m_pProblem->GetCollisionDetection(),
                                    m_pProblem->GetDistanceMetric(),nodes);
  cout << "Finished Generate Nodes" << endl;
  NodeGenClock.StopClock();


  //---------------------------
  // Connect roadmap nodes
  //---------------------------
  ConnectionClock.StartClock("Node Connection");
  cout << "Starting connect" << endl;
  m_pConnection->Connect(m_pProblem->GetRoadmap(), Stats, 
                          m_pProblem->GetCollisionDetection(),
                          m_pProblem->GetDistanceMetric(), 
                          m_pLocalPlanners,
                          addPartialEdge, addAllEdges);
  cout << "Finished connect" << endl;
  ConnectionClock.StopClock();
  /*
  Input* input = new Input;
  cout << "Writing roadmap" << endl;

  char env[100];
  char cmd[100];
  strcpy(env,m_pProblem->GetEnvFileName().c_str());
  input->envFile.SetValue(env);
  strcpy(cmd,"../obprm -f");
  cout << "setting envfile to: " << env << endl;
  strcpy(input->commandLine,cmd);
  */
  /*
  m_pProblem->GetRoadmap()->WriteRoadmap(input,m_pProblem->GetCollisionDetection(),
                               m_pProblem->GetDistanceMetric(), 
                               m_pLocalPlanners,
                               m_pProblem->GetOutputRoadmap());
  */
  m_pProblem->WriteRoadmapForVizmo();
  
  //delete input;
  cout << "Finished writting roadmap" << endl;
  LOG_MSG("~MPStrategy::GenerateMap()",DEBUG_MSG);
} 
