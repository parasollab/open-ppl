#ifndef MPStrategy_h
#define MPStrategy_h


#include "tinyxml.h"

#include "SwitchDefines.h"
#include<sys/time.h>

#include "OBPRMDef.h"
#include "Roadmap.h"
#include "Input.h"

#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "LocalPlanners.h"
#include "GenerateMapNodes.h"

#include "GeneratePartitions.h"

//#include "ExplicitInstantiation.h"

/* util.h defines EXIT used in initializing the environment*/
#include "util.h"
#include "MPProblem.h"

class MPStrategy_method 
{
  //Will be used to derive IMP,PRM,RRT,metaplanner, etc.
};

class MPStrategy
{
public: 
  MPStrategy(TiXmlNode* in_pNode, MPProblem* in_pproblem) {
    m_pproblem = in_pproblem;
    addPartialEdge= true;
    addAllEdges=true;
    if(!in_pNode) {
      cout << "Error -1" << endl; exit(-1);
    }
    if(string(in_pNode->Value()) != "MPStrategy") {
      cout << "Error reading <MPStrategy> tag...." << endl; exit(-1);
    }

    cout << "I will parse MPStrategy" << endl;
    
    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
      if(string(pChild->Value()) == "node_generation_methods") {
        //cout << "I will parse a <node_generation_methods>" << endl;
        gn_map = new GenerateMapNodes<CfgType>(pChild);
      } else if(string(pChild->Value()) == "connection_methods") {
       // cout << "I will parse a <connection_methods>" << endl;
        cm_map = new ConnectMap<CfgType, WeightType>(pChild);
      } else if(string(pChild->Value()) == "lp_methods") {
        lp_map = new LocalPlanners<CfgType, WeightType>(pChild);
      } else {
        cout << "  I don't know: " << *pChild << endl;
      }
    }
  }

  
void GenerateMap(){
  cout << "MPStrategy::GenerateMap()" << endl;
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
  gn_map->GenerateNodes(&m_pproblem->rmp,Stats,m_pproblem->cd,m_pproblem->dm,nodes);
  cout << "Finished Generate Nodes" << endl;
  NodeGenClock.StopClock();


  //---------------------------
  // Connect roadmap nodes
  //---------------------------
  ConnectionClock.StartClock("Node Connection");
  cout << "Starting connect" << endl;
  cm_map->Connect(&m_pproblem->rmp, Stats, m_pproblem->cd, 
                  m_pproblem->dm, lp_map,
                  addPartialEdge, addAllEdges);
  cout << "Finished connect" << endl;
  ConnectionClock.StopClock();
  
  Input* input = new Input;
  cout << "Writing roadmap" << endl;

  char env[100];
  char cmd[100];
  strcpy(env,m_pproblem->m_input_env.c_str());
  input->envFile.SetValue(env);
  strcpy(cmd,"../obprm -f");
  cout << "setting envfile to: " << env << endl;
  strcpy(input->commandLine,cmd);

  m_pproblem->rmp.WriteRoadmap(input,m_pproblem->cd,m_pproblem->dm,
                               lp_map,"rogertest.map");
  cout << "Finished writting roadmap" << endl;
} 
  
  
  ///@ToDo Move addPartialEdge, addAllEdges to ConnectMap
 private:
   MPProblem* m_pproblem;
  GenerateMapNodes<CfgType>* gn_map;
  ConnectMap<CfgType, WeightType>* cm_map;
  LocalPlanners<CfgType, WeightType>* lp_map;
  bool addPartialEdge, addAllEdges; //move to connect map class
  //Map_Evaluation
  //Filtering
  //vector< MPStrategy_method >
  //MPStrategy_method* selected method (only 1)
};
#endif
