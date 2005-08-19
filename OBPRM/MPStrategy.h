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


class MPStrategy_method 
{
  //Will be used to derive IMP,PRM,RRT,metaplanner, etc.
};

class MPstrategy
{
public: 
  MPstrategy(TiXmlNode* in_pNode) {
    if(!in_pNode) {
      cout << "Error -1" << endl; exit(-1);
    }
    if(string(in_pNode->Value()) != "MPstrategy") {
      cout << "Error reading <MPstrategy> tag...." << endl; exit(-1);
    }

    cout << "I will parse MPstrategy" << endl;
    
    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
      if(string(pChild->Value()) == "feature_sensitive_metaplanner") {
	//    MetaPlanner meta_planner(pChild);
      } else {
        cout << "  I don't know: " << *pChild << endl;
      }
    }
  }

  ///@ToDo Move addPartialEdge, addAllEdges to ConnectMap
 private:
  GenerateMapNodes<CfgType> gn_map;
  ConnectMap<CfgType, WeightType> cm_map;
  LocalPlanners<CfgType, WeightType> lp_map;
  bool addPartialEdge, addAllEdges; //move to connect map class
  //Map_Evaluation
  //Filtering
  //vector< MPStrategy_method >
  //MPStrategy_method* selected method (only 1)
};
