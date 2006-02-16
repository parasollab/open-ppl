
#include <iostream>
#include <sstream>
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


#include "MPRegion.h"
#include "MPProblem.h"
#include "MPStrategy.h"


void parse_unknown_tag(TiXmlNode* in_pNode)
{
  if(!in_pNode) {
    cout << "Error -1" << endl; exit(-1);
  }

  cout << "I don't know: " << *in_pNode << endl;
}

