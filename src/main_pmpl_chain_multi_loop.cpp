/** @file main_pmp.cpp
 * @brief main function to use the feature sensitive meta-planner
 */

#include <iostream>
#include <sstream>
#include "SwitchDefines.h"
#include<sys/time.h>

#include "OBPRMDef.h"
#include "Roadmap.h"

#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ValidityChecker.hpp"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "LocalPlanners.h"
#include "GenerateMapNodes.h"

#include "GeneratePartitions.h"

//#include "ExplicitInstantiation.h"

/* util.h defines PMPL_EXIT used in initializing the environment*/
#include "util.h"


#include "MPRegion.h"
#include "MPProblem.h"
#include "ClosedChainProblem.h"
#include "MPStrategy.h"
#include "ClosedChainStrategy.h"

#include "CfgTypes.h"

#include "Weight.h"


using namespace std;


//========================================================================
//  main
//========================================================================



int main(int argc, char** argv)
{  
  if(argc < 3) { cout << "Usage ... -f options.xml" << endl; exit(-1);}
  
  if(!(string(argv[1]) == "-f"))
  { cout << "Usage ... -f options.xml" << endl; exit(-1);}
  
  TiXmlDocument doc( argv[2] );
  bool loadOkay = doc.LoadFile();

  if ( !loadOkay ) {
    cout << "Could not load test file " << string(argv[2]) << ". Error=" << doc.ErrorDesc() <<". Exiting.\n";
    exit( 1 );
  }

  XMLNodeReader mp_node(std::string(argv[2]),doc, string("motion_planning"));

  //ClosedChainProblem* problem;
  ClosedChainProblem* problem;
  ClosedChainStrategy* strategy;
  //Iterate over child nodes
  cout<<"before loop"<<endl;
  for(XMLNodeReader::childiterator citr = mp_node.children_begin(); citr != mp_node.children_end(); ++citr) {
    if(citr->getName() == "MPProblem") {
      problem = new ClosedChainProblem(*citr);
    } else if(citr->getName() == "MPStrategy") {
      strategy = new ClosedChainStrategy(*citr,problem);
    }
    else {
      citr->warnUnknownNode();
    } 
  }
  problem->SetMPStrategy(strategy);
  if(problem != NULL) {
    cout<<"********* printing problem"<<endl;
    problem->PrintOptions(cout);
  } else {
    cerr << "I don't have a MPProblem" << endl << flush;
  }
  
  //Start generating!
  if(strategy != NULL) {
    cout<<"before solve"<<endl;
    strategy->Solve();
  } else {
    cerr << "I don't have a MPStrategy" << endl << flush;
  }
    return 0;
}


