/** @file main_pmp.cpp
 * @brief main function to use the feature sensitive meta-planner
 */

#include <iostream>
#include <sstream>
#include<sys/time.h>

#include "Roadmap.h"

#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ValidityChecker.hpp"
#include "ConnectMap.h"
#include "LocalPlanners.h"

#include "GeneratePartitions.h"

//#include "ExplicitInstantiation.h"

/* util.h defines PMPL_EXIT used in initializing the environment*/
#include "util.h"


#include "MPRegion.h"
#include "ClosedChainProblem.h"
#include "ClosedChainStrategy.h"



#include "CfgTypes.h"

#include "Weight.h"


using namespace std;


//========================================================================
//  main
//========================================================================



#ifdef _PARALLEL
void stapl_main(int argc, char *argv[])
#else 
int main(int argc, char** argv)
#endif
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

  MPProblem* problem;
  MPStrategy* strategy;
  //Iterate over child nodes
  
  for(XMLNodeReader::childiterator citr = mp_node.children_begin(); citr != mp_node.children_end(); ++citr) {
    if(citr->getName() == "MPProblem") {
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
          problem = new ClosedChainProblem(*citr);
#else
          problem = new MPProblem(*citr);
#endif
    } else if(citr->getName() == "MPStrategy") {
#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
        strategy = new ClosedChainStrategy(*citr,(ClosedChainProblem*)problem);
#else
        strategy = new MPStrategy(*citr,problem);
	
#endif
        problem->SetMPStrategy(strategy);
    }
    else {
      citr->warnUnknownNode();
    } 
  }
  
  //Output whats about to go on
  if(problem != NULL) {
    problem->PrintOptions(cout);
  } else {
    cerr << "I don't have a MPProblem" << endl << flush;
  }
  
  //Start generating!
  if(strategy != NULL) {
    strategy->Solve();
  } else {
    cerr << "I don't have a MPStrategy" << endl << flush;
  }
   #ifndef _PARALLEL
    return 0;
    #endif
}


