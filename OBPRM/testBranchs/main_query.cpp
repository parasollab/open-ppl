// $Id$
///////////////////////////////////////////////////////////////////////////////
//  main_query.c        
//
///////////////////////////////////////////////////////////////////////////////

#include "Query.h"
#include "GenerateMapNodes.h"
#include "ConnectMap.h"
#include "LocalPlanners.h"
#include "DistanceMetrics.h"
#include "CollisionDetection.h"
#include "Input.h"

#include "Stat_Class.h"
#include "Clock_Class.h"
#include "GraphAlgo.h"
#include "Weight.h"

#include "CfgTypes.h"

typedef Cfg_free_tree CfgType;
typedef DefaultWeight WeightType;

Input input;
QueryCmds Qinput;
extern Stat_Class Stats;

//========================================================================
//  main
//========================================================================
int main(int argc, char** argv)
{
  GenerateMapNodes<CfgType>   gn;
  ConnectMap<CfgType, WeightType> cm;
  LocalPlanners<CfgType, WeightType> lp;
  DistanceMetric     dm;
  CollisionDetection cd;
  Clock_Class        QueryClock;

  //----------------------------------------------------
  // instantiate query/roadmap object
  //    parse command line and init roadmap, lps, read in environment, etc
  //----------------------------------------------------
  Qinput.ReadCommandLine(&argc,argv);
  input.ReadCommandLine(argc,argv);

  CfgType::setNumofJoints(input.numofJoints);
  cout << "Cfg_free_tree::NumofJoints = "
       << CfgType::getNumofJoints() << endl;

  Query<CfgType, WeightType> query(&input,&Qinput, &cd, &dm, &lp,&cm);

  cd.ReadCommandLine(input.CDstrings, input.numCDs);
  lp.ReadCommandLine(input.LPstrings, input.numLPs, input.cdtype, true);
  dm.ReadCommandLine(input.DMstrings, input.numDMs);
  gn.ReadCommandLine(input.GNstrings, input.numGNs);
  cm.ReadCommandLine(&input, query.rdmp.GetEnvironment());
  
  Qinput.PrintValues(cout);

  //---------------------------
  // Print out some useful info
  //---------------------------
  lp.PrintValues(cout);
  cout << endl;
  DisplayCCStats(*(query.rdmp.m_pRoadmap),10);
  cout << endl;

  //----------------------------------------------------
  // perform the query
  // if successful, write path to a file
  //----------------------------------------------------
  QueryClock.StartClock("Query");
  if ( query.PerformQuery(&cd,&cm,&lp,&dm) ) {
    query.WritePath();
    cout << endl << "SUCCESSFUL query";
  } else {
    cout << endl << "UNSUCCESSFUL query";
  }
  QueryClock.StopClock();

  #if QUIET
  #else
    cout << ": " << QueryClock.GetClock_SEC()
         << " sec (ie, " << QueryClock.GetClock_USEC() << " usec)";
  #endif


  //------------------------
  // Done
  //------------------------
  cout << "\n  !!Bye!! \n";
  return 0;
}
