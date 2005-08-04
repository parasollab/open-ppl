// $Id$
///////////////////////////////////////////////////////////////////////////////
//  main_query.c        
//
///////////////////////////////////////////////////////////////////////////////

#include <iostream.h>
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

typedef Cfg_2D CfgType;
typedef DefaultWeight WeightType;

Input input;
QueryCmds Qinput;
Stat_Class Stats;

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

  Roadmap<CfgType, WeightType> rdmp;
  rdmp.InitRoadmap(&input,&cd,&dm,&lp,Qinput.mapFile.GetValue() );

  Query<CfgType, WeightType> query(&Qinput);

  cd.ReadCommandLine(input.CDstrings, input.numCDs);
  lp.ReadCommandLine(input.LPstrings, input.numLPs, input.cdtype, true);
  dm.ReadCommandLine(input.DMstrings, input.numDMs);
  gn.ReadCommandLine(input.GNstrings, input.numGNs);
  cm.ReadCommandLine(&input, rdmp.GetEnvironment());
  
  Qinput.PrintValues(cout);

  //---------------------------
  // Print out some useful info
  //---------------------------
  lp.PrintValues(cout);
  cout << endl;
  DisplayCCStats(*(rdmp.m_pRoadmap),10);
  cout << endl;

  //----------------------------------------------------
  // perform the query
  // if successful, write path to a file
  //----------------------------------------------------
  QueryClock.StartClock("Query");
  if ( query.PerformQuery(&rdmp,Stats,&cd,&cm,&lp,&dm) ) {
    query.WritePath(&rdmp);
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
