// $Id$
///////////////////////////////////////////////////////////////////////////////
//  main_query.c        
//
///////////////////////////////////////////////////////////////////////////////

#include <iostream.h>
#include "GenerateMapNodes.h"
#include "ConnectMapNodes.h"
#include "LocalPlanners.h"
#include "DistanceMetrics.h"
#include "CollisionDetection.h"
#include "Input.h"

#include "Query.h"
#include "Stat_Class.h"
#include "Clock_Class.h"

Input input;
QueryCmds Qinput;
Stat_Class Stats;

//========================================================================
//  main
//========================================================================
int main(int argc, char** argv)
{
  GenerateMapNodes   gn;
  ConnectMapNodes    cn;
  LocalPlanners      lp;
  DistanceMetric     dm;
  CollisionDetection cd;
  Clock_Class        QueryClock;

  //----------------------------------------------------
  // instantiate query/roadmap object
  //    parse command line and init roadmap, lps, read in environment, etc
  //----------------------------------------------------
  Qinput.ReadCommandLine(&argc,argv);
  input.ReadCommandLine(argc,argv);

  Query query(&input,&Qinput, &cd, &dm, &lp,&cn);

  cd.UserInit(&input,   &gn, &cn );
  lp.UserInit(&input,        &cn );
  dm.UserInit(&input,   &gn, &lp );
  gn.UserInit(&input, query.rdmp.GetEnvironment() );
  cn.UserInit(&input, query.rdmp.GetEnvironment() );
  
  /** set up set ids for query stage. And this has been 
      done after cn has been set up */
  query.initDefaultSetIDs(&cn);


  Qinput.PrintValues(cout);

  //---------------------------
  // Print out some useful info
  //---------------------------
  lp.planners.DisplayLPs();
  cout << endl;
  lp.planners.DisplayLPSets();
  cout << endl;
  query.rdmp.m_pRoadmap->DisplayCCStats(10);
  cout << endl;

  //----------------------------------------------------
  // perform the query
  // if successful, write path to a file
  //----------------------------------------------------
  QueryClock.StartClock("Query");
  if ( query.PerformQuery(&cd,&cn,&lp,&dm) ) {
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
