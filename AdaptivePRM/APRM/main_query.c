///////////////////////////////////////////////////////////////////////////////
//  main_query.c		
//
///////////////////////////////////////////////////////////////////////////////

#include <iostream.h>
#include "AdaptiveQuery.h"
#include "Stat_Class.h"
#include "Clock_Class.h"
#include "GraphAlgo.h"
#include "MyQueryCmds.h"
#include "MyInput.h"
#include "PriorityQuery.h"
#include "PriorityLocalPlanners.h"
#include "PriorityWeight.h"
#include "LocalPlanners.h"

MyInput input;
MyQueryCmds Qinput;
extern Stat_Class Stats;

//========================================================================
//  main
//========================================================================
int main(int argc, char** argv)
{
  GenerateMapNodes   gn;
  ConnectMapNodes    cn;
  LocalPlanners      lp;
  PriorityLocalPlanners plp;
  DistanceMetric     dm;
  CollisionDetection cd;
  Clock_Class        QueryClock;

  //----------------------------------------------------
  // instantiate query/roadmap object
  // 	parse command line and init roadmap, lps, read in environment, etc
  //----------------------------------------------------
  Qinput.ReadCommandLine(&argc,argv);
  input.ReadCommandLine(argc,argv);

  PriorityWeightFactory* fact = new PriorityWeightFactory();
  WeightObject::SetWeightFactory(fact);

  enum {SEQUENTIAL, PRIORITY} pathValidateType = SEQUENTIAL;
  if(!strncmp(Qinput.pathValidationFlag.GetValue(),"priority",8))
    pathValidateType = PRIORITY;

  AdaptiveQuery query(&input, &Qinput, &cd, &dm, &lp, &cn);
  PriorityQuery pquery(&input, &Qinput, &cd, &dm, &plp, &cn);

  cd.UserInit(&input,   &gn, &cn );
  lp.UserInit(&input,        &cn );
  plp.UserInit(&input,       &cn );
  dm.UserInit(&input,   &gn, &lp );
  gn.UserInit(&input,  query.rdmp.GetEnvironment() );
  cn.UserInit(&input,  query.rdmp.GetEnvironment() );

  /** set up set ids for query stage. And this has been 
      done after cn has been set up */
  query.initDefaultSetIDs(&cn);
  pquery.initDefaultSetIDs(&cn);


  Qinput.PrintValues(cout);

  //---------------------------
  // Print out some useful info
  //---------------------------
  lp.planners.DisplayLPs();
  cout << endl;
  lp.planners.DisplayLPSets();
  cout << endl;
  DisplayCCStats(*(query.rdmp.m_pRoadmap),10);
  cout << endl;

  //----------------------------------------------------
  // perform the query
  // if successful, write path to a file
  //----------------------------------------------------
  QueryClock.StartClock("Query");
  switch(pathValidateType) {
  case SEQUENTIAL:
    if ( query.PerformQuery(&cd,&cn,&lp,&dm) ) {
      query.WritePath();
      cout << endl << "SUCCESSFUL query";
      query.rdmp.WriteRoadmap(&input,&cd,&dm,&lp);
    } else {
      cout << endl << "UNSUCCESSFUL query";
    }   
    break;
  case PRIORITY:
    if ( pquery.PerformQuery(&cd,&cn,&plp,&dm) ) {
      pquery.WritePath();
      cout << endl << "SUCCESSFUL query";
      pquery.rdmp.WriteRoadmap(&input,&cd,&dm,&plp);
    } else {
      cout << endl << "UNSUCCESSFUL query";
    } 
    break;
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





