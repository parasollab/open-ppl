
///////////////////////////////////////////////////////////////////////////////
//  main_growccs.cpp (based on main_query.cpp)
//
///////////////////////////////////////////////////////////////////////////////

#include <iostream.h>
#include "GenerateMapNodes.h"
#include "ConnectMapNodes.h"
#include "LocalPlanners.h"
#include "DistanceMetrics.h"
#include "CollisionDetection.h"
#include "Input.h"

#include "ConnectCCs.h"
#include "Stat_Class.h"
#include "Clock_Class.h"
#include "GraphAlgo.h"

Input input;
//QueryCmds Qinput;
extern Stat_Class Stats;

ConnectCCsCmds connect_CCs_input;

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
  Clock_Class        clock;

  //----------------------------------------------------
  // instantiate query/roadmap object
  //    parse command line and init roadmap, lps, read in environment, etc
  //----------------------------------------------------
  connect_CCs_input.ReadCommandLine(&argc,argv);
cout <<"in main_grows.cpp" << connect_CCs_input.option_str.GetValue();
  input.ReadCommandLine(argc,argv);

  //Query query(&input,&connect_CCs_input, &cd, &dm, &lp,&cn);
  Roadmap rdmp(&input,  &cd, &dm, &lp);
  ConnectCCs connect_ccs(&input,&rdmp,&connect_CCs_input, &cd, &dm, &lp,&cn);
  //rdmp.ReadRoadmapGRAPHONLY( (& connect_CCs_input)->mapFile.GetValue() );
  rdmp.ReadRoadmap(&input,&cd,&dm,&lp,(& connect_CCs_input)->mapFile.GetValue() );
  rdmp.InitEnvironment(&input);
  cd.UserInit(&input,   &gn, &cn );
  lp.UserInit(&input,        &cn );
  dm.UserInit(&input,   &gn, &lp );
//    gn.UserInit(&input, query.rdmp.GetEnvironment() );
//    cn.UserInit(&input, query.rdmp.GetEnvironment() );
  gn.UserInit(&input, rdmp.GetEnvironment() );
  cn.UserInit(&input, rdmp.GetEnvironment() );


  //ConnectCCs connect_ccs(&input,&rdmp,&connect_CCs_input, &cd, &dm, &lp,&cn);
  //rdmp.ReadRoadmapGRAPHONLY( (& connect_CCs_input)->mapFile.GetValue() );
  
  /** set up set ids for query stage. And this has been 
      done after cn has been set up */
  connect_ccs.initDefaultSetIDs(&cn);


  connect_CCs_input.PrintValues(cout);

  connect_CCs_input.ReadCommandLine(&argc,argv); 
cout <<"in main_grows.cpp" << connect_CCs_input.option_str.GetValue();
  //---------------------------
  // Print out some useful info
  //---------------------------
  lp.planners.DisplayLPs();
  cout << endl;
  lp.planners.DisplayLPSets();
  cout << endl;
  DisplayCCStats(*(rdmp.m_pRoadmap),10);
  cout << endl;

  //----------------------------------------------------
  // perform the query
  // if successful, write path to a file
  //----------------------------------------------------
  clock.StartClock("Connection of CCs");
//    if ( query.PerformQuery(&cd,&cn,&lp,&dm) ) {
//      query.WritePath();
//      cout << endl << "SUCCESSFUL query";
//    } else {
//      cout << endl << "UNSUCCESSFUL query";
//    }
  connect_ccs.PerformConnectCCs(&rdmp,&cd,&cn,&lp,&dm);
  clock.StopClock();

  //---------------------------
  // Write roadmap
  //---------------------------
  rdmp.WriteRoadmap(&input,&cd,&dm,&lp);

/*  #if QUIET
  #else
    cout << ": " << clock.GetClock_SEC()
         << " sec (ie, " << clock.GetClock_USEC() << " usec)";
  #endif
*/
  //---------------------------
  // Print out some useful info
  //---------------------------
  #ifdef QUIET
    ofstream  myofstream(input.mapFile.GetValue(),ios::app);
    if (!myofstream) {
      cout<<"\nIn main_obprm: can't re-open mapfile: "
          <<input.mapFile.GetValue();
      exit(-1);
    }
    PrintRawLine(cout,
        &rdmp, &NodeGenClock,&clock,cn,1);  // to stdout
    PrintRawLine(myofstream,
        &rdmp, &NodeGenClock,&clock,cn,0);  // to map
  #else
    cout << "\n";
    clock.PrintName();
    cout << ": " << clock.GetClock_SEC()
         << " sec"
         << ", "<<rdmp.m_pRoadmap->GetEdgeCount()<<" edges\n"<< flush;
    Stats.PrintAllStats(&rdmp);
  #endif

  //------------------------
  // Done
  //------------------------
  cout << "\n  !!Bye!! \n";
  return 0;
}
