///////////////////////////////////////////////////////////////////////////////
//  main_growccs.test.cpp (based on main_query.cpp and main_growccs.cpp)
// We'll test the new framework with this main file

#include <iostream.h>
#include "GenerateMapNodes.h"
#include "ConnectMapNodes.h"
#include "LocalPlanners.h"
#include "DistanceMetrics.h"
#include "CollisionDetection.h"
#include "Input.h"

#include "Stat_Class.h"
#include "Clock_Class.h"
#include "GraphAlgo.h"

#include "ConnectCCMethod.h"

Input input;
extern Stat_Class Stats;

//========================================================================
int main(int argc, char** argv)
{
  GenerateMapNodes   gn;
  ConnectMapNodes    cn;
  LocalPlanners      lp;
  DistanceMetric     dm;
  CollisionDetection cd;
  Clock_Class        clock;

  Roadmap rdmp;
  //Testing the use of the ConnectCCMethodCaller
  //ConnectMapComponents component_connector;
  ConnectMapComponents component_connector =  ConnectMapComponents(&input,&rdmp,&cd,&dm, &lp,&cn);
  component_connector.ReadCommandLine(&argc, argv);

  //----------------------------------------------------
  // instantiate roadmap object
  //    parse command line and init roadmap, lps, read in environment, etc

  input.ReadCommandLine(argc,argv);
  //Roadmap rdmp(&input,  &cd, &dm, &lp);
  rdmp.InitRoadmap(&input, &cd, &dm, &lp);
  cd.UserInit(&input,   &gn, &cn );
  lp.UserInit(&input,        &cn );
  dm.UserInit(&input,   &gn, &lp );
  gn.UserInit(&input, rdmp.GetEnvironment() );
  cn.UserInit(&input, rdmp.GetEnvironment() );


  if ( input.inmapFile.IsActivated() ){
    //---------------------------
    // Read roadmap 
    //---------------------------
    cout << "branch 1 :" << (&input)->inmapFile.GetValue();
    rdmp.ReadRoadmap(&input,&cd,&dm,&lp,(&input)->inmapFile.GetValue());
  } else {
    //---------------------------
    // Use default file to read in roadmap
    //---------------------------
	char tmp[80];
	strcpy(tmp, (&input)->defaultFile.GetValue() ); 
	strcat(tmp,".map");
        cout << "branch 2 :" << tmp;
	//mapFile.PutValue(&input,&cd,&dm,&lp,(&tmp));
    rdmp.ReadRoadmap(&input,&cd,&dm,&lp,tmp);
  }
  //We need to move the commented code below to the ConnectCCMethods
//    cout <<"in main_grows.cpp" << connect_CCs_input.option_str.GetValue();
//    ConnectCCs connect_ccs(&input,&rdmp,&connect_CCs_input, &cd, &dm, &lp,&cn);
  /** set up set ids for query stage after cn has been set up */
//    connect_ccs.initDefaultSetIDs(&cn);
//    connect_CCs_input.PrintValues(cout);
//    connect_CCs_input.ReadCommandLine(&argc,argv); 

  //---------------------------
  // Print out some useful info
  //---------------------------
//  cout <<"in main_grows.cpp" << connect_CCs_input.option_str.GetValue();

//    lp.planners.DisplayLPs();
//    cout << endl;
//    lp.planners.DisplayLPSets();
//    cout << endl;
    DisplayCCStats(*(rdmp.m_pRoadmap),10);
    cout << endl;

  //----------------------------------------------------
  // The cc_connector (an instance of ConnectCCMethodCaller attempts 
  // to connect separate CCs
  //----------------------------------------------------
  clock.StartClock("Connection of CCs");
  component_connector.ConnectComponents();
  //    connect_ccs.PerformConnectCCs(&rdmp,&cd,&cn,&lp,&dm);
  clock.StopClock();

  //---------------------------
  // Write roadmap
  //---------------------------
  rdmp.WriteRoadmap(&input,&cd,&dm,&lp);

  //---------------------------
  // Print out some useful info
  //---------------------------
#ifdef QUIET
//    ofstream myofstream(input.mapFile.GetValue(),ios::app);
//    if (!myofstream) {
//      cout<<"\nIn main_obprm: can't re-open mapfile: "<<input.mapFile.GetValue();
//      exit(-1);
//    }
//    PrintRawLine(cout, &rdmp, &NodeGenClock,&clock,cn,1);  // to stdout
//    PrintRawLine(myofstream, &rdmp, &NodeGenClock,&clock,cn,0);  // to map
#else	
//    cout << "\n";
//    clock.PrintName();
//    cout << ": " << clock.GetClock_SEC() << " sec"
//         << ", "<< rdmp.m_pRoadmap->GetEdgeCount()<<" edges\n"<< flush;
//    Stats.PrintAllStats(&rdmp);
#endif
  
  //------------------------
  // Done
  //------------------------
  cout << "\n  !!Bye!! \n";
  return 0;
}
