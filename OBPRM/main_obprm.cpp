// $Id$
///////////////////////////////////////////////////////////////////////////////
//  main_obprm.c		
//
///////////////////////////////////////////////////////////////////////////////

#include <iostream.h>

#include "SwitchDefines.h"

#include "OBPRM.h"
#include "Roadmap.h"
#include "Input.h"


#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMapNodes.h"
#include "DistanceMetrics.h"

Input input;
extern Stat_Class Stats; 
void PrintRawLine( ostream& _os,
        Roadmap *rmap, Clock_Class *NodeGenClock, Clock_Class *ConnectionClock,
        ConnectMapNodes cn, int printHeaders);

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
  Clock_Class        NodeGenClock;
  Clock_Class        ConnectionClock;

  //----------------------------------------------------
  // Get information from the user via the command line
  //----------------------------------------------------
  input.ReadCommandLine(argc,argv);


  // do not seed for now while developing code... ADD SEEDING LATER
  //    srand48((unsigned int) time(NULL));

  //----------------------------------------------------
  // instantiate roadmap object 
  //	parse command line and init roadmap, lps, read in environment, etc
  // initiate collision detection
  // build map
  // write map to a file
  //----------------------------------------------------
  Roadmap rmap(&input,  &cd, &dm, &lp);
  cd.UserInit(&input,   &gn, &cn );
  lp.UserInit(&input,        &cn );
  gn.UserInit(&input, rmap.GetEnvironment() );
  cn.UserInit(&input, rmap.GetEnvironment() );
  dm.UserInit(&input,   &gn, &lp );


  #ifdef QUIET
  #else
    input.PrintValues(cout);
  #endif

  if ( input.inmapFile.IsActivated() ){
    //---------------------------
    // Read roadmap nodes
    //---------------------------
    rmap.ReadRoadmapGRAPHONLY(input.inmapFile.GetValue());
  } else {
    //---------------------------
    // Generate roadmap nodes
    //---------------------------
    NodeGenClock.StartClock("Node Generation");
    gn.GenerateNodes(&rmap,&cd,&dm, gn.gnInfo.gnsetid, gn.gnInfo);
    NodeGenClock.StopClock();
  }


  #if QUIET
  #else
    cout << "\n";
    if ( input.inmapFile.IsActivated() ){
      cout << "Node Generation: "<<argv[argc];
    }else{
      cout << ": " << NodeGenClock.GetClock_SEC()
           << " sec (ie, " << NodeGenClock.GetClock_USEC() << " usec)";
    }
    
	cout << ", "<<rmap.m_pRoadmap->GetVertexCount()<<" nodes\n"<< flush;
  #endif


  //---------------------------
  // Connect roadmap nodes
  //---------------------------
  ConnectionClock.StartClock("Node Connection");
  cn.ConnectNodes(&rmap,&cd,&lp,&dm, cn.cnInfo.cnsetid, cn.cnInfo);
  ConnectionClock.StopClock();

  //---------------------------
  // Write roadmap
  //---------------------------
  rmap.WriteRoadmap(&input,&cd,&dm,&lp);


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
        &rmap, &NodeGenClock,&ConnectionClock,cn,1);  // to stdout
    PrintRawLine(myofstream,
        &rmap, &NodeGenClock,&ConnectionClock,cn,0);  // to map
  #else
    cout << "\n";
    ConnectionClock.PrintName();
    cout << ": " << ConnectionClock.GetClock_SEC()
         << " sec"
         << ", "<<rmap.m_pRoadmap->GetEdgeCount()<<" edges\n"<< flush;
    Stats.PrintAllStats(&rmap);
  #endif


  //------------------------
  // Done
  //------------------------
  #ifdef QUIET
  #else
    cout << "\n  !!Bye!! \n";
  #endif
  return 0;
}


void PrintRawLine( ostream& _os,
        Roadmap *rmap, Clock_Class *NodeGenClock, Clock_Class *ConnectionClock,
        ConnectMapNodes cn,int printHeader = 0 ){

  _os << "\ndups  " <<cn.cnInfo.dupeNodes<<" "<<cn.cnInfo.dupeEdges;

  _os << "\nraw ";               // We can grep out "raw" datalines.
  _os << NodeGenClock->GetClock_SEC()     << " ";
  _os << NodeGenClock->GetClock_USEC()    << " ";
  _os << ConnectionClock->GetClock_SEC()  << " ";
  _os << ConnectionClock->GetClock_USEC() << " ";
  Stats.PrintDataLine(_os,rmap,printHeader);
  _os << "\n\n";
}
