// $Id$
///////////////////////////////////////////////////////////////////////////////
//  main_obprm.c		
//
///////////////////////////////////////////////////////////////////////////////

#include <iostream.h>

#include "SwitchDefines.h"

#include "OBPRM.h"
#include "Roadmap.h"

#include "MyInput.h"
#include "DynamicsLocalPlanners.h"


#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMapNodes.h"

#include "GradientDecent.h"
#include "BioPotentials.h"

MyInput input;
Stat_Class Stats; 
void PrintRawLine( ostream& _os, Roadmap *rmap, 
                   Clock_Class *NodeGenClock, Clock_Class *ConnectionClock );

num_param<int>    numNodes("-nodes",             10,  1,50000);
str_param<char *> mapFile          ("-outmapFile");

//========================================================================
//  main
//========================================================================
int main(int argc, char** argv)
{
  GenerateMapNodes   gn;
  ConnectMapNodes    cn;
  DynamicsLocalPlanners    lp;
  DistanceMetric     dm;
  CollisionDetection cd;
  Clock_Class        NodeGenClock;
  Clock_Class        ConnectionClock;

  numNodes.PutValue(5);
  mapFile.PutValue("salam");
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
  gn.UserInit(&input,                  rmap.GetEnvironment() );
  cn.UserInit(&input);
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



  #ifdef QUIET
  #else
    cout << "\n";
    if ( input.inmapFile.IsActivated() ){
      cout << "Node Generation: "<<argv[argc];
    }else{
      cout << ": " << NodeGenClock.GetClock_SEC()
           << " sec (ie, " << NodeGenClock.GetClock_USEC() << " usec)";
    }
      cout << ", "<<rmap.roadmap.GetVertexCount()<<" nodes\n"<< flush;
  #endif



  //---------------------------
  // Connect roadmap nodes
  //---------------------------
  ConnectionClock.StartClock("Node Connection");
  cn.ConnectNodes(&rmap,&cd,&lp,&dm, cn.cnInfo.cnsetid, cn.cnInfo);
  ConnectionClock.StopClock();


  #ifdef QUIET
    // When running closestVE, nodes and edges are explicitly added which are
    // already implicit.  We want see how many of each.
    ofstream  myofstream(input.mapFile.GetValue(),ios::app);
    if (!myofstream) {
      cout<<"\nIn main_obprm: can't re-open mapfile: "
          <<input.mapFile.GetValue();
      exit(-1);
    }
    myofstream << "\nbVE 0 0 0 0 ";
    Stats.PrintDataLine(myofstream,&rmap); // to map
  #else
    cout << "\n";
    ConnectionClock.PrintName();
    cout << ": " << ConnectionClock.GetClock_SEC()
         << " sec"
         << ", "<<rmap.roadmap.GetEdgeCount()<<" edges\n"<< flush;
  #endif


  //---------------------------
  // Write roadmap
  //---------------------------
  rmap.WriteRoadmap(&input,&cd,&dm,&lp);


  //---------------------------
  // Print out some useful info
  //---------------------------
  #ifdef QUIET
    myofstream << "\naVE "<<cn.cnInfo.dupeNodes<<" "<<cn.cnInfo.dupeEdges;
    PrintRawLine(      cout,&rmap,&NodeGenClock,&ConnectionClock);  // to stdout
    PrintRawLine(myofstream,&rmap,&NodeGenClock,&ConnectionClock);  // to map
  #else
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


void PrintRawLine( ostream& _os, Roadmap *rmap, 
                   Clock_Class *NodeGenClock, Clock_Class *ConnectionClock ){
  _os << "\nraw ";               // We can grep out "raw" datalines.
  _os << NodeGenClock->GetClock_SEC()     << " ";
  _os << NodeGenClock->GetClock_USEC()    << " ";
  _os << ConnectionClock->GetClock_SEC()  << " ";
  _os << ConnectionClock->GetClock_USEC() << " ";
  Stats.PrintDataLine(_os,rmap,1);
  _os << "\n\n";
}
