// $Id$
///////////////////////////////////////////////////////////////////////////////
//  main_obprm.c		
//
///////////////////////////////////////////////////////////////////////////////

#include <iostream.h>

#include "SwitchDefines.h"

#include "OBPRMDef.h"
#include "Roadmap.h"
#include "Input.h"


#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "Weight.h"
#include "LocalPlanners.h"
#include "GenerateMapNodes.h"

#include "CfgTypes.h"

typedef Cfg_2D CfgType;
typedef DefaultWeight WeightType;

Input input;
Stat_Class Stats; 
void PrintRawLine( ostream& _os,
        Roadmap<CfgType, WeightType> *rmap, Clock_Class *NodeGenClock, Clock_Class *ConnectionClock,
        int printHeaders);

//========================================================================
//  main
//========================================================================
int main(int argc, char** argv)
{
  GenerateMapNodes<CfgType> gn;
  ConnectMap<CfgType, WeightType> cm;
  LocalPlanners<CfgType, WeightType> lp;
  DistanceMetric     dm;
  CollisionDetection cd;
  Clock_Class        NodeGenClock;
  Clock_Class        ConnectionClock;

  //----------------------------------------------------
  // Get information from the user via the command line
  //----------------------------------------------------
  input.ReadCommandLine(argc,argv);

  // do not seed for now while developing code... ADD SEEDING LATER
  // srand48((unsigned int) time(NULL));

  //----------------------------------------------------
  // instantiate roadmap object 
  //	parse command line and init roadmap, lps, read in environment, etc
  // initiate collision detection
  // build map
  // write map to a file
  //----------------------------------------------------
  Roadmap<CfgType, WeightType> rmap(&input,  &cd, &dm, &lp);
  
  cd.ReadCommandLine(input.CDstrings, input.numCDs);
  lp.ReadCommandLine(input.LPstrings, input.numLPs, input.cdtype);
  gn.ReadCommandLine(input.GNstrings, input.numGNs);
  cm.ReadCommandLine(&input, rmap.GetEnvironment());
  dm.ReadCommandLine(input.DMstrings, input.numDMs);

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
    vector<CfgType> nodes;
    gn.GenerateNodes<WeightType>(&rmap,Stats,&cd,&dm,nodes);
    NodeGenClock.StopClock();
  }


  #ifdef QUIET
  #else
    cout << "\n";
    if ( input.inmapFile.IsActivated() ){
      cout << "Node Generation: ";
    }else{
      cout << "Node Generation: " << NodeGenClock.GetClock_SEC()
           << " sec (ie, " << NodeGenClock.GetClock_USEC() << " usec)";
    }
    
	cout << ", "<<rmap.m_pRoadmap->GetVertexCount()<<" nodes\n"<< flush;
  #endif


  //---------------------------
  // Connect roadmap nodes
  //---------------------------
  ConnectionClock.StartClock("Node Connection");
  cm.ConnectComponents(&rmap, Stats, &cd, &dm, &lp,
		       input.addPartialEdge.GetValue(), input.addAllEdges.GetValue());
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
        &rmap, &NodeGenClock,&ConnectionClock,1);  // to stdout
    PrintRawLine(myofstream,
        &rmap, &NodeGenClock,&ConnectionClock,0);  // to map
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
        Roadmap<CfgType, WeightType> *rmap, Clock_Class *NodeGenClock, Clock_Class *ConnectionClock,
        int printHeader = 0 ){

  _os << "\nraw ";               // We can grep out "raw" datalines.
  _os << NodeGenClock->GetClock_SEC()     << " ";
  _os << NodeGenClock->GetClock_USEC()    << " ";
  _os << ConnectionClock->GetClock_SEC()  << " ";
  _os << ConnectionClock->GetClock_USEC() << " ";
  Stats.PrintDataLine(_os,rmap,printHeader);
  _os << "\n\n";
}
