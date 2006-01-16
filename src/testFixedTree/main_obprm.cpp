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
#include "MapGenerator.h"

#include "CfgTypes.h"

typedef Cfg_fixed_tree CfgType;
typedef DefaultWeight WeightType;

Input input;
Stat_Class Stats; 

void PrintRawLine( ostream& _os,
        Roadmap<CfgType, WeightType> *rmap, Clock_Class *MapGenClock,
        int printHeaders);

//========================================================================
//  main
//========================================================================
int main(int argc, char** argv)
{
  MapGenerator<CfgType, WeightType> mg;
  LocalPlanners<CfgType, WeightType> lp;
  DistanceMetric     dm;
  CollisionDetection cd;
  Clock_Class        MapGenClock;

  //---------------------------------------------------
  // Get information from the user via the command line
  //----------------------------------------------------
  input.ReadCommandLine(argc,argv);

   CfgType::setNumofJoints(input.numofJoints.GetValue());
   cout << "Cfg_fixed_tree::NumofJoints = " 
       << CfgType::getNumofJoints() << endl;


  // do not seed for now while developing code... ADD SEEDING LATER
  // use the following way to seed
  // baseSeed = OBPRM_srand(seedValue);
  // e.g., baseSeed = OBPRM_srand((unsigned int) time(NULL)), here the seedValue is time
  long baseSeed;
  baseSeed = OBPRM_srand();

  //----------------------------------------------------
  // instantiate roadmap object 
  //	parse command line and init roadmap, lps, read in environment, etc
  // initiate collision detection
  // build map
  // write map to a file
  //----------------------------------------------------
  Roadmap<CfgType, WeightType> rmap(&input,  &cd, &dm, &lp, baseSeed);
  cout<<"the RNGseed in rmap is (from main): "<<rmap.GetRNGseed()<<endl;
  
  cd.ReadCommandLine(input.CDstrings, input.numCDs);
  lp.ReadCommandLine(input.LPstrings, input.numLPs, input.cdtype);
  mg.gn.ReadCommandLine(input.GNstrings, input.numGNs);
  mg.cm.ReadCommandLine(&input, rmap.GetEnvironment());
  dm.ReadCommandLine(input.DMstrings, input.numDMs);

  #ifdef QUIET
  #else
    input.PrintValues(cout);
  #endif

    MapGenClock.StartClock("Map Generation");
    vector<CfgType> nodes;
    mg.GenerateMap(&rmap,Stats,&cd,&dm,nodes,&lp, &input);
    MapGenClock.StopClock();

  
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
        &rmap, &MapGenClock,1);  // to stdout
    PrintRawLine(myofstream,
        &rmap, &MapGenClock,0);  // to map
  #else
    cout << "\n";
    MapGenClock.PrintName();
    cout << ": " << MapGenClock.GetClock_SEC()
         << " sec"
 	 << ", "<<rmap.m_pRoadmap->GetVertexCount()<<" nodes"
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
        Roadmap<CfgType, WeightType> *rmap, Clock_Class *MapGenClock,
        int printHeader = 0 ){

  _os << "\nraw ";               // We can grep out "raw" datalines.
  _os << MapGenClock->GetClock_SEC()     << " ";
  _os << MapGenClock->GetClock_USEC()    << " ";
  Stats.PrintDataLine(_os,rmap,printHeader);
  _os << "\n\n";
}
