///////////////////////////////////////////////////////////////////////////////
//  main_utility_guided.c		
//
///////////////////////////////////////////////////////////////////////////////

#include "Roadmap.h"
#include "Input.h"


#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "Weight.h"
#include "LocalPlanners.h"
#include "UtilityGuidedGenerator.h"

#include "CfgTypes.h"

Input input;
Stat_Class Stats; 


//========================================================================
//  main
//========================================================================
int main(int argc, char** argv)
{
  UtilityGuidedGenerator<CfgType, WeightType> mg;
  LocalPlanners<CfgType, WeightType> lp;
  DistanceMetric     dm;
  CollisionDetection cd;
  Clock_Class        MapGenClock;

  //---------------------------------------------------
  // Get information from the user via the command line
  //----------------------------------------------------
  input.ReadCommandLine(argc,argv);
  
  /*
  CfgType::setNumofJoints(input.numofJoints.GetValue());
  cout << "Cfg_free_tree::NumofJoints = " 
       << CfgType::getNumofJoints() << endl;
  */

  long baseSeed;
  //baseSeed = OBPRM_srand();
  baseSeed = OBPRM_srand(input.seed.GetValue());
  //baseSeed = OBPRM_srand((unsigned int) time(NULL));

  //----------------------------------------------------
  // instantiate roadmap object 
  //	parse command line and init roadmap, lps, read in environment, etc
  // initiate collision detection
  // build map
  // write map to a file
  //----------------------------------------------------

  // create new environment
  CfgType test_cfg;
  Environment env(test_cfg.DOF(),test_cfg.posDOF(), &input);
 
  //create new roadmap
  Roadmap<CfgType, WeightType> rmap(&input,  &cd, &dm, &lp, baseSeed, &env);
  cout<<"the RNGseed in rmap is (from main): "<<rmap.GetRNGseed()<<endl;
  
  cd.ReadCommandLine(input.CDstrings, input.numCDs);
  lp.ReadCommandLine(input.LPstrings, input.numLPs, input.cdtype);
  mg.ReadCommandLine(input.GNstrings, input.numGNs);
  //mg.cm.ReadCommandLine(&input, &env);
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
#else
  cout << "\n";
  MapGenClock.PrintName();
  cout << ": " << MapGenClock.GetClock_SEC()
       << " sec"
       << ", "<<rmap.m_pRoadmap->get_num_vertices()<<" nodes"
       << ", "<<rmap.m_pRoadmap->get_num_edges()<<" edges\n"<< flush;
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

