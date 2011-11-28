///////////////////////////////////////////////////////////////////////////////
//  main_obprm.c		
//
///////////////////////////////////////////////////////////////////////////////

#include "Roadmap.h"
#include "Input.h"
#include "Clock_Class.h"
#include "MetricUtils.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "Weight.h"
#include "LocalPlanners.h"
#include "CfgTypes.h"
#include "EntropyPRM.h"

#include "Query.h"


Input input;
QueryCmds Qinput; //sam
Stat_Class Stats; 

//========================================================================
//  main
//========================================================================
int main(int argc, char** argv)
{
  EntropyPRM<CfgType> eprm;
  LocalPlanners<CfgType, WeightType> lp;
  DistanceMetric     dm;
  CollisionDetection cd;
  CDInfo cdInfo;

  //---------------------------------------------------
  // Get information from the user via the command line
  //----------------------------------------------------
  Qinput.ReadCommandLine(&argc,argv); //sam
  input.ReadCommandLine(argc,argv);

 CfgType::setNumofJoints(input.numofJoints.GetValue());
   cout << "Cfg_free_tree::NumofJoints = " 
       << CfgType::getNumofJoints() << endl;

  long baseSeed;
  //baseSeed = OBPRM_srand();
  baseSeed = OBPRM_srand((unsigned int) time(NULL));

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
  Roadmap<CfgType, WeightType> rmap(&input, &cd, &dm, &lp, baseSeed, &env);
  Roadmap<CfgType, WeightType> regionMap(&input, &cd, &dm, &lp, baseSeed, &env);
  cout << "the RNGseed in rmap is (from main): " << rmap.GetRNGseed() << endl;

  cd.ReadCommandLine(input.CDstrings, input.numCDs);
  lp.ReadCommandLine(input.LPstrings, input.numLPs, input.cdtype);
  eprm.ReadCommandLine(input.GNstrings, input.numGNs);
  //eprm.cm.ReadCommandLine(&input, &env);
  dm.ReadCommandLine(input.DMstrings, input.numDMs);

  #ifdef QUIET
  #else
    input.PrintValues(cout);
  #endif

  Clock_Class MapGenClock;
  MapGenClock.StartClock("Map Generation");
  vector<CfgType> nodes;

  Clock_Class GenClock;
  GenClock.StartClock("Node Generation");
  eprm.GenerateNodes(&env, Stats, &cd, &cdInfo, &dm, nodes);
  rmap.m_pRoadmap->AddVertex(nodes);
  GenClock.StopPrintClock();
  eprm.cmodel.WriteRegionsToSpecFile("regions.spec");
  eprm.cmodel.WriteRegionsToMapFile(&regionMap, &input, &cd, &dm, &lp);

  Clock_Class MergeRegionClock;
  MergeRegionClock.StartClock("Merging Regions");
  eprm.cmodel.MergeRegions(&env, 
			   Stats, &cd, &cdInfo, &dm);
  MergeRegionClock.StopPrintClock();

/*
  Clock_Class RegionClock;
  RegionClock.StartClock("Merging Regions");
  eprm.cmodel.BuildRegionMap(&env, &dm);
  eprm.cmodel.MergeRegions(&env, Stats, &cd, &cdInfo, &dm);
  RegionClock.StopPrintClock();
  eprm.cmodel.WriteRegionsToSpecFile("merged.spec");
*/

/*
  Clock_Class ConnClock;
  ConnClock.StartClock("NodeConnection");
  eprm.Connect(&rmap, Stats, &cd, &cdInfo, &dm, &lp, 
	       input.addPartialEdge.GetValue(), input.addAllEdges.GetValue(),
	       5, 10);
  ConnClock.StopPrintClock();
*/

  //query used in lazy style
  Query<CfgType, WeightType> query(&Qinput);//sam
  CfgType s_cfg = query.getStartCFG();
  CfgType g_cfg = query.getGoalCFG();
  cout << " start config: " << s_cfg << endl;
  cout << " goal config: " << g_cfg << endl;
  eprm.ExtractRegionPath(&rmap,s_cfg, g_cfg, Stats, &cd, &cdInfo, &dm, &lp);

  MapGenClock.StopClock();

  
  //---------------------------
  // Write roadmap
  //---------------------------
  rmap.WriteRoadmap(&input,&cd,&dm,&lp);


  //---------------------------
  // Print out some useful info
  //---------------------------
  #ifndef QUIET
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
