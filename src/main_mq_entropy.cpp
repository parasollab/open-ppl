///////////////////////////////////////////////////////////////////////////////
//  main_obprm.c		
//
///////////////////////////////////////////////////////////////////////////////

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
#include "CfgTypes.h"
#include "EntropyPRM.h"

Input input;
Stat_Class Stats; 

//========================================================================
//  main
//========================================================================
int main(int argc, char** argv)
{
  EntropyPRM<CfgType> eprm;
  ConnectMap<CfgType, WeightType> cm;
  LocalPlanners<CfgType, WeightType> lp;
  DistanceMetric     dm;
  CollisionDetection cd;
  CDInfo cdInfo;

  //---------------------------------------------------
  // Get information from the user via the command line
  //----------------------------------------------------
  input.ReadCommandLine(argc,argv);

  CfgType::setNumofJoints(input.numofJoints.GetValue());
   cout << "Cfg_free_tree::NumofJoints = " 
       << CfgType::getNumofJoints() << endl;

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
  Roadmap<CfgType, WeightType> rmap(&input, &cd, &dm, &lp, baseSeed, &env);
  Roadmap<CfgType, WeightType> regionMap(&input, &cd, &dm, &lp, baseSeed, &env);
  cout << "the RNGseed in rmap is (from main): " << rmap.GetRNGseed() << endl;

  cd.ReadCommandLine(input.CDstrings, input.numCDs);
  lp.ReadCommandLine(input.LPstrings, input.numLPs, input.cdtype);
  eprm.ReadCommandLine(input.GNstrings, input.numGNs);
  cm.ReadCommandLine(&input, &env);
  dm.ReadCommandLine(input.DMstrings, input.numDMs);

  eprm.connectionPosRes = env.GetPositionRes();
  eprm.connectionOriRes = env.GetOrientationRes();

  cout << "res: " << eprm.connectionPosRes << " " << eprm.connectionOriRes << endl;


  #ifdef QUIET
  #else
    input.PrintValues(cout);
  #endif

  Clock_Class MapGenClock;
  MapGenClock.StartClock("Map Generation");
  vector<CfgType> nodes;

  Clock_Class GenClock;
  GenClock.StartClock("Node Generation");
  eprm.GenerateNodes(&env, 
		     Stats, &cd, &cdInfo, &dm, nodes);
  //rmap.m_pRoadmap->AddVertex(nodes);
  GenClock.StopPrintClock();
  //eprm.cmodel.WriteRegionsToSpecFile("regions.spec");

  Clock_Class RegionClock;
  RegionClock.StartClock("Merging Regions");
  cout << "Building region map" << endl;
  eprm.cmodel.BuildRegionMap(&env,
			     &dm);
  /*
  eprm.cmodel.MergeRegions(&env, 
			   Stats, &cd, &cdInfo, &dm);
  */

  RegionClock.StopPrintClock();
  //eprm.cmodel.WriteRegionsToSpecFile("merged.spec");
  //eprm.cmodel.WriteRegionsToMapFile(&regionMap, &input, &cd, &dm, &lp);

  Clock_Class AddClock;
  AddClock.StartClock("Add Nodes to Roadmap");
  eprm.AddNodesToRoadmap(&rmap);
  AddClock.StopPrintClock();

  Clock_Class ConnClock;
  ConnClock.StartClock("Node Connection");

  eprm.Connect(&rmap, Stats, &cd, &cdInfo, &dm, &lp, 
	       input.addPartialEdge.GetValue(), input.addAllEdges.GetValue());
  /*
  cm.Connect(&rmap, Stats, &cd, &dm, &lp, 
	     input.addPartialEdge.GetValue(), input.addAllEdges.GetValue());
  */
  ConnClock.StopPrintClock();

  //eprm.cmodel.WriteRegionsToSpecFile("merged.spec");

  MapGenClock.StopClock();

  //strip rm of unused cfgs
  //eprm.Strip();

  
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
 	 << ", "<<rmap.m_pRoadmap->GetVertexCount()<<" nodes"
         << ", "<<rmap.m_pRoadmap->GetEdgeCount()<<" edges\n"<< flush;
    Stats.PrintAllStats(&rmap);
  #endif

/*
  eprm.cmodel.WriteRegionsToMapFile(&regionMap, &input, &cd, &dm, &lp);

  regionMap.m_pRoadmap->EraseGraph();
  vector<CfgType> final;
  rmap.m_pRoadmap->GetVerticesData(final);
  for(vector<CfgType>::iterator F = final.begin(); F != final.end(); ++F)
    regionMap.m_pRoadmap->AddVertex(*F);
  regionMap.WriteRoadmap(&input, &cd, &dm, &lp, "final.map");
*/

  //------------------------
  // Done
  //------------------------
  #ifdef QUIET
  #else
    cout << "\n  !!Bye!! \n";
  #endif
  return 0;
}
