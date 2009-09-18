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
#include "MapGenerator.h"
#include "Query.h"

#include "CfgTypes.h"

Input input;
Stat_Class Stats; 

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

  Cfg::setNumofJoints(input.numofJoints.GetValue());
  Cfg_free_tree::setNumofJoints(input.numofJoints.GetValue());
  CfgType::setNumofJoints(input.numofJoints.GetValue());
  cout << "Cfg_free_tree::NumofJoints = " 
       << CfgType::getNumofJoints() << endl;

  CfgType::rdres = (input.rdres.GetValue());
  CfgType::gamma = (input.gamma.GetValue());

  //ideally read in from env file...
  string link_length_filename(input.defaultFile.GetValue());
  link_length_filename += ".len";
  CfgType::initialize_link_tree(link_length_filename.c_str());
  CfgType::print_link_tree(cout);

  // do not seed for now while developing code... ADD SEEDING LATER
  // use the following way to seed
  // baseSeed = OBPRM_srand(seedValue);
  // e.g., baseSeed = OBPRM_srand((unsigned int) time(NULL)), here the seedValue is time
  long baseSeed = OBPRM_srand();

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

  /*
  string query_name(input.defaultFile.GetValue());
  query_name += ".query";
  cout << "Query file: " << query_name << endl;
  Query<CfgType, WeightType> query(query_name.c_str());
  */

  ifstream ifsb("start.base");
  Vector6<double> start_base;
  start_base.Read(ifsb);
  ifstream ifsl("start.len");
  vector<double> start_len;
  copy(istream_iterator<double>(ifsl),
       istream_iterator<double>(),
       back_insert_iterator<vector<double> >(start_len));
  ifsl.close();
  ifstream ifso("start.ori");
  vector<int> start_ori;
  copy(istream_iterator<int>(ifso),
       istream_iterator<int>(),
       back_insert_iterator<vector<int> >(start_ori));
  ifsl.close();
  CfgType start(start_base, start_len, start_ori);

  ifstream ifgb("goal.base");
  Vector6<double> goal_base;
  goal_base.Read(ifgb);
  ifgb.close();
  ifstream ifgl("goal.len");
  vector<double> goal_len;
  copy(istream_iterator<double>(ifgl),
       istream_iterator<double>(),
       back_insert_iterator<vector<double> >(goal_len));
  ifgl.close();
  ifstream ifgo("goal.ori");
  vector<int> goal_ori;
  copy(istream_iterator<int>(ifgo),
       istream_iterator<int>(),
       back_insert_iterator<vector<int> >(goal_ori));
  ifgl.close();
  CfgType goal(goal_base, goal_len, goal_ori);

  cout << "start:\n"; start.print(cout); cout << endl;
  cout << "goal:\n"; goal.print(cout); cout << endl;

  Query<CfgType, WeightType> query(start, goal);

  cd.ReadCommandLine(input.CDstrings, input.numCDs);
  lp.ReadCommandLine(input.LPstrings, input.numLPs, input.cdtype);
  mg.gn.ReadCommandLine(input.GNstrings, input.numGNs);
  mg.cm.ReadCommandLine(&input, &env);
  dm.ReadCommandLine(input.DMstrings, input.numDMs);

  env.SetPositionRes(mg.cm.connectionPosRes);
  env.SetOrientationRes(mg.cm.connectionOriRes);

#ifdef QUIET
#else
  input.PrintValues(cout);
#endif

  MapGenClock.StartClock("Map Generation");
  do{
    Clock_Class clock;
    clock.StartClock("Iteration Time");
    vector<CfgType> nodes;
    mg.GenerateMap(&rmap,Stats,&cd,&dm,nodes,&lp, &input);
    clock.StopPrintClock();
    char name[100];
    sprintf(name, "2arms.%d.map", rmap.m_pRoadmap->get_num_vertices());
    rmap.WriteRoadmap(&input,&cd,&dm,&lp, name);
  } while (!query.PerformQuery(&rmap, Stats, &cd, &mg.cm, &lp, &dm));
  //} while (false);
  MapGenClock.StopClock();


  //---------------------------
  // Write roadmap
  //---------------------------
  rmap.WriteRoadmap(&input,&cd,&dm,&lp);
  
  char path_filename[100];
  sprintf(path_filename, "%s.path", input.defaultFile.GetValue());
  query.WritePath(&rmap, path_filename);

  /*
  ofstream ofs("edge.query");
  CfgType start; start.GetRandomCfg(&env); start.Write(ofs); ofs << endl;
  CfgType goal; goal.GetRandomCfg(&env); goal.Write(ofs); ofs << endl;
  ofs.close();

  LPOutput<CfgType, WeightType> lpOutput;
  if(lp.IsConnected(&env, Stats, &cd, &dm,
		    start, goal,
		    &lpOutput, 0.1, 0.1, true, true, false)) {
    cout << "connected\n";
    cout << "path size is " << lpOutput.path.size() << endl;
    WritePathConfigurations("edge.path", lpOutput.path, &env);
  } else {
    cout << "not connected\n";
  }
  */
       
  
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
