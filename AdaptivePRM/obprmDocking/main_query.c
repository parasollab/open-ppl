///////////////////////////////////////////////////////////////////////////////
//  main_query.c		
//
///////////////////////////////////////////////////////////////////////////////

#include <iostream.h>
#include "MyQuery.h"
#include "Stat_Class.h"
#include "MyInput.h"
#include "BioPotentials.h"
#include "DynamicsLocalPlanners.h"
//#include "MyDistanceMetrics.h"
#include "GraphAlgo.h"

MyInput input;
QueryCmds Qinput;
Stat_Class Stats;

//========================================================================
//  main
//========================================================================
int main(int argc, char** argv)
{
  GenerateMapNodes   gn;
  ConnectMapNodes    cn;
  DynamicsLocalPlanners      lp;
  //MyDistanceMetrics     dm;
  DistanceMetric     dm;
  CollisionDetection cd;

  //----------------------------------------------------
  // instantiate query/roadmap object
  // 	parse command line and init roadmap, lps, read in environment, etc
  //----------------------------------------------------
  Qinput.ReadCommandLine(&argc,argv);
  input.ReadCommandLine(argc,argv);

  MyQuery query(&input,&Qinput, &cd, &dm, &lp,&cn);

  cd.UserInit(&input,   &gn, &cn );
  lp.UserInit(&input,        &cn );
  dm.UserInit(&input,   &gn, &lp );
  gn.UserInit(&input,  query.rdmp.GetEnvironment() );
  cn.UserInit(&input,  query.rdmp.GetEnvironment() );

  query.initDefaultSetIDs(&cn);

  Qinput.PrintValues(cout);

  //---------------------------
  // Print out some useful info
  //---------------------------
  lp.planners.DisplayLPs();
  cout << endl;
  lp.planners.DisplayLPSets();
  cout << endl;
  DisplayCCStats(*(query.rdmp.m_pRoadmap),10);
  cout << endl;

  cout << "\nStart Cfg is " << query.query[0] << endl;
  cout << "And its potential is " << BioPotentials::GetPotential(query.query[0], 
		query.rdmp.GetEnvironment()) << endl;
  cout << "\nGoal Cfg is " << query.query[query.query.size()-1] << endl;
  cout << "And its potential is " << BioPotentials::GetPotential(
	query.query[query.query.size()-1], query.rdmp.GetEnvironment()) << endl;

  ofstream osp("potentialDist.m");
  ofstream osr("rmsdDist.m");
  ofstream os2("rmsdDist2.m");
  vector<Cfg> vertices;
  query.rdmp.m_pRoadmap->GetVerticesData(vertices);
  int i;
  for(i=0; i<vertices.size(); ++i) {
     osp << BioPotentials::GetPotential(vertices[i], query.rdmp.GetEnvironment()) << "\n";
     osr << dm.Distance(query.rdmp.GetEnvironment(), vertices[i], 
            query.query[query.query.size()-1], 0) << "\n";
     os2 << dm.Distance(query.rdmp.GetEnvironment(), vertices[i],
	    vertices[10], 0) << "\n";

  }

  //----------------------------------------------------
  // perform the query
  // if successful, write path to a file
  //----------------------------------------------------
  if ( query.PerformQuery(&cd,&cn,&lp,&dm) ) {
    query.WritePath();
    cout << endl << "SUCCESSFUL query";
    query.rdmp.WriteRoadmap(&input,&cd,&dm,&lp);
  } else {
    cout << endl << "UNSUCCESSFUL query";
  }

  //------------------------
  // Done
  //------------------------
  cout << "\n  !!Bye!! \n";
  return 0;
}
