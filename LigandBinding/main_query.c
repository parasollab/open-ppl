// $Id$
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
#include "MyDistanceMetrics.h"
#include "GradientDecent.h"

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
  MyDistanceMetrics     dm;
  //DistanceMetric     dm;
  CollisionDetection cd;

  //----------------------------------------------------
  // instantiate query/roadmap object
  // 	parse command line and init roadmap, lps, read in environment, etc
  //----------------------------------------------------
  Qinput.ReadCommandLine(&argc,argv);
  input.ReadCommandLine(argc,argv);

  cd.UserInit(&input,   &gn, &cn );
  lp.UserInit(&input,        &cn );
  cn.UserInit(&input);
  dm.UserInit(&input,   &gn, &lp );

  MyQuery query(&input,&Qinput, &cd, &dm, &lp,&cn);

  gn.UserInit(&input,           query.rdmp.GetEnvironment() );


  Qinput.PrintValues(cout);

  //---------------------------
  // Print out some useful info
  //---------------------------
  lp.planners.DisplayLPs();
  cout << endl;
  lp.planners.DisplayLPSets();
  cout << endl;
  query.rdmp.roadmap.DisplayCCStats(10);
  cout << endl;

  Environment *env = query.rdmp.GetEnvironment();
  vector<Cfg> possibleBindingSite;
  vector<Cfg> trace;
  vector<Cfg> vertices = query.rdmp.roadmap.GetVerticesData();
  ofstream o1("pot1.m"), o2("pot2.m");

  for(int k=0; k<query.query.size(); ++k) {
     vertices.push_back(query.query[k]);
  }
  for(int m=0; m<vertices.size(); ++m) {
     o1 << BioPotentials::GetPotential(vertices[m], env) << "\n";
     trace.push_back(vertices[m]);
     bool success = GradientDecent::findingLocalMin(env, trace);
     o2 << BioPotentials::GetPotential(trace[trace.size()-1], env) << "\n";
     if(success)
        possibleBindingSite.push_back(trace[trace.size()-1]);
  }
  WritePathTranformationMatrices("bindingsites.path", possibleBindingSite,
                 env);

  ofstream op("index_weight");
  vector<Cfg> &pbs = possibleBindingSite; // short name
  int neb = 500;
  for(m=0; m<pbs.size(); ++m) {
     //rmap.roadmap.AddVertex(pbs[m]);
     if(BioPotentials::GetPotential(pbs[m], env) > 50.0) continue; // used:  -5: 50.
     double totalPot = 0;
     for(int i=0; i<neb; ++i) {
        Cfg incr = Cfg::GetRandomCfg(9.0*pow(drand48(), 1.0/3), 1.0);
        Cfg tmp = pbs[m] + incr;
        double tmpPotential = BioPotentials::GetPotential(tmp, env);
        totalPot += tmpPotential > 5000 ? 5000 : tmpPotential;
        //double pathweights = 0.0;
        //if(tmpPotential < BioPotentials::ConnectionThreshold() ) {
          // if(lp.IsConnected(&rmap,cd,dm,possibleBindingSite[m],
        //      tmp, lpsid, &ci) {
        //      rmap.roadmap.AddVertex(tmp);
        //      rmap.roadmap.AddEdge(possibleBindingSite[m],tmp, ci.edge);
        //}
     }
     double dis = dm.Distance(query.rdmp.GetEnvironment(), pbs[m], query.query[0], -1);
     op << m << "    " << dis  << "    "  << totalPot/neb << "  " 
	<< BioPotentials::GetPotential(pbs[m], env) << ";\n";
  }


  cout << "\nStart Cfg is " << query.query[0] << endl;
  cout << "And its potential is " << BioPotentials::GetPotential(query.query[0], 
		query.rdmp.GetEnvironment()) << endl;
  vector<Vector3D> tmpv = BioPotentials::GetCoordinatesLigandBinding(query.query[0],
                query.rdmp.GetEnvironment());
  for(m=0; m<tmpv.size(); ++m) {
	cout << tmpv[m] << endl;
  }
  cout << "\nGoal Cfg is " << query.query[query.query.size()-1] << endl;
  cout << "And its potential is " << BioPotentials::GetPotential(
	query.query[query.query.size()-1], query.rdmp.GetEnvironment()) << endl;

#if 0
  ofstream osp("potentialDist.m");
  ofstream osr("rmsdDist.m");
  ofstream os2("rmsdDist2.m");
  vertices = query.rdmp.roadmap.GetVerticesData();
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
#endif
  //------------------------
  // Done
  //------------------------
  cout << "\n  !!Bye!! \n";
  return 0;
}
