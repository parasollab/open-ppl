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
#include "MyDistanceMetrics.h"

MyInput input;
Stat_Class Stats; 
void PrintRawLine( ostream& _os, Roadmap *rmap, 
                   Clock_Class *NodeGenClock, Clock_Class *ConnectionClock );
bool score_Compare(const pair<int, double>& c1, const pair<int, double>& c2);
double calScore(const Cfg &c, Environment *env);


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
  MyDistanceMetrics     dm;
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
  gn.UserInit(&input,   rmap.GetEnvironment() );
  cn.UserInit(&input,   rmap.GetEnvironment() );
  dm.UserInit(&input,   &gn, &lp );


  #ifdef QUIET
  #else
    input.PrintValues(cout);
  #endif

  // the .query provides the *goal* binding site from PDB file.
  // the .haptic file provides configurations collected thru haptic.
  char queryFile[80], hapticInputFile[80];
  vector<Cfg> query, haptic;
  sprintf(queryFile, "%s%s", input.defaultFile.GetValue(), ".query");
  sprintf(hapticInputFile, "%s%s", input.defaultFile.GetValue(), ".haptic");
  ReadCfgs(queryFile, query);
  ReadCfgs(hapticInputFile, haptic);

  if(query.empty()) {
     cout << "could not find *goal* binding site in .query file.\n"
          << "quitting ... " << endl;
     exit(2);
  }

  // OBPRM node generation
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


  vector<Cfg> vertices = rmap.roadmap.GetVerticesData(); // Node from OBPRM
  if(! haptic.empty()) // Nodes from Haptic
      vertices.insert(vertices.end(), haptic.begin(), haptic.end());
      
  // perform gradient descent for all the nodes.
  ofstream o1("pot_bf_gradDescent.m"), o2("pot_aft_gradDescent.m");
  vector<Cfg> trace, possibleBindingSite;
  Environment *env = rmap.GetEnvironment();

  int m;
  for(m=0; m<vertices.size(); ++m) {
     o1 << BioPotentials::GetPotential(vertices[m], env) << "\n";
     trace.push_back(vertices[m]);
     bool success = GradientDecent::findingLocalMin(env, trace);
     o2 << BioPotentials::GetPotential(trace.back(), env) << "\n";
     if(success)
        possibleBindingSite.push_back(trace.back());
  }

  // add these nodes to roadmap if they pass the first criterion: > 50. 
  // *goal* binding site added to roadmap too only for RMSD calc. purpose.
  vector<Cfg> pbs;
  for(m=0; m<possibleBindingSite.size(); ++m) {
     if(BioPotentials::GetPotential(possibleBindingSite[m], env) > 50.0) continue;
     rmap.roadmap.AddVertex(possibleBindingSite[m]);
     pbs.push_back(possibleBindingSite[m]);
  }
  rmap.roadmap.AddVertex(query.front()); 
  pbs.push_back(query.front()); // query[0] by itself is the binding site.

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


  // now, evaluate all the pbs nodes with second criterion: 
  // connected the largest CC (I. accessable) and II. having 
  // a good weight.
  vector< pair<int,VID> > ccstats = rmap.roadmap.GetCCStats();
  Cfg tmpC =  rmap.roadmap.GetData(ccstats[0].second);
  vector< pair<int, double> > cfgScore;
  for(m=0; m<pbs.size(); ++m) {
     // check if it is accessable, ie. in the biggest CC.
     if(!rmap.roadmap.IsSameCC(tmpC, pbs[m])) continue;
     double score = calScore(pbs[m], env);
     cfgScore.push_back(pair<int,double>(m, score));
  }
  sort(cfgScore.begin(), cfgScore.end(), ptr_fun(score_Compare));

  // output the result
  WritePathTranformationMatrices("possibleBindingSites.path", pbs, env);
  ofstream op("weight_RMSD_potential.dat");
  for(m=0; m<cfgScore.size(); ++m) {
     int index = cfgScore[m].first;
     double score = cfgScore[m].second;
     double disToGoal = dm.Distance(env, pbs[index], query.front(), -1);
     double potential = BioPotentials::GetPotential(pbs[index], env);
     op << score     << "  " << disToGoal << "  " 
        << potential << "  " << index+1   << ";\n";
  }



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

bool score_Compare(const pair<int, double>& c1, const pair<int, double>& c2) {
     return c1.second < c2.second;
}

double calScore(const Cfg &c, Environment *env) {
     static const int neb = 500;
     double totalPot = 0;
     for(int i=0; i<neb; ++i) {
        Cfg incr = Cfg::GetRandomCfg(9.0*pow(drand48(), 1.0/3), 1.0);
        Cfg tmp = c + incr;
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
     return totalPot/neb;
}

