///////////////////////////////////////////////////////////////////////////////
//  car_aprm.c		
//
///////////////////////////////////////////////////////////////////////////////

#include <iostream.h>

#include "SwitchDefines.h"

#include "OBPRM.h"
#include "Roadmap.h"
#include "MyInput.h"

#include "CfgCarLike.h"
#include "NoopLocalPlanners.h"
#include "NoopCollisionDetection.h"
#include "util.h"
#include "CurveWeight.h"

#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMapNodes.h"
#include "DistanceMetrics.h"

MyInput input;
Stat_Class Stats; 
void PrintRawLine( ostream& _os,
        Roadmap *rmap, Clock_Class *NodeGenClock, Clock_Class *ConnectionClock,
        ConnectMapNodes cn, int printHeaders);

//========================================================================
//  main
//========================================================================
int main(int argc, char** argv)
{
  GenerateMapNodes   gn;
  ConnectMapNodes    cn;
  LocalPlanners      lp;
  NoopLocalPlanners  noop_lp;
  DistanceMetric     dm;
  CollisionDetection cd;
  NoopCollisionDetection noop_cd;
  Clock_Class        NodeGenClock;
  Clock_Class        ConnectionClock;

  //----------------------------------------------------
  // Get information from the user via the command line
  //----------------------------------------------------
  input.ReadCommandLine(argc,argv);

  CurveWeightFactory* fact = new CurveWeightFactory();
  WeightObject::SetWeightFactory(fact);

  // do not seed for now while developing code... ADD SEEDING LATER
  //    srand48((unsigned int) time(NULL));

  //----------------------------------------------------
  // instantiate roadmap object 
  //	parse command line and init roadmap, lps, read in environment, etc
  // initiate collision detection
  // build map
  // write map to a file
  //----------------------------------------------------
  Roadmap rmap(&input, &cd, &dm, &lp);
  cd.UserInit(&input, &gn, &cn);
  noop_cd.UserInit(&input, &gn, &cn);
  lp.UserInit(&input, &cn);
  noop_lp.UserInit(&input, &cn);
  gn.UserInit(&input, rmap.GetEnvironment());
  cn.UserInit(&input, rmap.GetEnvironment());
  dm.UserInit(&input, &gn, &lp);

  enum VALIDATE {NONE, APPROXIMATE, COMPLETE};

  VALIDATE nodeValidateType = COMPLETE;
  if(!strncmp(input.nodeValidationFlag.GetValue(),"none",4)) {
    nodeValidateType = NONE;
  } else if(!strncmp(input.nodeValidationFlag.GetValue(),"approximate",11)) {
    nodeValidateType = APPROXIMATE;
  }

  VALIDATE edgeValidateType = COMPLETE;
  if(!strncmp(input.edgeValidationFlag.GetValue(),"none",4)) {
    edgeValidateType = NONE;
  } else if (!strncmp(input.edgeValidationFlag.GetValue(),"approximate",11)) {
    edgeValidateType = APPROXIMATE;
  }

  // initial CfgHelper at here.
  Cfg::CfgHelper = new CfgCarLike();
  CfgCarLike::turningRadius = 4.0;

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
    switch(nodeValidateType) {
    case NONE:
      gn.GenerateNodes(&rmap,&noop_cd,&dm, gn.gnInfo.gnsetid, gn.gnInfo);
      break;
    case APPROXIMATE:
    case COMPLETE:
      gn.GenerateNodes(&rmap,&cd,&dm, gn.gnInfo.gnsetid, gn.gnInfo);
      break;
    }
    NodeGenClock.StopClock();
  }


  #if QUIET
  #else
    cout << "\n";
    if ( input.inmapFile.IsActivated() ){
      cout << "Node Generation: "<<argv[argc];
    }else{
      cout << ": " << NodeGenClock.GetClock_SEC()
           << " sec (ie, " << NodeGenClock.GetClock_USEC() << " usec)";
    }
    
      cout << ", "<<rmap.m_pRoadmap->GetVertexCount()<<" nodes\n"<< flush;
  #endif


  //---------------------------
  // Connect roadmap nodes
  //---------------------------
  ConnectionClock.StartClock("Node Connection");
  switch(edgeValidateType) {
  case NONE:
    cn.ConnectNodes(&rmap,&noop_cd,&noop_lp,&dm, cn.cnInfo.cnsetid, cn.cnInfo);
    break;
  case APPROXIMATE:
    cn.setConnectionResolution(rmap.GetEnvironment()->GetPositionRes()*10,
			       rmap.GetEnvironment()->GetOrientationRes()*10);
  case COMPLETE:
    cn.ConnectNodes(&rmap,&cd,&lp,&dm, cn.cnInfo.cnsetid, cn.cnInfo);
    break;
  }
  ConnectionClock.StopClock();


  //---------------------------
  // Write control roadmap
  //---------------------------


  //***************************************
  // new stuff for car-like node generation and edge construction.
  //*************************************************************
  RoadmapGraph<Cfg,WEIGHT>* newmap = new RoadmapGraph<Cfg,WEIGHT>;
  /*
  vector< pair<pair<Cfg,Cfg>, WEIGHT> > elist = rmap.roadmap.GetEdgesVData();
  for(int i=0; i<elist.size(); ++i) {
      pair<Cfg,Cfg> directedCfg = CfgCarLike::GetDirectedCfg(elist[i].first);
      newmap.AddVertex(directedCfg.first);
      newmap.AddVertex(directedCfg.second);
  }
  */

  vector<VID> vids;
  rmap.m_pRoadmap->GetVerticesVID(vids);
  CDInfo cdinfo;
  for(int i=0; i<vids.size(); ++i) {
      vector< pair<pair<Cfg,Cfg>, WEIGHT> > neb;
      rmap.m_pRoadmap->GetIncidentEdgesVData(vids[i], neb);

      vector< pair<Cfg,Cfg> > directedCfg(neb.size());
      for(int j=0; j<neb.size(); ++j) {
      	  directedCfg[j] = CfgCarLike::GetDirectedCfg(neb[j].first);
	  bool isfree = false;
	  if(!directedCfg[j].first.isCollision(rmap.GetEnvironment(),&cd, cn.cnInfo.cdsetid, cdinfo)) {
             newmap->AddVertex(directedCfg[j].first);
	     isfree = true;
	  }
	  if(!directedCfg[j].second.isCollision(rmap.GetEnvironment(),&cd, cn.cnInfo.cdsetid, cdinfo)) {
             newmap->AddVertex(directedCfg[j].second);
	     isfree = true;
	  }
	  if(!isfree) 
             rmap.m_pRoadmap->DeleteEdge(neb[j].first.first, neb[j].first.second);
      }
      for(int j=0; j<neb.size(); ++j) {
         for(int k=j+1; k<neb.size(); ++k) {
	    CfgCarLike::AddCurvedEdges(newmap, directedCfg[j], directedCfg[k], neb[0].first.first); //should this be i instead of 0?
         }
      }
  }

  // output control roadmap
  rmap.WriteRoadmap(&input,&cd,&dm,&lp, "control.map");

  rmap.m_pRoadmap = newmap;


  //---------------------------
  // Write roadmap
  //---------------------------
  rmap.WriteRoadmap(&input,&cd,&dm,&lp);
  vector<Cfg> newdata;
  rmap.m_pRoadmap->GetVerticesData(newdata);
  WritePathConfigurations("newNodes.path", newdata, rmap.GetEnvironment());


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
        &rmap, &NodeGenClock,&ConnectionClock,cn,1);  // to stdout
    PrintRawLine(myofstream,
        &rmap, &NodeGenClock,&ConnectionClock,cn,0);  // to map
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
        Roadmap *rmap, Clock_Class *NodeGenClock, Clock_Class *ConnectionClock,
        ConnectMapNodes cn,int printHeader = 0 ){

  _os << "\ndups  " <<cn.cnInfo.dupeNodes<<" "<<cn.cnInfo.dupeEdges;

  _os << "\nraw ";               // We can grep out "raw" datalines.
  _os << NodeGenClock->GetClock_SEC()     << " ";
  _os << NodeGenClock->GetClock_USEC()    << " ";
  _os << ConnectionClock->GetClock_SEC()  << " ";
  _os << ConnectionClock->GetClock_USEC() << " ";
  Stats.PrintDataLine(_os,rmap,printHeader);
  _os << "\n\n";
}
