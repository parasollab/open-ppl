///////////////////////////////////////////////////////////////////////////////
//  main_assembly.cpp		
//  9/19/00 Sujay
///////////////////////////////////////////////////////////////////////////////

#include <iostream.h>

#include <string.h>
#include <stdlib.h>

#include "SwitchDefines.h"

#include "OBPRM.h"
#include "Roadmap.h"
#include "Input.h"


#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMapNodes.h"
#include "Query.h"

#include "Removal_Directions.h"

#define DIRECTIONS_ALONG_NORMALS 1


Input input;
Stat_Class Stats; 


LPInfo Initialize_LPinfo(Roadmap * _rm,CNInfo& info);

bool isDisassembled(Cfg *_cfg, Roadmap * _rm);

// Method returning the distance between 2 bodies
double cstkDistance(void * body1, void * body2);


//========================================================================
//  main
//========================================================================
int main(int argc, char** argv)
{
  GenerateMapNodes   gn;
  ConnectMapNodes    cn;
  LocalPlanners      lp;
  DistanceMetric     dm;
  CollisionDetection cd;
  Clock_Class        RoadmapClock;
  Clock_Class        RetraceClock;

  Query              qry;
  
  //----------------------------------------------------
  // Get information from the user via the command line
  //----------------------------------------------------
  input.ReadCommandLine(argc,argv);


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
  gn.UserInit(&input,                  rmap.GetEnvironment() );
  cn.UserInit(&input);
  dm.UserInit(&input,   &gn, &lp );

//	rmap.InitEnvironment(&input);

  qry.rdmp = rmap;
//	qry.path = NULL;

  #ifdef QUIET
  #else
    input.PrintValues(cout);
  #endif

// Get number of nodes to be generated
  int iterations = input.numNodes.GetValue();

// Get init info from the query file
  char filename[80];
  strcpy(filename, input.defaultFile.GetValue() );
  strcat(filename,".query");

  char pathfilename[80];
  strcpy(pathfilename, input.defaultFile.GetValue() );
  strcat(pathfilename,".path");

  qry.ReadQuery(filename);
  Cfg initcfg;
  initcfg.SetData(qry.query[0].GetData());
//  cout << "INIT CFG" << initcfg <<endl;

  cout << "Init Cfg in Collision :" << initcfg.isCollision(rmap.environment, &cd, cn.cnInfo.cdsetid, cd.cdInfo) << endl;


//-- initialize information needed to check connection between cfg's
  LPInfo lpInfo=Initialize_LPinfo(&rmap,cn.cnInfo);
  lpInfo.savePath = false;

// Main Algorithm for disassembly planning

  vector<pair<Cfg, VID> >   stack;

  // Need to get the initial cfg of the objects. This may have to be pulled out
  // from the query file to be in accordance with what is done in obprm.

  // TO DO: add the init cfg to the 'roadmap', edge set is empty initially 

  bool disassembly;
// assign cfg here,
  Cfg cfg = initcfg;
  VID cfg_VID = rmap.roadmap.AddVertex(cfg);
// as well as put it in the stack.
  pair<Cfg, VID> pr;
  pr.first = cfg;
  pr.second = cfg_VID;
  stack.push_back(pr);

  VID search_VID = cfg_VID;

  vector<pair<SID,vector<LP> > > sets = lp.planners.GetLPSets();

  MultiBody * mb = 
    rmap.GetEnvironment()->GetMultiBody(rmap.GetEnvironment()->GetRobotIndex());
  int numbodies = mb->GetFreeBodyCount();

// Start CLOCK
  RoadmapClock.StartClock("Total Time to generate the roadmap");

  while ( !stack.empty() && !(disassembly = isDisassembled(&cfg, &rmap)) && 
          iterations > 0) {
    cout << "Size of stack =" << stack.size() << endl;
    pair<Cfg, VID> lastpr = stack.back();
    cfg = lastpr.first;
    cfg_VID = lastpr.second;
    stack.pop_back();
    vector<Cfg> newcfgs = Removal_Directions::expand(&cfg, &rmap, cd, cn, 
                            DIRECTIONS_ALONG_NORMALS);
// expand obviously has to check if an edge can be created b/w cfg & newcfgs

    cout << "SIZE of newcfgs: " << newcfgs.size() << endl;

    for (int i = 0; i < newcfgs.size(); i++) {
    // Check connection with all LPsets to increase chances of a success
      for (int j = 0; j < sets.size(); j++) {
        SID setid = sets[j].first;
        if ( lp.IsConnected(&rmap,&cd,&dm,cfg,newcfgs[i],setid,&lpInfo) ) {
          VID newcfg_VID;
          if (!rmap.roadmap.IsVertex(newcfgs[i])){
            newcfg_VID = rmap.roadmap.AddVertex(newcfgs[i]);
            pair<Cfg, VID> temppair;
            temppair.first = newcfgs[i];
            temppair.second = newcfg_VID;
            stack.push_back(temppair);
          }
          else
            newcfg_VID = rmap.roadmap.GetVID(newcfgs[i]);
          if (!rmap.roadmap.IsEdge(cfg_VID, newcfg_VID)) {
            rmap.roadmap.AddEdge(cfg_VID, newcfg_VID, lpInfo.edge.first);
            search_VID = newcfg_VID;  							            // to keep track of what is the last vertex to be connected
            break;	 // to ensure that only one connection is made
          }
        }
      }
    }
    iterations--;
  }

// Stop CLOCK
  RoadmapClock.StopClock();

  if (disassembly) 
    cout << "Disassembly successful !!!" << endl;
  else 
    cout << "Disassembly unsuccessful" << endl;

// Next step is to create a path from the roadmap if disassembly is successful

  vector <pair <pair <VID, VID>, WEIGHT> > rmedges = rmap.roadmap.GetEdges();
  bool flag;

  lpInfo.path.erase(lpInfo.path.begin(),lpInfo.path.end());
  lpInfo.savePath = true;

  // Start CLOCK
  RetraceClock.StartClock("Time to retrace path");

  int numedges = 0;

  while (true) {
    flag = false;
    int i;
    for (i = 0; i < rmedges.size(); i++) {
      if (rmedges[i].first.second == search_VID) {
        cout << " FOUUUUUUUUUUND" << endl;
        search_VID = rmedges[i].first.first;
        bool x;
        Cfg cfg1 = rmap.roadmap.GetData(rmedges[i].first.first);
        Cfg cfg2 = rmap.roadmap.GetData(rmedges[i].first.second);
        x = lp.IsConnected(rmap.environment, &cd, &dm, cfg1, 
                           cfg2,cn.cnInfo.lpsetid, &lpInfo);
	if (x){
          cout << "PATH SEGMENT FOUND" << endl;
//        cout << rmap.roadmap.GetData(search_VID) << endl;
//        cout << rmap.roadmap.GetData(rmedges[i].first.second) << endl;
        }

//      cout << lpInfo.path.size() << "SIZE " << endl;
//      cout << qry.path.size() << "SIZE " << endl;

        qry.path.insert(qry.path.begin(), lpInfo.path.begin(), 
                          lpInfo.path.end());

        lpInfo.path.erase(lpInfo.path.begin(),lpInfo.path.end());
        flag = true;
        cout << " FOUUUUUUUUUUND END" << endl;
        numedges++;
        break;
      }
    }
    if (!flag)
      break;
  }

  // Stop CLOCK
  RetraceClock.StopClock();

  qry.WritePath(pathfilename);

// Useful Data

  RoadmapClock.PrintName();
  cout << ": " << RoadmapClock.GetClock_SEC() << " sec"
         << ", "<<rmap.roadmap.GetEdgeCount()<<" edges\n"<< flush;

  RetraceClock.PrintName();
  cout << ": " << RetraceClock.GetClock_SEC() << " sec\n" << flush;

  Stats.PrintAllStats(&rmap);

  cout << "Number of edges in the disassembly path: " << numedges << flush;

  //------------------------
  // Done
  //------------------------
  #ifdef QUIET
  #else
    cout << "\n  !!Bye!! \n";
  #endif
  return 0;

}


// ------------------------------------------------------------------
// some info needs to be set up for check connection between cfg's
// ------------------------------------------------------------------
LPInfo
Initialize_LPinfo(Roadmap * _rm,CNInfo& info){

  LPInfo lpInfo(_rm,info);

  lpInfo.positionRes    = _rm->GetEnvironment()->GetPositionRes();
  lpInfo.checkCollision = true;
  lpInfo.savePath       = false;
  lpInfo.cdsetid        = info.cdsetid;
  lpInfo.dmsetid        = info.dmsetid;

  return lpInfo;

}


bool isDisassembled(Cfg *_cfg, Roadmap* _rm) {
  _cfg->ConfigEnvironment(_rm->environment);
  MultiBody * robot = 
    _rm->environment->GetMultiBody(_rm->environment->GetRobotIndex());
  for (int i = 0; i < robot->GetFreeBodyCount(); i++) {
    void * body1 = robot->GetFreeBody(i)->GetCstkBody();
    for (int j = i + 1; j < robot->GetFreeBodyCount(); j++) {
      void * body2 = robot->GetFreeBody(j)->GetCstkBody();
      if (cstkDistance(body1, body2) < CLEARANCE)
        return false;
    }
    // NOTE: This is assuming that robot index is always 1
    for (int m = 1; m < _rm->GetEnvironment()->GetMultiBodyCount(); m++) {
      for (int k = 0; k < 
           _rm->GetEnvironment()->GetMultiBody(m)->GetFixedBodyCount(); k++) {		        void * body3 =   
        _rm->GetEnvironment()->GetMultiBody(m)->GetFixedBody(k)->GetCstkBody();
        if (cstkDistance(body1, body3) < CLEARANCE)
          return false;
      }
    }	
  }
  return true;

}


double cstkDistance(void * body1, void * body2) {
  return cstkBodyBodyDist(body1, body2, 500, 0, NULL, 0, NULL);
}
