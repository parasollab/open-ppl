///////////////////////////////////////////////////////////////////////////////
//  check_path.c		
//
///////////////////////////////////////////////////////////////////////////////

#include <iostream.h>
#include "MyQueryCmds.h"
#include "MyInput.h"
#include "QueryRequirements.h"
#include "Cfg.h"

#include "AdaptiveQuery.h"
#include "util.h"

MyInput input;
MyQueryCmds Qinput;


//========================================================================
//  main
//========================================================================
int main(int argc, char** argv)
{
  CollisionDetection cd;
  GenerateMapNodes gn;
  ConnectMapNodes cn;
  DistanceMetric dm;
  LocalPlanners lp;
  Environment* env;

  //----------------------------------------------------
  // instantiate query/roadmap object
  // 	parse command line and init roadmap, lps, read in environment, etc
  //----------------------------------------------------
  Qinput.ReadCommandLine(&argc,argv);
  input.ReadCommandLine(argc,argv);

  AdaptiveQuery query(&input, &Qinput, &cd, &dm, &lp, &cn);
  env = query.rdmp.GetEnvironment();
  cd.UserInit(&input,   &gn, &cn );
  lp.UserInit(&input,        &cn );
  dm.UserInit(&input,   &gn, &lp );
  gn.UserInit(&input,  query.rdmp.GetEnvironment() );
  cn.UserInit(&input,  query.rdmp.GetEnvironment() ); 
  query.initDefaultSetIDs(&cn);
   
  char name[80];
  strcpy(name, Qinput.queryFile.GetValue());
  strcat(name, ".requirement"); 
  QueryRequirementsObject queryReq(name);

  char fname[200];
  sprintf(fname,"%s",Qinput.pathFile.GetValue());
  //cout << fname << endl;
  VerifyFileExists(fname, EXIT);
  ifstream is(fname);
  char trash[256];
  is.getline(trash, 256, '\n');
  is.getline(trash, 256, '\n');
  int num_steps;
  if ( !(is >> num_steps) ) {
    cout << "ERROR\n";
    return 0;
  }

  //------------------------
  // Read in path
  //------------------------

  vector<Cfg> path;

  for(int i=0; i<num_steps; i++) {
    Cfg tmp;
    tmp.Read(is);
    path.push_back(tmp);
  }
 

  //------------------------
  // Check nodes
  //------------------------

  for(int i=0; i<path.size(); i++)
    if( !queryReq.isNodeValid(path[i], env, &cd, query.cdsetid) ) {
      cout << "\nFound bad node: " << path[i] << endl;
      return 0;
    }
  cout << "\nNodes meet requirements.";


  //------------------------
  // Check edges
  //------------------------

  LPInfo info;
  info.positionRes = query.rdmp.GetEnvironment()->GetPositionRes();
  info.orientationRes = query.rdmp.GetEnvironment()->GetOrientationRes();
  info.checkCollision = true;
  info.cdsetid = query.cdsetid;
  info.dmsetid = query.dmsetid;
  info.savePath = false;

  for(int i=0; i<path.size()-1; i++) {
    if(!query.GetPathSegment(path[i], path[i+1], &cd, &lp, &dm, 
			     query.rdmp.m_pRoadmap->GetEdgeWeight(path[i],path[i+1]),
			     &info)) {
      cout << "\nFound edge in-collision: " << path[i] << "-> " << path[i+1] << endl;
      return 0;
    }

    vector<Cfg> edge;
    edge.push_back(path[i]);
    edge.push_back(path[i+1]);
    if( !queryReq.isEdgeValid(edge, env, &cd, query.cdsetid) ) {
      cout << "\nFound bad edge: " << path[i] << " -> " << path[i+1] << endl;
      return 0;
    }
  }
  cout << "\nEdges meet requirements.";


  //------------------------
  // Done
  //------------------------
  cout << "\n  !!Bye!! \n";
  return 0;
}





