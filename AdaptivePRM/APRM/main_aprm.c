///////////////////////////////////////////////////////////////////////////////
//  main_aprm.c		
//
///////////////////////////////////////////////////////////////////////////////

#include <iostream.h>

#include "SwitchDefines.h"

#include "OBPRM.h"
#include "Roadmap.h"
#include "MyInput.h"


#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "NoopCollisionDetection.h"
#include "NoopLocalPlanners.h"
#include "LocalPlanners.h"
#include "ConnectMapNodes.h"
#include "PriorityWeight.h"

#include "GMSPolyhedron.h"
#include "MultiBody.h"

extern "C"{
#include <stdio.h>
#include "qhull.h"
#include "poly.h"
#include "qset.h"

char qh_version[] = "Hull";
}

MyInput input;
Stat_Class Stats; 
void PrintRawLine( ostream& _os,
        Roadmap *rmap, Clock_Class *NodeGenClock, Clock_Class *ConnectionClock,
        ConnectMapNodes cn, int printHeaders);

GMSPolyhedron* createBBoxPolyhedron(MultiBody* mb);
GMSPolyhedron* createBSpherePolyhedron(MultiBody* mb);
GMSPolyhedron* createHullPolyhedron(MultiBody* mb);

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
  Clock_Class        ApproxEnvClock;
  Clock_Class        NodeGenClock;
  Clock_Class        ConnectionClock;

  //----------------------------------------------------
  // Get information from the user via the command line
  //----------------------------------------------------
  input.ReadCommandLine(argc,argv);

  PriorityWeightFactory* fact = new PriorityWeightFactory();
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

  //read in validate flags
  enum VALIDATE {NONE, APPROXIMATE, COMPLETE};
  enum APPROX_TYPE {BOX, SPHERE, HULL};

  VALIDATE nodeValidateType = COMPLETE;
  APPROX_TYPE nodeApproximationType = SPHERE;

  if(!strncmp(input.nodeValidationFlag.GetValue(),"none",4)) {
    nodeValidateType = NONE;
  } else if(!strncmp(input.nodeValidationFlag.GetValue(),"approximate",11)) {
    nodeValidateType = APPROXIMATE;

    if(!strncmp(input.nodeValidationFlag.GetValue(),"approximate box",15)) {
      nodeApproximationType = BOX;
    } else if(!strncmp(input.nodeValidationFlag.GetValue(),"approximate hull",16)) {
      nodeApproximationType = HULL;
    }
  }

  double resolution;
  VALIDATE edgeValidateType = COMPLETE;
  if(!strncmp(input.edgeValidationFlag.GetValue(),"none",4)) {
    edgeValidateType = NONE;
  } else if(!strncmp(input.edgeValidationFlag.GetValue(),"approximate",11)) {
    edgeValidateType = APPROXIMATE;
    char tmpType[20];
    istrstream is(input.edgeValidationFlag.GetValue());
    is >> tmpType; //read in "approximate"
    if(is >> resolution) {
      if(resolution<0 || resolution>1) { //out of range
	cout << "\n\nERROR: resolution out of range! Must be [0,1].\n";
	exit(-1);
      }
    } else {
      resolution = 0.1; //default
    }
  }

  #ifdef QUIET
  #else
    input.PrintValues(cout);
  #endif


  //if approximating node validation, calculate new environment
  if(nodeValidateType == APPROXIMATE) {
    ApproxEnvClock.StartClock("Approximate Env Setup");
    Environment* realEnv = rmap.GetEnvironment();
    Environment* approxEnv = new Environment();

    for(int i=0; i<realEnv->GetMultiBodyCount(); i++) {
      MultiBody* realMB = realEnv->GetMultiBody(i);
      MultiBody* approxMB = new MultiBody(approxEnv);
      GMSPolyhedron P;

      switch(nodeApproximationType) {
      case BOX:
	P = *(createBBoxPolyhedron(realMB));
	break;
      case SPHERE:
	P = *(createBSpherePolyhedron(realMB));
	break;
      case HULL:
	P = *(createHullPolyhedron(realMB));
	break;
      }

      /*
      char filename[100];
      sprintf(filename,"shape%d.g",i);
      ofstream os(filename);
      P.WriteBYU(os);
      */

      if( realMB->GetFreeBodyCount() ) {
	FreeBody* freeBody = new FreeBody(approxMB,P);
	freeBody->buildCDstructure(input.cdtype);
	approxMB->AddBody(freeBody);
      } else {
	FixedBody* fixedBody = new FixedBody(approxMB,P);
	fixedBody->buildCDstructure(input.cdtype);
	approxMB->AddBody(fixedBody);
      }
      approxMB->CalculateArea();
      approxEnv->AddMultiBody(approxMB);
    }

    approxEnv->SetRobotIndex( realEnv->GetRobotIndex() );

    approxEnv->FindBoundingBox();
    approxEnv->UpdateBoundingBox(&input);
    if ( input.posres.IsActivated() )
      approxEnv->SetPositionRes( input.posres.GetValue() );
    approxEnv->SetOrientationRes( input.orires.GetValue() ); 

    rmap.InitEnvironment(approxEnv);

    ApproxEnvClock.StopClock();

    #if QUIET
    #else
      cout << "\n";
      ApproxEnvClock.PrintName();
      cout << ": " << ApproxEnvClock.GetClock_SEC()
	   << " sec (ie, " << ApproxEnvClock.GetClock_USEC() << " usec)"
	   << endl;
    #endif
  }


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
    cn.setConnectionResolution(rmap.GetEnvironment()->GetPositionRes()/resolution,
			       rmap.GetEnvironment()->GetOrientationRes()/resolution);
  case COMPLETE:
    cn.ConnectNodes(&rmap,&cd,&lp,&dm, cn.cnInfo.cnsetid, cn.cnInfo);
    break;
  }
  ConnectionClock.StopClock();

  //---------------------------
  // Write roadmap
  //---------------------------
  rmap.WriteRoadmap(&input,&cd,&dm,&lp);


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


GMSPolyhedron* createBBoxPolyhedron(MultiBody* mb) {
  GMSPolyhedron* p = new GMSPolyhedron();

  //find bounding box coords
  mb->FindBoundingBox();
  double* bbox = mb->GetBoundingBox();
  double minx, maxx, miny, maxy, minz, maxz;
  minx = bbox[0]; maxx = bbox[1];
  miny = bbox[2]; maxy = bbox[3];
  minz = bbox[4]; maxz = bbox[5];

  //create polyhedron file in BYU format
  char polyhedron[1000];
  sprintf(polyhedron,"1 8 12 1 1 1\n%f %f %f\n%f %f %f\n%f %f %f\n%f %f %f\n%f %f %f\n%f %f %f\n%f %f %f\n%f %f %f\n8 4 -2\n8 2 -6\n8 7 -3\n8 3 -4\n8 6 -5\n8 5 -7\n4 3 -1\n4 1 -2\n7 5 -1\n7 1 -3\n6 2 -1\n6 1 -5\n",minx,miny,minz,minx,miny,maxz,minx,maxy,minz,minx,maxy,maxz,maxx,miny,minz,maxx,miny,maxz,maxx,maxy,minz,maxx,maxy,maxz);

  //init polyhedron
  istrstream is(polyhedron);
  p->ReadBYU(is);
  //p->Write(cout); 
  
  return p;
}


GMSPolyhedron* createBSpherePolyhedron(MultiBody* mb) {
  GMSPolyhedron* p = new GMSPolyhedron();
  return p;
}


GMSPolyhedron* createHullPolyhedron(MultiBody* mb) {
  GMSPolyhedron* p = new GMSPolyhedron();

  //set up qhull variables
  int curlong, totlong, exitcode;
  static char* options="qhull Qs Qx QJ i Tcv C-0";
  facetT *facet;
  vertexT *vertex;
  vertexT **vertexp;
  setT *vertices;
  
  //put points in qhullData array
  int size = 0;
  for (int i=0; i<mb->GetBodyCount(); i++)
    size += mb->GetBody(i)->GetPolyhedron().numVertices;
  coordT* qhullData = new coordT[size*3];
  for(int i=0; i<mb->GetBodyCount(); i++) {
    GMSPolyhedron* real_p = &(mb->GetBody(i)->GetPolyhedron());
    for(int j=0; j<real_p->numVertices; j++) {
      qhullData[j*3+0] = real_p->vertexList[j].getX();
      qhullData[j*3+1] = real_p->vertexList[j].getY();
      qhullData[j*3+2] = real_p->vertexList[j].getZ();
    }
  }
  
  //initialize qh
  qh_init_A(stdin, stdout, stderr, 0, NULL);
  exitcode = setjmp(qh errexit);
  if(exitcode) {
    cerr << "error building convex hull of Polyhedron \a" << endl;
    cerr << "exitcode: " << exitcode << endl;
    delete qhullData;
    qh NOerrexit = True;
    qh_freeqhull(!qh_ALL);
    qh_memfreeshort(&curlong, &totlong);
    exit(-1);
  }
  qh_initflags(options);
  qh_init_B(qhullData, size, 3, false);
  
  //run
  qh_qhull();
  qh_check_output();
  
  //copy points into vertex list
  vector<Vector3D> polyVertices;
  FORALLvertices {
    Vector3D tmp(vertex->point[0], vertex->point[1], vertex->point[2]);
    polyVertices.push_back(tmp);
  }
    
  //copy polygon surfaces
  vector<vector<int> > polyFacets;
  int count = 0;
  FORALLfacets {
    vector<int> tmpFacetList;
    vertices = qh_facet3vertex(facet);
    FOREACHvertex_(vertices) {
      Vector3D tmp(vertex->point[0], vertex->point[1], vertex->point[2]);
      //find index
      int index = -1;
      for (int i=0; i<polyVertices.size(); i++) {
	if(polyVertices[i] == tmp) {
	  index = i;
	  break;
	}
      }
      if(index == -1) {
	cout << "\n\nERROR: in createHullPolyhedron, didn't find vertex in list\n";
	exit(-1);
      }
      tmpFacetList.push_back(index);
    }
    polyFacets.push_back(tmpFacetList);
    count++;
  }

  //create polyhedron file in BYU format
  char polyhedron[10000];
  char tmp[100];

  sprintf(tmp, "1 %d %d 1 1 1\n", polyVertices.size(), polyFacets.size());
  strcpy(polyhedron,tmp);

  for(int i=0; i<polyVertices.size(); i++) {
    sprintf(tmp, "%f %f %f\n", polyVertices[i].getX(), polyVertices[i].getY(), polyVertices[i].getZ());
    strcat(polyhedron, tmp);
  }

  for(int i=0; i<polyFacets.size(); i++)
    for(int j=0; j<polyFacets[i].size(); j++) {
      if(j == polyFacets[i].size()-1) { //last one
        sprintf(tmp, "-%d\n", polyFacets[i][j]+1);
      } else {
        sprintf(tmp, "%d ", polyFacets[i][j]+1);
      }
      strcat(polyhedron, tmp);
    }
  
  //init polyhedron
  istrstream is(polyhedron);
  p->ReadBYU(is);
  //p->Write(cout);

  //release qhull data
  delete qhullData;
  qh NOerrexit = True;
  qh_freeqhull(!qh_ALL);
  qh_memfreeshort(&curlong, &totlong);

  return p;
}
