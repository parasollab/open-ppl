/////////////////////////////////////////////////////////////////////
//   Push.cpp
//
//   General Description
//      This class contains pushing methods based on the 
//      HapticInput class written by O.B. Bayazit and Guang Song.
//      This class takes a bad path and attempts to push it to the 
//      free space.
//  Created
//      09/29/98  O.B. Bayazit (HRoadmap class)
/////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include "Push.h"

// *** not sure if this is needed, or should go in Haptic.cpp
#define EXPANSION_FACTOR 100
#define MAX_PATH_NUM 20
typedef struct _cfig {
  double position[3];
  double rotation[3];
  struct _cfig * next;
} cfig;

static cfig* critical_cfigs = NULL;   // list of critical configurations.
static int path_num = 0;
// ***

////////////////////////////////////////////////////////
//
// Push Class methods
//
///////////////////////////////////////////////////////


//-----------------------------------------------------
// Constructor. This insures that everything gets
// initialized properly. It must be instanciated in
// order to call any pushing methods.
//-----------------------------------------------------
Push::Push(Roadmap *rm, CollisionDetection *_cd, 
		  LocalPlanners *_lp, DistanceMetric *_dm, GNInfo gnInfo, CNInfo cnInfo) {
   rdmp = rm;
   generationInfo = gnInfo;
   connectionInfo = cnInfo;
   env=rm->GetEnvironment();
   cd = _cd;
   lp = _lp;
   dm = _dm;
}


Push::~Push() {
}


//----------------------------------------------------
// Used only by ShortestPush().
//----------------------------------------------------
vector<Cfg> Push::GenerateClosestOutsideNode(bool &directionKnown, Vector3D &direct,
double &jumpSize, Cfg inside, double incrCoord) {
     static Vector3D savedDirection;
     if(!directionKnown)
	 savedDirection = direct;
     int testSize = directionKnown ? 10 : 100;
     double stepSize = env->GetPositionRes()/5.0;
     jumpSize *= 0.8;
     vector<Cfg> randomRay;
     vector<Cfg> startCfg;

     Vector3D Zaxis = direct;
     Zaxis.normalize(); // Zaxis is Zaxis
     Vector3D Xaxis = Vector3D(0.0, -Zaxis.getZ(), Zaxis.getY()); // this makes Xaxis*Zaxis = 0.
     Xaxis.normalize();
     Vector3D Yaxis = Zaxis.crossProduct(Xaxis);

     double angle_step = 2.*3.1415926/testSize;
     for(int i=0; i < testSize; i++) {
	    double ceta = angle_step*i;
	    Vector3D direction = Xaxis*cos(ceta) + Yaxis*sin(ceta);
	    double phi = 3.1415926*drand48();
	    if(directionKnown) phi *= 0.05;
            double tmpx=direction[0], tmpy=direction[1],tmpz= direction[2];
	    direction = Zaxis*cos(phi) + direction*sin(phi);
	    //Cfg ray(Vector6<double>(0,0,0,0,0,0));
//ray=Vector6<double>(direction[0], direction[1], direction[2], 0, 0, 0);
	    //Cfg ray(Vector6<double>(direction[0], direction[1], direction[2], 0, 0, 0));
	    Cfg ray(Vector6<double>(tmpx,tmpy,tmpz, 0, 0, 0));
	    randomRay.push_back(ray*stepSize);
	    startCfg.push_back(inside+ray*jumpSize);
     }
	 
     // Now, try each ray in parallel, find out which ray reach outside first, then return.
     int loopSize = 20;
     vector<Cfg> free;
     for(int l=0; l<loopSize; l++) {
        for(int i=0; i < testSize; i++) {
	   startCfg[i] = startCfg[i] + randomRay[i];
	   if(!startCfg[i].isCollision(env, cd, connectionInfo.cdsetid,connectionInfo.cdInfo)) {
                cout << " loop counter is " << l << "step Size is " << stepSize << endl;
		jumpSize += stepSize * (l+1);
		direct = randomRay[i].GetRobotCenterPosition();
		directionKnown = true;
	        free.push_back(startCfg[i]);	
		return free;
	   }
        }
     }
     directionKnown = false;
     jumpSize = 0.0;
     direct = savedDirection;
     return free;
}
	   
	  
//----------------------------------------------------
// This method finds the shortest distance between
// colliding configurations and the free space and
// pushes in that direction. This method requires that
// the first configuration in the vector nodes is  
// free. (It does not check this.)
//----------------------------------------------------
void Push::ShortestPush(vector <Cfg> nodes)
{
  cout << " Push::ShortestPush " << endl;
#if 1
	  vector<Cfg> mycfgs; 
	  int steps = 2;
	  for(int m=0; m<nodes.size()-1; m++) {
	      //Cfg &incr = nodes[m].FindIncrement(nodes[m+1], steps);
	      //Cfg tmp = nodes[m];
	      for(int n=0; n < steps; n++) {
		 mycfgs.push_back(Cfg::WeightedSum(nodes[m], nodes[m+1], 1./steps*n));
		 //tmp.Increment(incr);
              }
	  }
	  nodes = mycfgs;
          WritePathTranformationMatrices("mycfgs.path", nodes, env);
#endif

  vector <Cfg> fixed;
  vector <Cfg> pushed;
  vector <Cfg> surface;
  vector <Cfg> inside;
  Cfg prevFreeCfg=nodes[0]; 
  bool prevFound=true;
  double incrCoord = EXPANSION_FACTOR*env->GetPositionRes();;
  Cfg direction;
  int i,k;
  Cfg zero(Vector6<double>(0,0,0,0,0,0));
  for(i=0;i<nodes.size();i++) {
    cout << "At node " << nodes[i] << endl << flush;
   
    if(nodes[i].isCollision(env, cd, connectionInfo.cdsetid,connectionInfo.cdInfo) && i<(nodes.size()-1) ) {
       prevFound=false;
       inside.push_back(nodes[i]);
    } else {
       if(!prevFound) {
        
          cout << "Using between \n" << prevFreeCfg << endl << nodes[i] << flush;
          // Generate intermediate nodes 
          vector<Cfg> free; 
	  vector<Cfg> modifiedInside = inside;
#if 0
	  int steps = 3;
	  for(k=0; k<inside.size()-1; k++) {
	      Cfg &incr = inside[k].FindIncrement(inside[k+1], steps);
	      Cfg tmp = inside[k];
	      for(int n=0; n < steps; n++) {
		 modifiedInside.push_back(tmp);
		 tmp.Increment(incr);
              }
	  }
#endif
	  Vector3D direct = (nodes[i] - prevFreeCfg).GetRobotCenterPosition();
	  bool directionKnown = false;
	  double jump = 0.0;
          for(k=0;k<modifiedInside.size();k++) {
              cout << "GenerateClosestOutsideNode: k = " << k << ", jump = " << jump << endl;
	      free = GenerateClosestOutsideNode(directionKnown, direct, 
		     jump, modifiedInside[k], incrCoord); 
              cout << "inside[" << k << "] is " << modifiedInside[k] << endl;
	      for(int n=0; n<free.size(); n++) {
	         fixed.push_back(free[n]);
	         pushed.push_back(free[n]);
                 surface.push_back(free[n]);
 	      }
          }
          inside.erase(inside.begin(),inside.end());
	  modifiedInside.erase(modifiedInside.begin(), modifiedInside.end());
       }  // if
       prevFound=true;
       prevFreeCfg=nodes[i];
       // skip the last one, it may be in collision.
       if(i < nodes.size()-1 || !nodes[i].isCollision(env, cd, connectionInfo.cdsetid,connectionInfo.cdInfo))  
	  fixed.push_back(nodes[i]);
      
    } //else
 
 } // for i
 Vector3D com;
 WritePathTranformationMatrices("surface2.path", surface, env);
 WritePathTranformationMatrices("fixed.path", fixed, env);
 WritePathTranformationMatrices("pushed.path", pushed, env);
 if(fixed.size()) {
    cout << "Add as fixed : " << fixed.size() << " nodes added\n";
    for(i=0;i<fixed.size();i++)
       cout << "VID : "<< rdmp->roadmap.AddVertex(fixed[i]) << " " <<fixed[i] << " added in fixed\n";
    LPInfo lpInfo(rdmp, connectionInfo);
    for(i=1;i<fixed.size();i++) {
       if(lp->IsConnected(env,cd,dm,fixed[i-1],fixed[i],connectionInfo.lpsetid,&lpInfo)) {
          rdmp->roadmap.AddEdge(fixed[i-1],fixed[i], lpInfo.edge);
       }
       else  cout << "Warning:  Could not connect the successive haptic nodes\n";
    }

 } 

}


//----------------------------------------------------
// This method is used internally by AddFreeNodes in
// the Haptic class and should be moved there.
//----------------------------------------------------
bool Push::CheckConnection(Cfg c1,Cfg c2)
{
vector<LP> mylpset = lp->planners.GetLPSet(SL);
  LP bsl = mylpset[0];
  int i;
  // to create an edge
  WEIGHT bfedge = bsl.GetFEdgeMask();
  WEIGHT bbedge = bsl.GetBEdgeMask();

  pair<WEIGHT,WEIGHT> bedge(bfedge,bbedge);

  LPInfo lpInfo;
  lpInfo.positionRes = rdmp->GetEnvironment()->GetPositionRes();
  lpInfo.checkCollision = true;
  lpInfo.savePath = false;
  lpInfo.cdsetid = connectionInfo.cdsetid;
  lpInfo.dmsetid = connectionInfo.dmsetid;
  return lp->IsConnected(env,cd,dm, c1,c2,connectionInfo.lpsetid,&lpInfo);

}


//----------------------------------------------------
//  closestKvertex: copy from util.c (Burchan)
//----------------------------------------------------
//static int Compare3D(Vector3D *a,Vector3D *b)
static int Compare3D(const void *c,const void  *d)
{
   double aval,bval;
   Vector3D *a=(Vector3D *) c,*b=(Vector3D *) d;
   //aptal++;
   aval= a->magnitude();
   bval= b->magnitude();
   if (aval<bval) return -1;
   else if (aval>bval) return 1;
   else return 0;
}


///////////////////////////////////////////////////////////////////////
// Get the direction vectores for the "k" closest pairs of vertices
// between the robot and the obstacle in the environment belonging
// to the given roadmap.
//
// 
// We have assume that there are one robot and one obstacle in the
// environment.
//
// This method is used by WorkspaceAssistedPush().
////////////////////////////////////////////////////////////////////////
vector<Vector3D>  closestKvertex(Roadmap * _roadmap, int _k)
{
  int i;
  vector<Vector3D> returnVectorList;
  Vector3D *vectorList;

  int robot_index = 0;  // for now, the robot is the 1st body in the environment

  // Get the polyhedron for the robot (which is not world-transformed)
  GMSPolyhedron robotPoly = 
    _roadmap->GetEnvironment()->GetMultiBody(robot_index)->GetFirstBody()->GetPolyhedron();
  
  // Get the transformation matrix corresponding to the given robot's configuration
/*
  t44 robotT;
  double Q[6];
  Q[0] = _robotCfg.GetPositionVector().getX();
  Q[1] = _robotCfg.GetPositionVector().getY();
  Q[2] = _robotCfg.GetPositionVector().getZ();
  Q[3] = _robotCfg.GetOrientationVector().getX();
  Q[4] = _robotCfg.GetOrientationVector().getY();
  Q[5] = _robotCfg.GetOrientationVector().getZ();
  RPY_Config_To_Transform(Q, robotT);
*/

  // How to get the obstacle index???
  int obstacle_index = 1;  // for now, let's assume it as being "1", the 2nd body
  GMSPolyhedron obstaclePoly = _roadmap->GetEnvironment()->
	     GetMultiBody(obstacle_index)->GetFirstBody()->GetWorldPolyhedron();
#if 0
  if(  (vectorList=(Vector3D *) 
	malloc(sizeof(Vector3D)*robotPoly.numVertices*obstaclePoly.numVertices))==NULL) {cout << "Out of memory \n" << flush; exit(5); }

  for (i=0; i< robotPoly.numVertices; i++)
      for (int j=0; j< obstaclePoly.numVertices; j++){
	  vectorList[i*obstaclePoly.numVertices+j]=(obstaclePoly.vertexList[j] - robotPoly.vertexList[i]);
	}
    cout << "soring "  << robotPoly.numVertices*obstaclePoly.numVertices <<endl <<endl;
    qsort((void *)vectorList,robotPoly.numVertices*obstaclePoly.numVertices,sizeof(Vector3D),Compare3D); 
    cout << "sorted" << _k << endl <<flush;
#endif
  int pairs = robotPoly.numVertices*obstaclePoly.numVertices;
  int size = pairs > 10000 ? 10000 : pairs;
  vectorList = new Vector3D[size];
  for(i=0; i<size; i++) {
	vectorList[i] = obstaclePoly.vertexList[(int)(obstaclePoly.numVertices*drand48())] -
			robotPoly.vertexList[(int)(robotPoly.numVertices*drand48())];
  }
  cout << "sorting "  << endl;
  qsort((void *)vectorList, size, sizeof(Vector3D), Compare3D);
  cout << "sorted" << _k << endl <<flush;

    for(i=0;i<_k;i++)
    {
	returnVectorList.push_back(vectorList[i]);
     }
	free(vectorList);
    return returnVectorList;
}


//----------------------------------------------------
// This method uses the workspace to select a 
// direction to translate the robot. This method works 
// best when the colliding path is close to a free 
// path. This method will fail in cases where rotation
// is required to reach a free configuration.
//----------------------------------------------------
void Push::WorkspaceAssistedPush(vector <Cfg> seeds,int totalNodes)
{
  vector <Cfg> nodes;

  vector<Vector3D> closestNodes;
  vector<Cfg> surface;
 
  int i,j;
  Cfg inter;
  double Q[6];
  cout << "Adding using  closest workspace points with " << totalNodes << "\n" << flush;
  double incrCoord = EXPANSION_FACTOR*env->GetPositionRes();
  
  closestNodes=closestKvertex(rdmp,totalNodes);
  cout << "size " << closestNodes.size() << endl << flush;
  for(i=0;i<seeds.size();i++) {
    for(j=0;j<closestNodes.size();j++) {
            double tmpx=closestNodes[j].getX(), tmpy=closestNodes[j].getY(),tmpz= closestNodes[j].getZ();
        Cfg ccc(Vector6<double>(tmpx,tmpy,tmpz,0,0,0));

        inter=GenerateOutsideCfg(env,cd, seeds[i],
			ccc,
                          generationInfo);
        cout << "VID : "<< rdmp->roadmap.AddVertex(inter) << 
		" " <<inter << " added in closestWorkspacePoints\n";

        surface.push_back(inter);
    }
 }
 Vector3D com;
 WritePathTranformationMatrices("closestWorkspacePoints.path", surface, env);
}


//----------------------------------------------------
// This method uses connects a straight line between
// the first and last configurations of the vector. It
// then pushes the colliding configurations towards 
// the line and out into the free space. This method
// requires that the first and last configurations in
// the vector nodes are free. (It does not check this.)
//----------------------------------------------------
void Push::SimplePush(vector <Cfg> nodes,int numIntermediate)
{
  vector <Cfg> surface;
  vector <Cfg> intermediate;
  vector <Cfg> inside;
  vector <Cfg> InterList;
  Cfg inter;
  Cfg prevFreeCfg=nodes[0]; 
  bool prevFound=true;
  int num;
  double incrCoord = EXPANSION_FACTOR*env->GetPositionRes();;
  Cfg direction;
  int i,j,k;
  Cfg intermediate2;
  Cfg zero(Vector6<double>(0,0,0,0,0,0));
  cout << "incord= " <<incrCoord << endl;
  cout << "Adding using as surface: " << numIntermediate <<" intermediate nodes\n";
  for(i=0;i<nodes.size();i++)
  {
    cout << "At node " << nodes[i] << endl << flush;
   
    if(nodes[i].isCollision(env, cd, connectionInfo.cdsetid,connectionInfo.cdInfo)) {
       cout << "It is in  collision " << endl << flush;
       prevFound=false;
       inside.push_back(nodes[i]);
    } else {
       if(!prevFound) {
        
          cout << "Using between \n" << prevFreeCfg << endl << nodes[i] << flush;
          intermediate.erase(intermediate.begin(),intermediate.end());
          num=inside.size()*numIntermediate+1;
          // Generate intermediate nodes 
	  Cfg incr = prevFreeCfg.FindIncrement(nodes[i], num);
	  inter = prevFreeCfg;
          for(j=1;j<num;j++)
          {
	     inter.Increment(incr);
             intermediate.push_back(inter); 
             InterList.push_back(inter);
             if(!inter.isCollision(env, cd, connectionInfo.cdsetid,connectionInfo.cdInfo)) {
                  surface.push_back(inter);
                  cout << "Intermediate " << inter << " is added to surface list\n" << flush;
             }
                
          }
          for(j=0;j< inside.size(); j++) {
            Cfg ccc;
            for(k=0;k<numIntermediate;k++) {
              if(intermediate[k].isCollision(env, cd, connectionInfo.cdsetid,connectionInfo.cdInfo)) {
                 cout << "GenerateSurfaceCfg\n";
                 inter=GenerateOutsideCfg(env,cd, inside[j], 
			(intermediate[k]-inside[j]),
			  generationInfo);
              } else {
                 cout << "GenerateOutsideCfg\n";

                 inter=GenerateSurfaceCfg(env,cd, dm, inside[j],
                    intermediate[k], generationInfo);

              }
              cout << "\nBetween " << inside[j]  << endl <<intermediate[k] << flush;
              cout << "\n inter " << inter << endl << flush;
              surface.push_back(inter);
 	      Vector3D tmp = (intermediate[k]-inside[j]).GetRobotCenterPosition();
	      ccc = Cfg(Vector6<double>(tmp[0], tmp[1], tmp[2], 0.0, 0.0, 0.0));
              if(!ccc.AlmostEqual(zero)) {
                  cout << "\n Calling Generate outside for " << ccc << endl << flush;
                  inter=GenerateOutsideCfg(env,cd,inside[j],
			ccc,
                          generationInfo);
                  cout << "\n Positional inter " << inter << endl << flush;
                  surface.push_back(inter);
	      }
            } 


          }
        inside.erase(inside.begin(),inside.end());

       }  // if
       prevFound=true;
       prevFreeCfg=nodes[i];
      
    } //else
 
  } // for i
 Vector3D com;
 WritePathTranformationMatrices("intermediates.path", InterList, env);
 WritePathTranformationMatrices("surface.path", surface, env);
  if(surface.size())
  {
    cout << "Add as surface : " << surface.size() << " nodes added\n";
    for(i=0;i<surface.size();i++)
           cout << "VID : "<< rdmp->roadmap.AddVertex(surface[i]) << " " <<surface[i] << " added in surface\n";
  } 

}


//written by Sujay and Shawna
//----------------------------------------------------
// This method generates intermediate configurations
// between cfg_start and cfg_end. The pushing methods
// require that there be a high frequency of 
// configurations. It is useful when generating 
// intermediate configurations along an edge.
//----------------------------------------------------
vector <Cfg> Push::GenerateIntermediateCfgs(Cfg cfg_start, Cfg cfg_end, 
             double stepSize) {
  vector <Cfg> intermediateNodes;
  Cfg temp = cfg_start;
  Cfg original_dir = cfg_end - temp;

  //original_dir = original_dir / original_dir.PositionMagnitude();
  original_dir.SetSingleParam(0,
	    original_dir.GetSingleParam(0) / original_dir.PositionMagnitude());
  original_dir.SetSingleParam(1,
            original_dir.GetSingleParam(1) / original_dir.PositionMagnitude());
  original_dir.SetSingleParam(2,
            original_dir.GetSingleParam(2) / original_dir.PositionMagnitude());

  Cfg dir = original_dir;
  
  while ((original_dir.GetSingleParam(0) == dir.GetSingleParam(0)) &&
         (original_dir.GetSingleParam(1) == dir.GetSingleParam(1)) &&
         (original_dir.GetSingleParam(2) == dir.GetSingleParam(2))) {
    intermediateNodes.push_back(temp);
    temp = Cfg:: c1_towards_c2(temp,cfg_end,stepSize);
    dir = cfg_end - temp;

    dir.SetSingleParam(0, dir.GetSingleParam(0) / dir.PositionMagnitude());
    dir.SetSingleParam(1, dir.GetSingleParam(1) / dir.PositionMagnitude());
    dir.SetSingleParam(2, dir.GetSingleParam(2) / dir.PositionMagnitude());    
  }

  return intermediateNodes;
}


// This method takes in a set of Cfgs and returns the configurations
// that are in collision.
vector <Cfg> Push::findCollidedCfgs(vector<Cfg> cfgs) {
  vector <Cfg> collidedCfgs;
  for (int i = 0; i < cfgs.size(); i++) {
    if (cfgs[i].isCollision(env, cd, connectionInfo.cdsetid,connectionInfo.cdInfo))
      collidedCfgs.push_back(cfgs[i]);
  }
  return collidedCfgs;  
}


// This method takes in a path (generated by one of pushing methods)
// and checks if the path is collison free.
bool Push::isPathGood(vector <Cfg> cfgs) {
  bool goodpath = true;
  for (int i = 0; i < cfgs.size(); i++) {
    if (cfgs[i].isCollision(env, cd, connectionInfo.cdsetid,connectionInfo.cdInfo)){
      goodpath = false;
      break;
    }
  }
  return goodpath;
}


//----------------------------------------------------
// the following three methods were in 
// GenerateMapNodes Class, should go back there later. 
// 07/23/99 (G)
//----------------------------------------------------
Cfg
GenerateSurfaceCfg(Environment *env, CollisionDetection *cd, DistanceMetric *dm,
		   Cfg insideCfg, Cfg outsideCfg, GNInfo& _gnInfo){

    const double PositionRes = env->GetPositionRes();
    Cfg surface=outsideCfg;
    surface;

    Cfg low, high, mid,prev;
    double delta;
    int cnt;

    low = insideCfg; high = outsideCfg;
    if(high.isCollision(env, cd, _gnInfo.cdsetid,_gnInfo.cdInfo))
       cout << "Ebesinin ki  sicmis ya " << "\n" <<flush;
if(high.isCollision(env, cd, _gnInfo.cdsetid,_gnInfo.cdInfo) ||
   !low.isCollision(env, cd, _gnInfo.cdsetid,_gnInfo.cdInfo) ) {
  cout << " **************************** " << endl;
}

    mid = Cfg::WeightedSum(low, high);  //(low + high) / 2.0;
    delta = dm->Distance(env, low, high, _gnInfo.dmsetid);
    cnt = 0;
    prev=high;
    // Do the Binary Search
    cout << "detal = " << delta << " positionres= " << PositionRes << "\n" <<flush;

    cout << "Ebesinin ki " << mid << "\n" <<flush;
    while((delta >= PositionRes) &&
                !high.isCollision(env, cd,_gnInfo.cdsetid,_gnInfo.cdInfo)) {
            prev=high;
         cout << "Ebesinin ki hi " << high << "\n" <<flush;
         cout << "Ebesinin ki mid" << mid << "\n" <<flush;
         cout << "Ebesinin ki low " << low << "\n" <<flush;
         cout << "delta =   " << delta << "\n" <<flush;
        if(mid.isCollision(env, cd, _gnInfo.cdsetid,_gnInfo.cdInfo)){
            low = mid;
        } else {
            high = mid;
            surface=high;
        }
        mid = Cfg::WeightedSum(low, high); // (low + high) / 2.0;
        delta = dm->Distance(env, low, high, _gnInfo.dmsetid);
        cnt++;
    }
    if(surface.isCollision(env, cd,_gnInfo.cdsetid,_gnInfo.cdInfo)) {
           if(high.isCollision(env, cd,_gnInfo.cdsetid,_gnInfo.cdInfo))
                cout <<" Anlamiyorum abi " << flush;
cout <<" returning high" << flush;
           return prev;
    }
     else return surface;
}


Cfg
GenerateOutsideCfg(Environment *env, CollisionDetection *cd, 
                   Cfg InsideNode, Cfg incrCfg, GNInfo &_gnInfo){

    Cfg OutsideNode;
    OutsideNode = InsideNode + incrCfg;
    while(OutsideNode.isCollision(env, cd, _gnInfo.cdsetid,_gnInfo.cdInfo)){
        OutsideNode = OutsideNode + incrCfg;
    }
    return OutsideNode;
}


vector <Cfg>
GenerateOBPRMNodes(Environment *env, CollisionDetection *cd, DistanceMetric *dm,
		   vector <Cfg> seeds,int nodePerSeed, GNInfo &_gnInfo)
{
    vector <Cfg> SurfaceNodes;
    int i,j;
    Cfg incrCfg,OutsideNode;
    const double PositionRes = env->GetPositionRes();
    double incrCoord = EXPANSION_FACTOR*PositionRes;

    for(i=0;i<seeds.size();i++) {
       fprintf(stderr,"%d \n",i);
       for(j=0;j<nodePerSeed;j++) {
       fprintf(stderr,"%d \n",j);
           // Generate Random direction
           incrCfg=Cfg::GetRandomRay(incrCoord);

           cout << "Using "<< incrCfg << endl;
           // Generate outside cfg
           OutsideNode = GenerateOutsideCfg(env, cd, seeds[i], incrCfg, _gnInfo);
           cout << "Outsid "<< OutsideNode << endl;
           SurfaceNodes.push_back(GenerateSurfaceCfg(env,cd,dm,seeds[i],OutsideNode, _gnInfo));
       }
    }
  return SurfaceNodes;
}
