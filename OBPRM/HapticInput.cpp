/////////////////////////////////////////////////////////////////////
//
//   HapticInput.h
//
//   General Description
//      Based on HRoadmap class written by O.B. Bayazit, this class
//      read in a 'haptic' path as input and process it and return 
//      free nodes in the roadmap.
//  Created
//      09/29/98  O.B. Bayazit (HRoadmap class)
/////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include "HapticInput.h"

#define EXPANSION_FACTOR 100
#define MAX_PATH_NUM 20
typedef struct _cfig {
  double position[3];
  double rotation[3];
  struct _cfig * next;
} cfig;

static cfig* critical_cfigs = NULL;   // list of critical configurations.
static cfig* path_head[MAX_PATH_NUM]; // the head of each prong of "road map".
static int path_num = 0;

/////////////////////////////////////////////////////
//
//  Local function:
//     1 read_prong()
//     2 create_list(,,)
//
/////////////////////////////////////////////////////
void read_prong(char fileName[80])
{
  int i;
  FILE *in;
  int timeStep;
  cfig *prev;
  char line[256];
  cout << "Reading from the approx. path file\n" << fileName << endl;
  if ( (in=fopen(fileName,"rt"))==NULL) { 
   fprintf(stderr,"Failed to open file %s\n",fileName);
   exit (5);
  }
  path_num=-0;
  while(fgets(line,255,in))
  {
   if( strstr(line,"segment # := "))
     {
       path_head[path_num]=new(cfig);
       prev=path_head[path_num];
       path_num++;
     }
   else
   {
    cfig *t=new(cfig);
    sscanf(line,"%lf%lf%lf%lf%lf%lf",
     &t->position[0],&t->position[1],&t->position[2],
     &t->rotation[0],&t->rotation[1],&t->rotation[2]);
    printf("Read : %f %f %f %f %f %f \n",
     t->position[0],t->position[1],t->position[2],
     t->rotation[0],t->rotation[1],t->rotation[2]);
    
     t->next=NULL;
     prev->next=t;
     prev=t;
   }
  }
}
vector <vector <Cfg> > create_list(cfig **c,int size)
{
  vector <vector <Cfg> > l;
  int i;
  cfig *head;
  for(i=0;i<size;i++) {
     vector <Cfg> v;
     head=c[i]->next;
     while(head!=NULL)
     {
      
        v.push_back(Cfg(Vector6<double>(head->position[0],head->position[1],head->position[2],
		      head->rotation[0],head->rotation[1],head->rotation[2])));
        head=head->next; 
     }
     l.push_back(v);
   } 
  return(l);
 
}


////////////////////////////////////////////////////////
//
// HapticInput Class methods
//
///////////////////////////////////////////////////////

HapticInput::HapticInput() {
}
HapticInput::~HapticInput() {
}


void HapticInput::init(Roadmap *rm, char * tmp[5], CollisionDetection *_cd, 
		  LocalPlanners *_lp, DistanceMetric *_dm, GNInfo gnInfo, CNInfo cnInfo) {

   rdmp = rm;
   generationInfo = gnInfo;
   connectionInfo = cnInfo;
   env=rm->GetEnvironment();
   cd = _cd;
   lp = _lp;
   dm = _dm;


   int n=1,m=10, closest;
   int mask;
   read_prong(tmp[0]);
   sscanf(tmp[1],"%d",&n);
   sscanf(tmp[2],"%d",&m);
   sscanf(tmp[3],"%d",&closest);
   sscanf(tmp[4],"%d",&mask);
   cout << "n & m are " << n << " " << m << endl;
   vector <vector <Cfg> > list;
   list=create_list(path_head,path_num);
   for(int i=0; i<list.size(); i++) {
	AddHapticPath(list[i], mask, n,m,closest,true);
   }
  #if INTERMEDIATE_FILES
    vector<Cfg> vertices = rm->roadmap.GetVerticesData();
    vector<Cfg> nodes;
    nodes.reserve(vertices.size());
    for(int k=0; k<vertices.size(); ++k)
	nodes.push_back(vertices[k]);
    WritePathConfigurations("hapticNodes.path", nodes, env);
  #endif

}

void HapticInput::AddHapticPath(vector <Cfg> cfgs, int mask,
                        int nodesPerSeed, int intermediate,
			int closest, bool checkConnection)
{
  vector <Cfg> free;
  vector <Cfg> collided;
  int i;
  fprintf(stderr,"size=%d\n",cfgs.size());
  for(i=0;i<cfgs.size();i++)
  {
   if(cfgs[i].isCollision(env, cd, connectionInfo.cdsetid,connectionInfo.cdInfo))
   {
     collided.push_back(cfgs[i]);
   }
   else free.push_back(cfgs[i]);

  }

  cout << " Total number of Cfgs in Haptic Path : " << cfgs.size() << endl;
  cout << " Number of Free  Cfgs in Haptic Path : " << free.size() << endl;
  cout << " Number of Colld Cfgs in Haptic Path : " << collided.size() << endl;
  cout << " mask  =  " << mask << endl;

  if(mask&ADD_FREE) AddFreeNodes(cfgs,checkConnection);
  if(mask&USE_AS_SEED)
    AddUsingSeed(collided,nodesPerSeed);
  if(mask&USE_SURFACE)
    AddUsingSurface(cfgs,intermediate);
  if(mask&CLOSESTWORKSPACEPOINTS)
    AddUsingClosestWorkspacePoints(collided,closest);

  if(mask&USE_SURFACE2)
    AddUsingSurface2(cfgs);
}


   
vector<Cfg> HapticInput::GenerateClosestOutsideNode(bool &directionKnown, Vector3D &direct,
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
	   
	  
    
void HapticInput::AddUsingSurface2(vector <Cfg> nodes)
{
  cout << " HapticInput::AddUsingSurface2 " << endl;
#if 1
	  vector<Cfg> mycfgs; 
	  int steps = 2;
	  for(int m=0; m<nodes.size()-1; m++) {
	      for(int n=0; n < steps; n++) {
		 mycfgs.push_back(Cfg::WeightedSum(nodes[m], nodes[m+1], 1./steps*n));
              }
	  }
	  nodes = mycfgs;
          WritePathConfigurations("mycfgs.path", nodes, env);
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
 WritePathConfigurations("surface2.path", surface, env);
 WritePathConfigurations("fixed.path", fixed, env);
 WritePathConfigurations("pushed.path", pushed, env);
 if(fixed.size()) {
    cout << "Add as fixed : " << fixed.size() << " nodes added\n";
    for(i=0;i<fixed.size();i++)
       cout << "VID : "<< rdmp->roadmap.AddVertex(fixed[i]) << " " <<fixed[i] << " added in fixed\n";
    //Connect_MapNodes();
    LPInfo lpInfo(rdmp, connectionInfo);
    for(i=1;i<fixed.size();i++) {
       if(lp->IsConnected(env,cd,dm,fixed[i-1],fixed[i],connectionInfo.lpsetid,&lpInfo)) {
          rdmp->roadmap.AddEdge(fixed[i-1],fixed[i], lpInfo.edge);
       }
       else  cout << "Warning:  Could not connect the successive haptic nodes\n";
    }

 } 

}



//----------------------------------------------------------------------------
void HapticInput::AddFreeNodes(vector <Cfg>cfgs,bool checkConnection)
{
  VID prev,current;
  prev=INVALID_VID;
  Cfg prevCfg;
  int i;

  LPInfo lpInfo;
  lpInfo.positionRes = rdmp->GetEnvironment()->GetPositionRes();
  lpInfo.checkCollision = true;
  lpInfo.savePath = false;
  lpInfo.cdsetid = connectionInfo.cdsetid;
  lpInfo.dmsetid = connectionInfo.dmsetid;
    


  for(i=0;i<cfgs.size();i++)
  {
     cout << "Trying " << cfgs[i] << endl << flush;
     if(cfgs[i].isCollision(env, cd, connectionInfo.cdsetid,connectionInfo.cdInfo))
          {prev=INVALID_VID; prevCfg=cfgs[i]; continue;}
  
     current=rdmp->roadmap.AddVertex(cfgs[i]);
     cout << "VID : "<< current << " " <<cfgs[i] << " added in Free\n";
     if(prev==INVALID_VID) {prev=current; prevCfg=cfgs[i];continue; } 
    
     
     if(!checkConnection) {
        if(prev!=INVALID_VID) {
	   rdmp->roadmap.AddEdge(prev,current,lpInfo.edge);
	   cout << "Edge between (without check)" << prev << "-"<<current<<endl<<flush; 
	}
     } else { 
       if(prev!=INVALID_VID &&
	  lp->IsConnected(env,cd,dm,prevCfg,cfgs[i],connectionInfo.lpsetid,&lpInfo)) {
             rdmp->roadmap.AddEdge(prev,current,lpInfo.edge );
       }
       else  cout << "Warning:  Could not connect the successive haptic nodes\n";
     }
     prevCfg=cfgs[i];
     prev=current;
  }
  


}
bool HapticInput::CheckConnection(Cfg c1,Cfg c2)
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
void HapticInput::AddUsingSeed(vector <Cfg> seeds,int nodesPerSeed)
{
  vector <Cfg> nodes;
  
  cout << "Adding using as seed with " << nodesPerSeed << "\n";
  nodes=GenerateOBPRMNodes(env,cd, dm, seeds,nodesPerSeed,generationInfo);
  for(int i=0;i<nodes.size();i++)
  {
    if(nodes[i].isCollision(env, cd, connectionInfo.cdsetid,connectionInfo.cdInfo))
    {
      cout << "Error: The surface node is in collision\n";
    }
   cout << "VID : "<< rdmp->roadmap.AddVertex(nodes[i]) << " " <<nodes[i] << " added in seed\n";
  }
       cout << "Added " << nodes.size() << endl;
  if (nodes.size()) {  
       cout << "Added " << nodes.size() << endl;
  }

}
//-----------------------------------------------------------------------------
//  closestKvertex: copy from util.c (Burchan)
//-----------------------------------------------------------------------------
//static int Compare3D(Vector3D *a,Vector3D *b)
static int Compare3D(const void *c,const void  *d)
{
   double aval,bval;
   Vector3D *a=(Vector3D *) c,*b=(Vector3D *) d;
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


void HapticInput::AddUsingClosestWorkspacePoints(vector <Cfg> seeds,int totalNodes)
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
 WritePathConfigurations("closestWorkspacePoints.path", surface, env);
}

void HapticInput::AddUsingSurface(vector <Cfg> nodes,int numIntermediate)
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
 WritePathConfigurations("intermediates.path", InterList, env);
 WritePathConfigurations("surface.path", surface, env);
  if(surface.size())
  {
    cout << "Add as surface : " << surface.size() << " nodes added\n";
    for(i=0;i<surface.size();i++)
           cout << "VID : "<< rdmp->roadmap.AddVertex(surface[i]) << " " <<surface[i] << " added in surface\n";
  } 

}


// the following three methods were in GenerateMapNodes Class, should go
// back there later.
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

