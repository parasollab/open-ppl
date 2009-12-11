/////////////////////////////////////////////////////////////////////
//
//   Haptic.cpp
//
//   General Description
//      This class reads in a 'haptic' path as input and calls the
//      appropriate pushing method located in Push.cpp. This class
//      is based on the HapticInput class.
//  Created
//      09/29/98  O.B. Bayazit (HRoadmap class)
/////////////////////////////////////////////////////////////////////


#include "Haptic.h"
#include "Environment.h"
#include "Push.h"
#include "util.h"
#include "LocalPlanners.h"

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
//     1 read_segment()
//     2 create_list(,,)
//
/////////////////////////////////////////////////////
void read_segment(char fileName[80])
{
  FILE *in;
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
      
        v.push_back(Cfg(Vector6D(head->position[0],head->position[1],head->position[2],
		      head->rotation[0],head->rotation[1],head->rotation[2])));
        head=head->next; 
     }
     l.push_back(v);
   } 
  return(l);
 
}


////////////////////////////////////////////////////////
//
// Haptic Class methods
//
///////////////////////////////////////////////////////

Haptic::Haptic() {
}
Haptic::~Haptic() {
}


void Haptic::init(Roadmap *rm, char * tmp[5], CollisionDetection *_cd, 
		  LocalPlanners *_lp, DistanceMetric *_dm, GNInfo gnInfo, CNInfo cnInfo) {

   std::string Callee(GetName());
   {std::string Method("-haptic::init");Callee=Callee+Method;}

   Push pushObject(rm, _cd, _lp, _dm, gnInfo, cnInfo);
   
   rdmp = rm;
   generationInfo = gnInfo;
   connectionInfo = cnInfo;
   env=rm->GetEnvironment();
   cd = _cd;
   lp = _lp;
   dm = _dm;
   
   int n=1,m=10, closest;
   int mask;
   read_segment(tmp[0]);
   sscanf(tmp[1],"%d",&n);
   sscanf(tmp[2],"%d",&m);
   sscanf(tmp[3],"%d",&closest);
   sscanf(tmp[4],"%d",&mask);
   cout << "n & m are " << n << " " << m << endl;
   vector <vector <Cfg> > list;
   list=create_list(path_head,path_num);

   bool checkConnection = true;
   for(int i=0; i<list.size(); i++) {
     vector <Cfg> cfgs = list[i];
     vector <Cfg> free;
     vector <Cfg> collided;
     int j;
     fprintf(stderr,"size=%d\n",cfgs.size());
     for(j=0;j<cfgs.size();j++)
     {
       if(cfgs[j].isCollision(env, cd, connectionInfo.cdsetid,connectionInfo.cdInfo,
                              true, &Callee ))
       {
         collided.push_back(cfgs[j]);
       }
       else free.push_back(cfgs[j]);
     }

     cout << " Total number of Cfgs in Haptic Path : " << cfgs.size() << endl;
     cout << " Number of Free  Cfgs in Haptic Path : " << free.size() << endl;
     cout << " Number of Colld Cfgs in Haptic Path : " << collided.size() << endl;
     cout << " mask  =  " << mask << endl;

     //rename these pushing methods ***
     if(mask&ADD_FREE) 
       AddFreeNodes(cfgs,checkConnection);

     if(mask&USE_AS_SEED)
       AddUsingSeed(collided,n);

     if(mask&USE_SURFACE)
       pushObject.SimplePush(cfgs,m);

     if(mask&CLOSESTWORKSPACEPOINTS)
       pushObject.WorkspaceAssistedPush(collided,closest);

     if(mask&USE_SURFACE2)
       pushObject.ShortestPush(cfgs);
   }
  #if INTERMEDIATE_FILES
    vector<Cfg> vertices;
    rm->m_pRoadmap->GetVerticesData(vertices);

    vector<Cfg> nodes;
    nodes.reserve(vertices.size());
    for(int k=0; k<vertices.size(); ++k)
	nodes.push_back(vertices[k]);
    WritePathConfigurations("hapticNodes.path", nodes, env);
  #endif

}


void Haptic::AddFreeNodes(vector <Cfg>cfgs,bool checkConnection)
{
  VID prev,current;
  prev=INVALID_VID;
  Cfg prevCfg;
  int i;
  std::string Callee(GetName());
  {std::string Method("-haptic::AddFreeNodes");Callee=Callee+Method;}

  LPInfo lpInfo;
  lpInfo.positionRes = rdmp->GetEnvironment()->GetPositionRes();
  lpInfo.checkCollision = true;
  lpInfo.savePath = false;
  lpInfo.cdsetid = connectionInfo.cdsetid;

  for(i=0;i<cfgs.size();i++)
  {
     cout << "Trying " << cfgs[i] << endl << flush;
     if(cfgs[i].isCollision(env, cd, connectionInfo.cdsetid,connectionInfo.cdInfo,
                            true, &Callee))
          {prev=INVALID_VID; prevCfg=cfgs[i]; continue;}
  
     current=rdmp->m_pRoadmap->AddVertex(cfgs[i]);
     cout << "VID : "<< current << " " <<cfgs[i] << " added in Free\n";
     if(prev==INVALID_VID) {prev=current; prevCfg=cfgs[i];continue; } 
    
     
     if(!checkConnection) {
        if(prev!=INVALID_VID) {
	   rdmp->m_pRoadmap->AddEdge(prev,current,lpInfo.edge);
	   cout << "Edge between (without check)" << prev << "-"<<current<<endl<<flush; 
	}
     } else { 
       if(prev!=INVALID_VID &&
	  lp->IsConnected(env,cd,dm,prevCfg,cfgs[i],connectionInfo.lpsetid,&lpInfo)) {
             rdmp->m_pRoadmap->AddEdge(prev,current,lpInfo.edge );
             //cout << "Edge between after check" << prev << "-"<<current<<endl<< flush;
       }
       else  cout << "Warning:  Could not connect the successive haptic nodes\n";
     }
     prevCfg=cfgs[i];
     prev=current;
  }
  


}


void Haptic::AddUsingSeed(vector <Cfg> seeds,int nodesPerSeed)
{
  vector <Cfg> nodes;
   std::string Callee(GetName());
   {std::string Method("-haptic::AddUsingSeed");Callee=Callee+Method;}

  
  cout << "Adding using as seed with " << nodesPerSeed << "\n";
  nodes=GenerateOBPRMNodes(env,cd, dm, seeds,nodesPerSeed,generationInfo);
  for(int i=0;i<nodes.size();i++)
  {
    if(nodes[i].isCollision(env, cd, connectionInfo.cdsetid,connectionInfo.cdInfo,
                            true, &Callee))
    {
      cout << "Error: The surface node is in collision\n";
    }
   cout << "VID : "<< rdmp->m_pRoadmap->AddVertex(nodes[i]) << " " <<nodes[i] << " added in seed\n";
  }
       cout << "Added " << nodes.size() << endl;
  if (nodes.size()) {  
       cout << "Added " << nodes.size() << endl;
  }

}
