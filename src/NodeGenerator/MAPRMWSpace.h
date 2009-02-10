#ifndef BasicMAPRM_h
#define BasicMAPRM_h

#include "NodeGeneratorMethod.h"
#include "MultiBody.h"

template <class CFG>
class BasicMAPRM : public NodeGenerationMethod<CFG> {
 public:
  //////////////////////
  // Constructors and Destructor
  BasicMAPRM();
  virtual ~BasicMAPRM();

  //////////////////////
  // Access
  virtual char* GetName();
  virtual void SetDefault();
  virtual int GetNextNodeIndex();
  virtual void SetNextNodeIndex(int);
  virtual void IncreaseNextNodeIndex(int);


  //////////////////////
  // I/O methods
  virtual void PrintOptions(ostream& out_os);
  virtual NodeGenerationMethod<CFG>* CreateCopy();

  //////////////////////
  // Core: Connection methods 
  virtual void GenerateNodes(Environment* _env, Stat_Class& Stats,
			     DistanceMetric *, vector<CFG>& nodes);

  void MoveOutObstacle(CFG & cfg, Environment *_env, Stat_Class& Stats,
		       CollisionDetection* cd, DistanceMetric* dm);
  void MoveOutObstacle(CFG & cfg, Vector3D & dir, Environment *_env, 
		       Stat_Class& Stats, CollisionDetection* cd);

  void MoveToMedialAxis(CFG &cfg, vector<CFG> *path, Environment *_env, 
			Stat_Class& Stats, CollisionDetection* cd, 
			DistanceMetric *dm, vector<CFG>& nodes, int l=0);

  static bool getCollisionInfo(CFG& cfg, Environment* _env, Stat_Class& Stats, CollisionDetection* cd, CDInfo& cdInfo);

#ifdef USE_VCLIP
  void BuildVCLIP(Environment* env);
#endif
	

  /////////////////////
  // Data

  int m_bApprox; //using approximation or exact computation
  int m_iRays; //number of shooting rays for approximation penetration.
  //Index for next node 
  //used in incremental map generation
  static int nextNodeIndex;

};


template <class CFG>
int BasicMAPRM<CFG>::nextNodeIndex = 0;


//---------------------------------------------------------------
// Ran3 -- "Global" UNIFORM Random number function
// This ought to be placed elsewhere, but at the moment
// it - along with Medial Axis is "in development"
// so it is stuck here - do NOT count on it remaining here !!!
// The code comes from Numerical Recipes in C, pages 274 - 285
//---------------------------------------------------------------
#define MBIG    1000000000
#define MSEED    161803398
#define MZ      0
#define FAC ((float)1.0 / MBIG)

float Ran3(long *idum);


template <class CFG>
BasicMAPRM<CFG>::
BasicMAPRM() : NodeGenerationMethod<CFG>() {
  SetDefault();
}


template <class CFG>
BasicMAPRM<CFG>::
~BasicMAPRM() {
}


template <class CFG>
char*
BasicMAPRM<CFG>::
GetName() {
  return "BasicMAPRM";
}


template <class CFG>
void
BasicMAPRM<CFG>::
SetDefault() {
  NodeGenerationMethod<CFG>::SetDefault();
  m_bApprox = 1;
  m_iRays = 10;
}


template <class CFG>
int
BasicMAPRM<CFG>::
GetNextNodeIndex() {
  return nextNodeIndex;
}

template <class CFG>
void
BasicMAPRM<CFG>::
SetNextNodeIndex(int index) {
  nextNodeIndex = index;
}

template <class CFG>
void
BasicMAPRM<CFG>::
IncreaseNextNodeIndex(int numIncrease) {
  nextNodeIndex += numIncrease;
}


template <class CFG>
void
BasicMAPRM<CFG>::
PrintOptions(ostream& out_os){
  out_os << "    " << GetName() << ":: ";
  out_os << "numNodes" << " " << this->numNodes << " ";
  out_os << "chunkSize" << " " << this->chunkSize << " ";
  out_os << "exactNodes" << " " << this->exactNodes << " ";
  out_os << "m_bApprox" << " " << m_bApprox << " ";
  out_os << "m_iRays" << " " << m_iRays << " ";
  out_os << endl;
}




template <class CFG>
NodeGenerationMethod<CFG>* BasicMAPRM<CFG>::CreateCopy() {
  NodeGenerationMethod<CFG> * _copy = new BasicMAPRM<CFG>(*this);
  return _copy;
}

template <class CFG>
void 
BasicMAPRM<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats, 
	      DistanceMetric* dm, vector<CFG>& nodes) {
  bool collided;
  CFG cfg;
  CollisionDetection* cd = this->GetMPProblem()->GetCollisionDetection();
  std::string Callee(GetName()), 
              Method("::GenerateNodes"),
	      CallCnt("1");
#ifndef QUIET
  cout << endl << "\t- Begin BasicMAPRM...\n";
  if( m_bApprox==0 ){
    cout<<"\t\t- Use exact penetration\n";
  }else{
    cout<<"\t\t- Use approximate penetration\n";
    cout<<"\t\t- " << m_iRays << " rays will be used to approximate penetration.\n";
  }

  cout << "(exactNodes=" << this->exactNodes << ") ";
#endif
  bool bExact = this->exactNodes == 1? true: false;

#if INTERMEDIATE_FILES
  vector<CFG> path; 
  path.reserve(this->numNodes);
#endif
  
#ifdef USE_VCLIP
  if( m_bApprox==0 ) //use exact penetration (VCLIP require). 
    BuildVCLIP(_env); //Make sure vclip is buid
#endif
  
#ifndef QUIET
  cout<<"- "<<flush;
#endif
  std::string tmpStr;
  for (int i=0; i < this->numNodes; i++){
    // Get a random configuration that STARTS in the bounding box of env
    cfg.GetRandomCfg(_env);  // should always be in bounding box
    
    //use approximate computation for moving out robot from obs
    if( m_bApprox ){
      this->cdInfo->ret_all_info = false;
      tmpStr = Callee+Method+CallCnt;
      collided = cfg.isCollision(_env, Stats, cd, *this->cdInfo, true, &tmpStr);
      if( collided ){
	MoveOutObstacle(cfg,_env, Stats, cd, dm);
	CallCnt="2";
	tmpStr = Callee+Method+CallCnt;
	collided = cfg.isCollision(_env, Stats, cd, *this->cdInfo, true, &tmpStr);
      }
      CallCnt = "3";
      tmpStr = Callee+Method+CallCnt;
      //Stats.IncCfgIsCall(&tmpStr);
      if(cd->isInsideObstacle(cfg,_env,*this->cdInfo)){
	this->cdInfo->ret_all_info = true;
	CallCnt="4";
	tmpStr = Callee+Method+CallCnt;
	cfg.isCollision(_env, Stats, cd, *this->cdInfo, true, &tmpStr);
	Vector3D trans_dir=(this->cdInfo->object_point-this->cdInfo->robot_point)*1.00001;
	this->cdInfo->ret_all_info = false;
	MoveOutObstacle(cfg,trans_dir,_env,Stats,cd);
	collided=!cfg.InBoundingBox(_env); //out of box
      }
    }
#ifdef USE_VCLIP
    else{ //use exact computation for penetration
      
      this->cdInfo->ret_all_info = true;	
      vector<CollisionDetectionMethod*> cd_selected;
      Vclip vclip;
      cd_selected.push_back(&vclip);
      CollisionDetection cd_vclip(cd_selected);
      CallCnt="vclip1";
      tmpStr = Callee+Method+CallCnt;
      cfg.isCollision(_env, Stats, &cd_vclip, *this->cdInfo, true, &tmpStr); //use vclip
      this->cdInfo->ret_all_info = false;
      if( collided ){
	Vector3D dir=(this->cdInfo->object_point-this->cdInfo->robot_point)*1.00001;
	cfg.SetSingleParam(0, cfg.GetData()[0]+dir[0]);
	cfg.SetSingleParam(1, cfg.GetData()[1]+dir[1]);
	cfg.SetSingleParam(2, cfg.GetData()[2]+dir[2]);
        CallCnt="vclip2";
	tmpStr = Callee+Method+CallCnt;
	collided = cfg.isCollision(_env, Stats, cd, *this->cdInfo,true, &tmpStr);
      }
    }
#endif
    
    // So we "should" be out of collision and INSIDE bounding box 
    // so we can move cfg toward Medial Axis
    if (!collided) {
      int nPrevNodes = nodes.size();
#if INTERMEDIATE_FILES
      MoveToMedialAxis(cfg, &path, _env, Stats, cd, dm, nodes);
      //nodes.push_back(cfg);
#else
      MoveToMedialAxis(cfg, NULL, _env, Stats, cd, dm, nodes);
      //nodes.push_back(cfg);
#endif
      
      if (bExact && nodes.size() ==  nPrevNodes) 
	i--; // To make sure the exact number of nodes
    } else {
      //cout << "BasicMAPRM unable to move random cfg out of collision." << endl;
      //cout<<"?"<<flush;
      i--;
      continue;
    }
    
#ifndef QUIET
    if( i%80==0 && i!=0 ) cout<<"("<<i<<"/"<<this->numNodes<<")"<<endl<<"- ";
    cout<<"#"<<flush;
#endif
    
  } // end for i = 1 to numNodes
  
#if INTERMEDIATE_FILES
  WritePathConfigurations("maprm.path", path, _env);
#endif
  
#ifndef QUIET
  cout << "(done)\n";
  cout << "... END BasicMAPRM\n";
#endif
}


#ifdef USE_VCLIP
template <class CFG>
void 
BasicMAPRM<CFG>::
BuildVCLIP(Environment* env) {
  int n=env->GetMultiBodyCount();
  for( int iM=0;iM<n;iM++ ){ //for each m-body
    MultiBody * mbody=env->GetMultiBody(iM);
    int nb=mbody->GetBodyCount();
    for( int iB=0;iB<nb;iB++ ){
      Body * body=mbody->GetBody(iB);
      PolyTree * tree=body->GetVclipBody();
      if( tree==NULL )
	body->buildCDstructure(VCLIP);
    }//end for each body
  }//end for each m-body
}
#endif


template <class CFG>
void 
BasicMAPRM<CFG>::
MoveOutObstacle(CFG& cfg, Environment* _env, Stat_Class& Stats,
		CollisionDetection* cd, DistanceMetric* dm) {

  std::string Callee(GetName()), 
              Method("::MoveOutObstacle"),
	      CallCnt("1");

  // Set _info so we do NOT get all info (for speed)
  this->cdInfo->ResetVars();
  
  /*
  // Generate Random direction
  CFG move_out_dir_cfg;
  move_out_dir_cfg.GetRandomRay( min(_env->GetPositionRes(),
				     _env->GetOrientationRes(), _env, dm);
  */
  long num_rays = m_iRays;  // how many rays do we try to get random cfg out of collision
  CFG* rays=new CFG[num_rays]; //testing directions
  CFG* pos=new CFG[num_rays];  //test position, will update each time by rays
  if( rays==NULL || pos==NULL ) return; //test if enough memory
  
  //init arrays
  for( int iR=0;iR<num_rays; iR++ ){
    rays[iR].GetRandomRay( min(_env->GetPositionRes(),
			       _env->GetOrientationRes()), _env, dm);	
    pos[iR]=cfg; //all start from given cfg
  }
  
  //start to escape though each shooting rays
  bool bCollide=true; //init condition is in collision, so set to true
  std::string tmpStr;
  while( bCollide ){ //loop until no in collision
    bool allOutBBX=true; //if all out of bbx, we don't want to go ahead.
    for( int iR=0;iR<num_rays;iR++ ){
      //pos[iR].add(pos[iR], rays[iR]);
      pos[iR].Increment(rays[iR]);
      if( !pos[iR].InBoundingBox(_env) ) continue; //out of bounding box
      allOutBBX=false;
      tmpStr = Callee+Method+CallCnt;
      if( !pos[iR].isCollision(_env, Stats, cd, *this->cdInfo, true, &tmpStr) ){
	bCollide=false; //not in collision any more
	cfg=pos[iR];
      }
    }//end for
    if( allOutBBX ) break; //failed
  }//end while( bCollide )
}


template <class CFG>
void 
BasicMAPRM<CFG>::
MoveOutObstacle(CFG& cfg, Vector3D& dir, Environment* _env, Stat_Class& Stats, 
		CollisionDetection* cd) {

  std::string Callee(GetName()), 
              Method("::MoveOutObstacle(,v)");

  // Store cfg in its "original" state
  cfg.SetSingleParam(0, cfg.GetData()[0]+dir[0]);
  cfg.SetSingleParam(1, cfg.GetData()[1]+dir[1]);
  cfg.SetSingleParam(2, cfg.GetData()[2]+dir[2]);
  
  //Make smaller increament
  CFG trans_cfg; //increament
  dir.normalize();
  trans_cfg.SetSingleParam(0, dir[0]);
  trans_cfg.SetSingleParam(1, dir[1]);
  trans_cfg.SetSingleParam(2, dir[2]);
  trans_cfg.multiply(trans_cfg, 0.1);
  
  //move in same dir until out of collision
  std::string tmpStr = Callee+Method;
  do{
    cfg.Increment(trans_cfg);
    //cfg.add(cfg, trans_cfg);
  }
  while(cfg.isCollision(_env, Stats, cd, *this->cdInfo, &tmpStr));
}


template <class CFG>
void 
BasicMAPRM<CFG>::
MoveToMedialAxis(CFG &cfg, vector<CFG>* path, Environment* _env, Stat_Class& Stats,
                 CollisionDetection* cd, DistanceMetric *dm,
		 vector<CFG>& nodes, int l) {
  //if(l>3){ nodes.push_back(cfg); return;}
  
  Vector3D trans_dir;
  CFG      trans_cfg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  CDInfo   oldInfo;
  std::string Callee(GetName()), 
              Method("::MoveToMedialAxis(,v)");
  std::string tmpStr = Callee+Method;
  
  // Set flag in _info.cdInfo to cause isCollision 
  // to return "all info" such as witness pairs and min dist 
  // and collision object index
  // NEW info currently always put in _info.cdInfo
  // note we do NOTHING with cd->stuff
  this->cdInfo->ResetVars();
  this->cdInfo->ret_all_info = true;
  
  // find closest obstacle -- and collided better come back false!
  getCollisionInfo(cfg,_env,Stats,cd,*this->cdInfo);
  trans_dir=this->cdInfo->robot_point-this->cdInfo->object_point;	
  trans_dir.normalize();
  
  // And then scale arbitrarily 'small' - prefer to have scale determined by the _env
  trans_dir = trans_dir*0.2;
  
  // Create a translation variable cfg
  // Make some BIG assumptions about parameter order - for cfg_free this should be right
  trans_cfg.SetSingleParam(0, trans_dir[0]);
  trans_cfg.SetSingleParam(1, trans_dir[1]);
  trans_cfg.SetSingleParam(2, trans_dir[2]);
  
  //Find the cfg that changes cloeset obs id
  //(This is not good (for non-convex), we should check closest pt.)
  CFG oldcfg=cfg;	
  CFG newcfg=cfg;
  Vector3D diff;
  
  do  //want to translate cfg in trans_dir until closest obstacle changes
    {
      oldcfg = newcfg;
      newcfg.add(oldcfg, trans_cfg);
      oldInfo = *this->cdInfo;   // oldInfo used in loop termination check
      this->cdInfo->ResetVars();
      this->cdInfo->ret_all_info = true;
      getCollisionInfo(newcfg,_env,Stats,cd,*this->cdInfo);
      diff=this->cdInfo->object_point-oldInfo.object_point;
    } while ( diff.normsqr()<1e-2 );
  
  tmpStr = Callee+Method;
  Stats.IncCfgIsColl(&tmpStr);  
  if( cd->isInsideObstacle(newcfg,_env,*this->cdInfo) ) return;
  //make sure newcfg is collision free
  this->cdInfo->ResetVars();
  tmpStr = Callee+Method;

  while( true ){
    if( newcfg.isCollision(_env, Stats, cd, *this->cdInfo, true, &tmpStr)==false ) 
      break;
    CFG tmp;
    tmp.subtract(oldcfg, newcfg);
    if( tmp.PositionMagnitude() < _env->GetPositionRes() ) return;
    newcfg.add(oldcfg,newcfg);
    newcfg.divide(newcfg,2);
  }
  
  cfg=newcfg;
  // while we should never move into a collision state, we check anyway	
#if INTERMEDIATE_FILES
  //MoveToMedialAxis(cfg, path, _env, cd, dm, nodes,++l);
  path->push_back(cfg);
#endif
  
  nodes.push_back(cfg);
  
} // end MoveToMedialAxis


template <class CFG>
bool 
BasicMAPRM<CFG>::
getCollisionInfo(CFG& cfg, Environment* _env, Stat_Class& Stats, 
		 CollisionDetection* cd, CDInfo& cdInfo){
  //cout<<cfg<<endl;
  //if( !cfg.InBoundingBox(_env) ) 
  //cout<<"Ho...Shit"<<endl;
  std::string Callee(cfg.GetName()), 
              Method("-BasicMAPRM::getCollisionInfo"); 

  std::string tmpStr = Callee+Method;
  if( cfg.isCollision(_env, Stats, cd, cdInfo, &tmpStr) ) return true;
  //  const static double * bbx = _env->GetBoundingBox();

  const BoundingBox * bbx = _env->GetBoundingBox();

  MultiBody *robot = _env->GetMultiBody(_env->GetRobotIndex());
  
  std::pair<double,double> bbx_range;
  //find cloest pt between robot and bbx
  for(int m=0; m<robot->GetFreeBodyCount(); m++) {
    GMSPolyhedron &poly = robot->GetFreeBody(m)->GetWorldPolyhedron();
    for(size_t j = 0 ; j < poly.vertexList.size() ; j++){
      //cout<<j<<" "<<poly.vertexList[j]<<endl;
      bool change=false;
      bbx_range = bbx->GetRange(0);

      if( (poly.vertexList[j][0]-bbx_range.first) < cdInfo.min_dist ){
	change = true;
	cdInfo.object_point[0] = bbx_range.first;
	cdInfo.object_point[1] = poly.vertexList[j][1];
	cdInfo.object_point[2] = poly.vertexList[j][2];
	cdInfo.min_dist = poly.vertexList[j][0]-bbx_range.first;
	cdInfo.nearest_obst_index = -1; //bbx
      }
      
      if( (bbx_range.second-poly.vertexList[j][0]) < cdInfo.min_dist ){
	change = true;
	cdInfo.object_point[0] = bbx_range.second;
	cdInfo.object_point[1] = poly.vertexList[j][1];
	cdInfo.object_point[2] = poly.vertexList[j][2];
	cdInfo.min_dist = bbx_range.second-poly.vertexList[j][0];
	cdInfo.nearest_obst_index = -2; //bbx
      }
      
      bbx_range = bbx->GetRange(1);

      if( (poly.vertexList[j][1]-bbx_range.first) < cdInfo.min_dist ){
	change = true;
	cdInfo.object_point[0] = poly.vertexList[j][0]; 
	cdInfo.object_point[1] = bbx_range.first;
	cdInfo.object_point[2] = poly.vertexList[j][2];
	cdInfo.min_dist = poly.vertexList[j][1]-bbx_range.first;
	cdInfo.nearest_obst_index = -3; //bbx
      }
      
      if( (bbx_range.second-poly.vertexList[j][1]) < cdInfo.min_dist ){
	change = true;
	cdInfo.object_point[0] = poly.vertexList[j][0];
	cdInfo.object_point[1] = bbx_range.second;
	cdInfo.object_point[2] = poly.vertexList[j][2];
	cdInfo.min_dist = bbx_range.second-poly.vertexList[j][1];
	cdInfo.nearest_obst_index = -4; //bbx
      }
      
      bbx_range = bbx->GetRange(2);
      if( (poly.vertexList[j][2]-bbx_range.first) < cdInfo.min_dist ){
	change = true;
	cdInfo.object_point[0] = poly.vertexList[j][0]; 
	cdInfo.object_point[1] = poly.vertexList[j][1]; 
	cdInfo.object_point[2] = bbx_range.first;
	cdInfo.min_dist = poly.vertexList[j][2]-bbx_range.first;
	cdInfo.nearest_obst_index = -5; //bbx
      }
      
      if( (bbx_range.second-poly.vertexList[j][2]) < cdInfo.min_dist ){
	change = true;
	cdInfo.object_point[0] = poly.vertexList[j][0];
	cdInfo.object_point[1] = poly.vertexList[j][1];
	cdInfo.object_point[2] = bbx_range.second;
	cdInfo.min_dist = bbx_range.second-poly.vertexList[j][2];
	cdInfo.nearest_obst_index = -6; //bbx
      }
      
      if(change==true){
	cdInfo.robot_point[0]=poly.vertexList[j][0]; 
	cdInfo.robot_point[1]=poly.vertexList[j][1];
	cdInfo.robot_point[2]=poly.vertexList[j][2];
      }
    }
  }
  
  return (cdInfo.min_dist>=0)?false:true;;
}

#endif


