/////////////////////////////////////////////////////////////////////
//
//  CollisionDetection.c
//
//  General Description
//
//  Created
//      8/11/98  Daniel Vallejo
//
/////////////////////////////////////////////////////////////////////

#include "CollisionDetection.h"

#include "Cfg.h"
#include "Environment.h"
#include "MultiBody.h"
//#include "GenerateMapNodes.h"
//#include "ConnectMap.h"
#include <string.h>
#include "util.h"
#include "Transformation.h"
#include "DistanceMetrics.h"
#include "Stat_Class.h"


/////////////////////////////////////////////////////////////////////////
// BEGIN CLASS CDInfo
/////////////////////////////////////////////////////////////////////////
// Constructor
/////////////////////////////////////////////////////////////////////////
CDInfo::
CDInfo() {
  ResetVars();
} // end constructor




/////////////////////////////////////////////////////////////////////////
// Destructor
/////////////////////////////////////////////////////////////////////////
CDInfo::
~CDInfo() {
  // do nothing
}

/////////////////////////////////////////////////////////////////////////
// ResetVars
// 
// Re-Init vars as done by constructor
/////////////////////////////////////////////////////////////////////////
void CDInfo::ResetVars() {
  colliding_obst_index = -1;
  
  ret_all_info = false;
  nearest_obst_index = -1;
  min_dist = MaxDist;      // =  1e10 by CollisionDetection.h
  robot_point = 0;         // hope Vector3D class defined well
  object_point = 0;
} // end ResetVars



/////////////////////////////////////////////////////////////////////
//
//  METHODS for class CollisionDetection
//
/////////////////////////////////////////////////////////////////////

CollisionDetection::
CollisionDetection() { 
  penetration = -1;

#ifdef USE_CSTK
  Cstk* cstk = new Cstk();
  all.push_back(cstk);
#endif

#ifdef USE_VCLIP
  Vclip* vclip = new Vclip();
  all.push_back(vclip);
#endif

#ifdef USE_RAPID
  Rapid* rapid = new Rapid();
  all.push_back(rapid);
#endif

#ifdef USE_PQP
  Pqp* pqp = new Pqp();
  all.push_back(pqp);
#endif

  BoundingSpheres* boundingSpheres = new BoundingSpheres();
  all.push_back(boundingSpheres);

  InsideSpheres* insideSpheres = new InsideSpheres();
  all.push_back(insideSpheres);

  Naive* naive = new Naive();
  all.push_back(naive);

  Quinlan* quinlan = new Quinlan();
  all.push_back(quinlan);
}



CollisionDetection::
CollisionDetection(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
    MPBaseObject(in_Node, in_pProblem) { 
  LOG_DEBUG_MSG("CollisionDetection::CollisionDetection()");
  penetration = -1;

#ifdef USE_CSTK
  Cstk* cstk = new Cstk();
  all.push_back(cstk);
#endif

#ifdef USE_VCLIP
  Vclip* vclip = new Vclip();
  all.push_back(vclip);
#endif

#ifdef USE_RAPID
  Rapid* rapid = new Rapid();
  all.push_back(rapid);
#endif

#ifdef USE_PQP
  Pqp* pqp = new Pqp();
  all.push_back(pqp);
#endif

  BoundingSpheres* boundingSpheres = new BoundingSpheres();
  all.push_back(boundingSpheres);

  InsideSpheres* insideSpheres = new InsideSpheres();
  all.push_back(insideSpheres);

  Naive* naive = new Naive();
  all.push_back(naive);

  Quinlan* quinlan = new Quinlan();
  all.push_back(quinlan);
  
  vector<CollisionDetectionMethod*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    delete *I;
  selected.clear();

  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    for(int i=0; i<all.size(); ++i) {
      if(citr->getName() == all[i]->GetName()) {
        cout << "CollisionDetectionMethod selected = " << all[i]->GetName() << endl;
        selected.push_back(all[i]);
      }
    }
  }
  
  if(selected.size() < 1) {
    LOG_WARNING_MSG("No CollisionDetectionMethods selected!");
  }
  
  
  LOG_DEBUG_MSG("~CollisionDetection::CollisionDetection()");
}








CollisionDetection::
CollisionDetection(vector<CollisionDetectionMethod*>& _selected) {
  penetration = -1;

#ifdef USE_CSTK
  Cstk* cstk = new Cstk();
  all.push_back(cstk);
#endif

#ifdef USE_VCLIP
  Vclip* vclip = new Vclip();
  all.push_back(vclip);
#endif

#ifdef USE_RAPID
  Rapid* rapid = new Rapid();
  all.push_back(rapid);
#endif

#ifdef USE_PQP
  Pqp* pqp = new Pqp();
  all.push_back(pqp);
#endif

  BoundingSpheres* boundingSpheres = new BoundingSpheres();
  all.push_back(boundingSpheres);

  InsideSpheres* insideSpheres = new InsideSpheres();
  all.push_back(insideSpheres);

  Naive* naive = new Naive();
  all.push_back(naive);

  Quinlan* quinlan = new Quinlan();
  all.push_back(quinlan);

  for(int i=0; i<_selected.size(); i++) {
    CollisionDetectionMethod* method = _selected[i]->CreateCopy();
    selected.push_back(method);
  }
}


CollisionDetection::
~CollisionDetection() {
  vector<CollisionDetectionMethod*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    delete *I;

  for(I=all.begin(); I!=all.end(); I++)
    delete *I;

  for(int i=0; i<directions.size(); i++)
    delete directions[i];
}


void CollisionDetection::
PrintOptions(ostream& out_os) {
  out_os << "  CollisionDetection" << endl;
  vector<CollisionDetectionMethod*>::const_iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    (*I)->PrintOptions(out_os);
}


vector<CollisionDetectionMethod*>
CollisionDetection::
GetDefault() {
  vector<CollisionDetectionMethod*> Default;
#ifdef USE_RAPID
  Rapid* rapid = new Rapid();
  Default.push_back(rapid);
#else
#ifdef USE_VCLIP
  Vclip* vclip = new Vclip();
  Default.push_back(vclip);
#else
#ifdef USE_PQP
  Pqp* pqp = new Pqp();
  Default.push_back(pqp);
#else
  BoundingSpheres* boundingSpheres = new BoundingSpheres();
  Default.push_back(boundingSpheres);
#endif
#endif
#endif
  return Default;
}


vector<cd_predefined>
CollisionDetection::
GetAllCDTypes() const
{
  vector<cd_predefined> cdtypes;
  for(vector<CollisionDetectionMethod*>::const_iterator I = all.begin(); I != all.end(); ++I)
    cdtypes.push_back((*I)->GetCDType());
  return cdtypes;
}


vector<cd_predefined>
CollisionDetection::
GetSelectedCDTypes() const
{
  vector<cd_predefined> cdtypes;
  for(vector<CollisionDetectionMethod*>::const_iterator I = selected.begin(); I != selected.end(); ++I)
    cdtypes.push_back((*I)->GetCDType());
  return cdtypes;
}


void
CollisionDetectionMethod::
SetDefault() {
}


void
CollisionDetectionMethod::
ParseCommandLine(int argc, char** argv) {
  if(argc > 1) {
    cerr << "\nERROR ParseCommandLine: Don\'t understand \"";
    for(int i=0; i<argc; i++)
      cerr << argv[i] << " ";
    cerr << "\"\n\n";
    PrintUsage(cerr);
    cerr << endl;
    exit(-1);
  }
}


void
CollisionDetectionMethod::
PrintUsage(ostream& _os) const {
  _os.setf(ios::left,ios::adjustfield);

  _os << "\n" << GetName() << " ";

  _os.setf(ios::right,ios::adjustfield);
}


void
CollisionDetectionMethod::
PrintValues(ostream& _os) const {
  _os << "\n" << GetName() << " ";
  _os << endl;
}

void
CollisionDetectionMethod::
PrintOptions(ostream& _os) const {
  _os << "    " << GetName() << " ";
  _os << endl;
}



bool
CollisionDetection::
ParseCommandLine(int argc, char** argv) {
  bool found = FALSE;
  vector<CollisionDetectionMethod*>::iterator itr;

  int cmd_begin = 0;
  int cmd_argc = 0;
  char* cmd_argv[50];
  do {
    for(itr=all.begin(); itr!=all.end(); itr++) {
      if( !strcmp(argv[cmd_begin], (*itr)->GetName()) ) {
        cmd_argc = 0;
        bool is_method_name = false;
        do {
          cmd_argv[cmd_argc] = &(*(argv[cmd_begin+cmd_argc]));
          cmd_argc++;

          vector<CollisionDetectionMethod*>::iterator itr_names;
          is_method_name = false;
          for(itr_names=all.begin(); itr_names!=all.end() && cmd_begin+cmd_argc < argc; itr_names++)
            if( !strcmp(argv[cmd_begin+cmd_argc], (*itr_names)->GetName())) {
              is_method_name = true;
              break;
            }
        } while(!is_method_name && cmd_begin+cmd_argc < argc);

        (*itr)->ParseCommandLine(cmd_argc, cmd_argv);
        selected.push_back((*itr)->CreateCopy());
        (*itr)->SetDefault();
        found = TRUE;
        break;
      }
    }
    if(!found)
      break;
    cmd_begin = cmd_begin + cmd_argc;
  } while(cmd_begin < argc);

  return found;
}

void 
CollisionDetection::
PrintUsage(ostream& _os) const {
  vector<CollisionDetectionMethod*>::const_iterator I;
  for(I=all.begin(); I!=all.end(); I++)
    (*I)->PrintUsage(_os);
}


void 
CollisionDetection::
PrintValues(ostream& _os) const {
  vector<CollisionDetectionMethod*>::const_iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    (*I)->PrintValues(_os);
}


void 
CollisionDetection::
PrintDefaults(ostream& _os) const {
  vector<CollisionDetectionMethod*> Default;
  Default = GetDefault();
  vector<CollisionDetectionMethod*>::iterator I;
  for(I=Default.begin(); I!=Default.end(); I++)
    (*I)->PrintValues(_os);
  for(I=Default.begin(); I!=Default.end(); I++)
    delete (*I);
}

/*
void 
CollisionDetection::
WriteCDs(const char* _fname) const {
  ofstream  myofstream(_fname);
  if (!myofstream) {
    cout << endl << "In WriteCDS: can't open outfile: " << _fname ;
  }
  WriteCDs(myofstream);
  myofstream.close();
}

void
CollisionDetection::
WriteCDs(ostream& _myostream) const {  
  _myostream << endl << "#####CDSTART#####";
  _myostream << endl << selected.size();  // number of cds
  PrintValues(_myostream);
  _myostream << "#####CDSTOP#####";
}
*/  
/*
void 
CollisionDetection::
ReadCDs(const char* _fname) {
  ifstream  myifstream(_fname);
  if (!myifstream) {
    cout << endl << "In ReadCDs: can't open infile: " << _fname ;
    return;
  }
  ReadCDs(myifstream);
  myifstream.close();
};


void
CollisionDetection::
ReadCDs(istream& _myistream) {
  char tagstring[100];
  char cddesc[100];
  int  numCDs;
  
  _myistream >> tagstring;
  if ( !strstr(tagstring,"CDSTART") ) {
    cout << endl << "In ReadCDs: didn't read CDSTART tag right";
    return;
  }
  
  _myistream >> numCDs;
  _myistream.getline(cddesc,100,'\n');  // throw out rest of this line
  for (int i = 0; i < numCDs; i++) {
    _myistream.getline(cddesc,100,'\n');
    std::istringstream _cdstream(cddesc);
    int argc = 0;
    char* argv[50];
    char cmdFields[50][100];
    while(_cdstream >> cmdFields[argc]) {
      argv[argc] = (char*)(&cmdFields[argc]);
      argc++;
    }

    bool found = FALSE;
    try {
      found = ParseCommandLine(argc, argv);
      if(!found)
	throw BadUsage();
    } catch (BadUsage) {
      cerr << "Line error" << endl;
      exit(-1);
    }
  }
  
  _myistream >> tagstring;
  if ( !strstr(tagstring,"CDSTOP") ) {
    cout << endl << "In ReadCDs: didn't read CDSTOP tag right";
    return;
  }
}
*/  

bool
CollisionDetection::
IsInCollision(Environment* env, Stat_Class& Stats, CDInfo& _cdInfo, 
	      int robot, int obstacle, std::string *pCallName) {
  MultiBody *rob, *obst;
  rob = env->GetMultiBody(robot);
  obst = env->GetMultiBody(obstacle);
  
  return IsInCollision(env, Stats, _cdInfo, rob, obst, pCallName);
}


//////////////////////////////////////////////////////////////////////////
// IsInCollision
//
// modified by Brent, June 2000
// Added option to "get all info"
//
// Behavior is as follows:
// lineRobot parameter defaults to NULL if not sent
//
// if we DON'T want all info we 
//    will exit on the first collision detected
//    _cdInfo.colliding_obst_index is set
//    or loop through all objects and return no collision
//
// else 
//    we will ALWAYS go examine every obstacle
//    we still return if there is a collision
//    all of _cdInfo will be set to correspond to the closest obstacle
//    Notice if the robot self-collides, NOT all info is gotten
//
//////////////////////////////////////////////////////////////////////////
bool
CollisionDetection::
IsInCollision(Environment* env, Stat_Class& Stats, CDInfo& _cdInfo, 
	      MultiBody* lineRobot, bool enablePenetration, std::string *pCallName) {
  int nmulti, robot;
  bool ret_val, collision_found; // needed to go thru ALL obstacles to get ALL info
  CDInfo local_cd_info;
  nmulti = env->GetMultiBodyCount();
  robot = env->GetRobotIndex();
  
  MultiBody* rob = env->GetMultiBody(robot);
  
  // A line Segment generated on the fly, to check if 'seemingly connectable'.
  if (lineRobot) {
    rob = lineRobot;
  }
  
  ret_val = false;
  
  for (int i = 0; i < nmulti; i++) {
    if ( i != robot ) {
      // Note that the below call sets _cdInfo as needed
      collision_found = IsInCollision(env, Stats, _cdInfo, rob, env->GetMultiBody(i), pCallName);
      
      if ( (collision_found) && ( ! _cdInfo.ret_all_info) ) {
	_cdInfo.colliding_obst_index = i;
	
	return true;
      } else  if (_cdInfo.ret_all_info) {  // store more info
	if ((collision_found) && (!ret_val)) {
	  // colliding_obst_index is always the FIRST obstacle found in collision
	  // nearest_obst_index is 'nearest' obstacle (colliding or not)
	  local_cd_info.colliding_obst_index = i;
	  ret_val = true;
	}
	
	// Be certain that IsInCollision set _cdInfo.min_dist
	// Check new mins against old, reset *_points if needed
	// Store everything in local_cd_info, copy back to _cdInfo at end of function
	if (_cdInfo.min_dist < local_cd_info.min_dist) {
	  local_cd_info.nearest_obst_index = i;
	  local_cd_info.min_dist = _cdInfo.min_dist;
	  local_cd_info.robot_point = _cdInfo.robot_point;
	  local_cd_info.object_point = _cdInfo.object_point;
	} // end updating local_cd_info
      }
    } else {
      // robot self checking. Warning: rob and env->GetMultiBody(robot) may NOT be the same.
      if ( (rob->GetBodyCount() > 1) && 
	   (IsInCollision(env, Stats, _cdInfo, rob, rob, pCallName)) ) {
	if (_cdInfo.ret_all_info) {
	  // set stuff to indicate odd happenning
	  _cdInfo.colliding_obst_index = -1;
	  _cdInfo.min_dist = MaxDist;
	  _cdInfo.nearest_obst_index = -1;
	  _cdInfo.robot_point[0] = _cdInfo.robot_point[1] = _cdInfo.robot_point[2] = 0;
	  _cdInfo.object_point[0] = _cdInfo.object_point[1] = _cdInfo.object_point[2] = 0;
	}
	
	return true;
      }
    } // end  if-else i == robot
    
  } // end for i
  
  if (_cdInfo.ret_all_info) {
    // local_cd_info should contain "all the info" across all objects
    // _cdInfo only contains info for the last one processed above
    _cdInfo = local_cd_info;
  }
  
  return ret_val;
} // end IsInCollision ( 4 params, 4th defaults to NULL)


bool
CollisionDetection::
IsInCollision(Environment* env, Stat_Class& Stats, CDInfo& _cdInfo,
	      MultiBody* rob, MultiBody* obst, std::string *pCallName) {
  int nFreeRobot;
  nFreeRobot = rob->GetFreeBodyCount();

  vector<CollisionDetectionMethod*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++) {
    int tp = (*I)->GetType();
    
    // Type Out: no collision sure; collision unsure.
    if((tp == Out) && ((*I)->IsInCollision(rob, obst, Stats, _cdInfo, pCallName) == false)) {
      return false;
    }

    // Type In: no collision unsure; collision sure.
    if ((tp == In) && ((*I)->IsInCollision(rob, obst, Stats, _cdInfo, pCallName) == true)) {
      return true;
    }

    // Type Exact: no collision sure; collision sure.
    if(tp == Exact) {
      return (*I)->IsInCollision(rob, obst, Stats, _cdInfo, pCallName);
    }
  }
      
  return true;
}


double
CollisionDetection::
Clearance(Environment* env, Stat_Class& Stats) {
  return selected[0]->Clearance(env, Stats);
}


bool
CollisionDetection::
clearanceAvailable() {
  return selected[0]->clearanceAvailable();
}


bool
CollisionDetection::
isInsideObstacle(const Cfg& cfg, Environment* env, CDInfo& _cdInfo) {
  return selected[0]->isInsideObstacle(cfg, env, _cdInfo);
}


void 
CollisionDetection::
SetPenetration(double times) {
  penetration=times;
}


#ifdef DEBUG
vector<Cfg*> acceptable;
#endif 
//////////////////////////////////////////
// AcceptablePenetration
//
// If there is a collision, check whether it is in acceptable range
//
//
/////////////////////////////////////////////
bool 
CollisionDetection::
AcceptablePenetration(Cfg& c, Environment* env, Stat_Class& Stats,
		      CollisionDetection* cd, 
		      CDInfo& cdInfo) {
  int numOkCfgs=0;
  std::string Callee(c.GetName());
  {std::string Method("-collisiondetection::AcceptablePenetration"); Callee = Callee+Method; }
  
  for(int i=0;i<directions.size();i++) {
    //Cfg next=c+directions[i];
    Cfg* next = c.CreateNewCfg();
    next->add(c, *directions[i]);
    
    if (!next->isCollision(env, Stats, cd, cdInfo, false)) {
      numOkCfgs++;
      if ((numOkCfgs*1.0/directions.size()) > acceptableRatio) {
#ifdef DEBUG
	acceptable.push_back(&c);
#endif
	delete next;
	return true;
      }
    }
    delete next;
  }

  if((numOkCfgs*1.0/directions.size()) > acceptableRatio)
    return true;
  else
    return false;	
}


//////////


CollisionDetectionMethod::
CollisionDetectionMethod() {
  cdtype = CD_USER1;
}


CollisionDetectionMethod::
~CollisionDetectionMethod() {
}


bool
CollisionDetectionMethod::
operator==(const CollisionDetectionMethod& cd) const {
  return ( !(strcmp(GetName(), cd.GetName())) );
}


int
CollisionDetectionMethod::
GetType() {
  return type;
}


double
CollisionDetectionMethod::
Clearance(Environment* env, Stat_Class& Stats) {
  cout << "Clearance function is not supported by "
       << "current collision detection library." << endl
       << "Please recompile with a supporting library.\n";
  exit(5);
  return -1;
}


bool
CollisionDetectionMethod::
clearanceAvailable() {
  return false;
};


bool 
CollisionDetectionMethod::
isInsideObstacle(const Cfg& cfg, Environment* env, CDInfo& _cdInfo) {
  cerr<<"isInsideObstacle: Not implemeneted yet"<<endl;
  exit(1);
  return false;
}


//////////


#ifdef USE_CSTK
Cstk::
Cstk() : CollisionDetectionMethod() {
  type = Exact;
  cdtype = CSTK;
}


Cstk::
~Cstk() {
}


char*
Cstk::
GetName() const {
  return "cstk";
}


CollisionDetectionMethod*
Cstk::
CreateCopy() {
  CollisionDetectionMethod* _copy = new Cstk(*this);
  return _copy;
}


double
Cstk::
Clearance(Environment* env, Stat_Class& Stats) {
  int nmulti, robot;
  nmulti = env->GetMultiBodyCount();
  robot = env->GetRobotIndex();
  
  MultiBody *rob, *obst;
  rob = env->GetMultiBody(robot);
  
  double tmp, dist = MaxDist;
  for(int i = 0 ; i < nmulti ; i++) {
    if(i != robot) {
      obst = env->GetMultiBody(i);
      tmp = cstkDistance(Stats, rob, obst);
      if(tmp < dist) {
	dist = tmp;
      }
    }
  }
  return dist;
}


//----------------------------------------------------------------------
// SetLineTransformation
// set linTrans[12], which is used by cstk collision checker.
//----------------------------------------------------------------------
void 
Cstk::
SetLineTransformation(const Transformation& trans, double linTrans[12]) {
  Transformation tmp = trans;
  tmp.orientation.ConvertType(Orientation::Matrix);
  for(int n=0; n<3; n++) {
    for(int j=0; j<3; j++) {
      linTrans[4*n+j]=tmp.orientation.matrix[n][j];
    }
    linTrans[4*n+3]=tmp.position[n];
  }
}


bool
Cstk::
clearanceAvailable() {
  return true;
};


bool
Cstk::
IsInCollision(MultiBody* robot, MultiBody* obstacle, 
	      Stat_Class& Stats, CDInfo& _cdInfo, std::string *pCallName){
  Stats.IncNumCollDetCalls( "cstk", pCallName);
  
  // Identity in row-major order
  double linTrans[12] = {1,0,0,0, 0,1,0,0, 0,0,1,0};
  
  void *rob, *obst;
  cstkReal tolerance = 0.001;
  cstkReal dist;
  
  for(int i = 0 ; i < robot->GetFreeBodyCount(); i++){
    
    rob = robot->GetFreeBody(i)->GetCstkBody();
    SetLineTransformation(robot->GetFreeBody(i)->WorldTransformation(), linTrans);
    cstkUpdateLMovableBody(rob, linTrans);
    
    for(int k = 0 ; k < obstacle->GetBodyCount(); k++){
      
      // if robot check self collision, skip adjacent links.
      if(robot == obstacle &&
	 robot->GetFreeBody(i)->isAdjacent(obstacle->GetBody(k)) )
	continue;
      
      obst = obstacle->GetBody(k)->GetCstkBody();
      if(!obstacle->GetBody(k)->IsFixedBody()) { // FreeBody, cstkUpdate
	SetLineTransformation(obstacle->GetBody(k)->WorldTransformation(), linTrans);
	cstkUpdateLMovableBody(obst, linTrans);
      }
      
      dist = cstkBodyBodyDist(rob, obst, 500, 0, NULL, 0, NULL);
      if(dist < tolerance){
	return (true);
      }
    }
  }
  return false;
}


double
Cstk::
cstkDistance(Stat_Class& Stats, MultiBody* robot, MultiBody* obstacle, std::string *pCallName){
  Stats.IncNumCollDetCalls( "cstkDistance", pCallName );
  
  double linTrans[12] = {1,0,0,0, 0,1,0,0, 0,0,1,0};  // Identity in row-major order
  
  void *rob, *obst;
  cstkReal tmp, dist = MaxDist;
  
  for(int i = 0 ; i < robot->GetFreeBodyCount(); i++){
    rob = robot->GetFreeBody(i)->GetCstkBody();
    SetLineTransformation(robot->GetFreeBody(i)->WorldTransformation(), linTrans);
    cstkUpdateLMovableBody(rob, linTrans);
    for(int k = 0 ; k < obstacle->GetFixedBodyCount(); k++){
      obst = obstacle->GetFixedBody(k)->GetCstkBody();
      tmp = cstkBodyBodyDist(rob, obst, 500, 0, NULL, 0, NULL);
      if(tmp < dist){
	dist = tmp;
      }
    }
  }
  return dist;
}
#endif


//////////


#ifdef USE_VCLIP
Vclip::
Vclip() : CollisionDetectionMethod() {
  type = Exact;
  cdtype = VCLIP;
}


Vclip::
~Vclip() {
}


char*
Vclip::
GetName() const {
  return "vclip";
}


CollisionDetectionMethod*
Vclip::
CreateCopy() {
  CollisionDetectionMethod* _copy = new Vclip(*this);
  return _copy;
}


ClosestFeaturesHT closestFeaturesHT(3000);

bool
Vclip::
IsInCollision(MultiBody* robot, MultiBody* obstacle, 
	      Stat_Class& Stats, CDInfo& _cdInfo, std::string *pCallName) {
  Stats.IncNumCollDetCalls( "vclip", pCallName);
  
  Real dist;
  VclipPose X12;
  PolyTree *rob, *obst;
  Vect3 cp1, cp2;   // closest points between bodies, in local frame
  // we're throwing this info away for now
  
  if (_cdInfo.ret_all_info == true) {
    bool ret_val;
    ret_val = IsInColl_AllInfo_vclip(robot, obstacle, _cdInfo);
    return ret_val;
  }
  
  for(int i=0 ; i<robot->GetFreeBodyCount(); i++) {
    
    rob = robot->GetFreeBody(i)->GetVclipBody();
    
    for(int j=0; j<obstacle->GetBodyCount(); j++) {
      
      // if robot check self collision, skip adjacent links.
      if(robot == obstacle &&
	 robot->GetFreeBody(i)->isAdjacent(obstacle->GetBody(j)) ) {
	continue;
      }
      
      obst = obstacle->GetBody(j)->GetVclipBody();
      X12 = GetVclipPose(robot->GetFreeBody(i)->WorldTransformation(),
			 obstacle->GetBody(j)->WorldTransformation());
      dist = PolyTree::vclip(rob,obst,X12,closestFeaturesHT, cp1, cp2);
      
      if(dist < 0.0) { // once was < 0.001 ????
	return true;
      }
    } // end for j
  } // end for i
  
  return false;
} // end IsInCollision_vclip()


//////////////////////////////////////////////////////////////////////////
// IsInColl_AllInfo_vclip
// written by Brent, June 2000
//
// This function will fill in as much of _cdInfo as possible
// w.r.t. the robot and obstacle sent
// Notice each obstacle could change the results in _cdInfo
// Trace back to general IsInCollision call to see how it all
// gets updated correctly.
//////////////////////////////////////////////////////////////////////////
bool
Vclip::
IsInColl_AllInfo_vclip(MultiBody* robot, MultiBody* obstacle, 
		       CDInfo& _cdInfo) {
  Real dist, min_dist_so_far;
  VclipPose X12;
  PolyTree *rob, *obst;
  Vect3 cp1, cp2;   // closest points between bodies, in local frame
  Vector3D robot_pt, obs_pt;
  bool ret_val;
  
  ret_val = false;
  min_dist_so_far = MaxDist;  // =  1e10 by CollisionDetection.h
  
  for(int i=0; i<robot->GetFreeBodyCount(); i++) {
    rob = robot->GetFreeBody(i)->GetVclipBody();
    
    for(int j=0; j<obstacle->GetBodyCount(); j++) {
      
      // if robot check self collision, skip adjacent links.
      if(robot == obstacle &&
	 robot->GetFreeBody(i)->isAdjacent(obstacle->GetBody(j)) ) {
	continue;
      }
      
      obst = obstacle->GetBody(j)->GetVclipBody();
      X12 = GetVclipPose(robot->GetFreeBody(i)->WorldTransformation(),
			 obstacle->GetBody(j)->WorldTransformation());
      dist = PolyTree::vclip(rob,obst,X12,closestFeaturesHT, cp1, cp2);
      
      if ( dist < 0.0 ) {
	ret_val = true;
      }
      
      if (dist < min_dist_so_far) {
	min_dist_so_far = dist;
	// _cdInfo.nearest_obst_index =  is set by IsInCollision()
	// which called this function - look there for more info
	_cdInfo.min_dist = dist;
	
	// change a 3 elmt array to Vector3D class
	robot_pt[0] = cp1[0];
	robot_pt[1] = cp1[1];
	robot_pt[2] = cp1[2];
	
	obs_pt[0] = cp2[0];
	obs_pt[1] = cp2[1];
	obs_pt[2] = cp2[2];
	
	//cout << "CD method, robot pt = " << robot_pt << endl;
	//cout << "CD method, obs_pt = " << obs_pt << endl;
	
	// transform points to world coords
	// using *_pt vars in case overloaded * was not done well.
	_cdInfo.robot_point = robot->GetFreeBody(i)->WorldTransformation() * robot_pt;
	_cdInfo.object_point = obstacle->GetBody(j)->WorldTransformation() * obs_pt;
	
      }
    } // end for j
  } // end for i
  
  return ret_val;
} // end IsInColl_AllInfo_vclip()


VclipPose 
Vclip::
GetVclipPose(const Transformation &myT, const Transformation &obstT) {	
  Transformation diff = Transformation(obstT).Inverse() * myT;
  
  diff.orientation.ConvertType(Orientation::EulerXYZ);
  
  //------------------------------------------------
  // here's where it really starts.
  //------------------------------------------------
  
  Vect3 XYZ(diff.position.getX(),diff.position.getY(),diff.position.getZ());
  
  Quat RPY         (diff.orientation.alpha,Vect3::I);
  RPY.postmult(Quat(diff.orientation.beta ,Vect3::J));
  RPY.postmult(Quat(diff.orientation.gamma,Vect3::K));
  
  // the above is for EulerXYZ.
  // For EulerZYX, or FixedXYZ, we should have the following instead,
  // i.e. Rotation = Rz(alpha) * Ry(beta) * Rx(gamma)
  //Quat RPY         (diff.orientation.alpha,Vect3::K);
  //RPY.postmult(Quat(diff.orientation.beta ,Vect3::J));
  //RPY.postmult(Quat(diff.orientation.gamma,Vect3::I));
  
  return VclipPose(RPY,XYZ);
}
#endif


//////////


#ifdef USE_RAPID
Rapid::
Rapid() : CollisionDetectionMethod() {
  type = Exact;
  cdtype = RAPID;
}


Rapid::
~Rapid() {
}


char*
Rapid::
GetName() const {
  return "RAPID";
}


CollisionDetectionMethod*
Rapid::
CreateCopy() {
  CollisionDetectionMethod* _copy = new Rapid(*this);
  return _copy;
}


bool
Rapid::
IsInCollision(MultiBody* robot, MultiBody* obstacle, 
	      Stat_Class& Stats, CDInfo& _cdInfo, std::string *pCallName) {
    Stats.IncNumCollDetCalls( "RAPID", pCallName);
	
    RAPID_model *rob, *obst;
	
    if (_cdInfo.ret_all_info == true)
    {
		cout << endl;
		cout << "Currently unable to return ALL info using RAPID cd." << endl;
		cout << "Defaulting to minimal information." << endl;
	}
	
    for(int i=0 ; i<robot->GetFreeBodyCount(); i++){
		
      rob = robot->GetFreeBody(i)->GetRapidBody();
      
      for(int j=0; j<obstacle->GetBodyCount(); j++){
	
	// if robot check self collision, skip adjacent links.
	if(robot == obstacle &&
	   robot->GetFreeBody(i)->isAdjacent(obstacle->GetBody(j)) )
	  continue;
	
	obst = obstacle->GetBody(j)->GetRapidBody();
	Transformation &t1 = robot->GetFreeBody(i)->WorldTransformation();
	Transformation &t2 = obstacle->GetBody(j)->WorldTransformation();
	t1.orientation.ConvertType(Orientation::Matrix);
	t2.orientation.ConvertType(Orientation::Matrix);
	double p1[3], p2[3];
	for(int p=0; p<3; p++) {
	  p1[p] = t1.position[p];
	  p2[p] = t2.position[p];
	}
	if(RAPID_Collide(t1.orientation.matrix, p1, rob,
			 t2.orientation.matrix, p2, obst, RAPID_FIRST_CONTACT)) {
	  cout << "Error in CollisionDetection::RAPID_Collide, RAPID_ERR_COLLIDE_OUT_OF_MEMORY"
	       << RAPID_Collide(t1.orientation.matrix, p1, rob, t2.orientation.matrix, p2, obst, RAPID_FIRST_CONTACT) << endl;
	  exit(1);
	}
	if(RAPID_num_contacts) {
	  return true;
	}
	
      }
    }
    return false;
}
#endif


//////////


#ifdef USE_PQP
Pqp::
Pqp() : CollisionDetectionMethod() {
  type = Exact;
  cdtype = PQP;
}


Pqp::
~Pqp() {
}


char*
Pqp::
GetName() const {
  return "PQP";
}


CollisionDetectionMethod*
Pqp::
CreateCopy() {
  CollisionDetectionMethod* _copy = new Pqp(*this);
  return _copy;
}


bool 
Pqp::
isInsideObstacle(const Cfg& cfg, Environment* env, CDInfo& _cdInfo) {
  if( m_pRay==NULL ) 
    m_pRay = BuildPQPSegment(1e10,0,0);
  assert(m_pRay!=NULL);
  int nmulti = env->GetMultiBodyCount();
  int robot = env->GetRobotIndex();
  PQP_REAL t[3]={cfg.GetData()[0], cfg.GetData()[1], cfg.GetData()[2]};
  static PQP_REAL r[3][3]={{1,0,0}, {0,1,0}, {0,0,1}};
  
  for( int i=0; i < nmulti; i++ ) {//for all obstacles
    if( i==robot ) 
      continue;
    MultiBody* obstacle = env->GetMultiBody(i);
    for(int j=0; j<obstacle->GetBodyCount(); j++) {
      PQP_Model* obst = obstacle->GetBody(j)->GetPqpBody();
      GMSPolyhedron& poly=obstacle->GetBody(j)->GetPolyhedron();
      Transformation& t2 = obstacle->GetBody(j)->WorldTransformation();
      t2.orientation.ConvertType(Orientation::Matrix);
      double p2[3];
      for(int p=0; p<3; p++) 
	p2[p] = t2.position[p];
      
      PQP_CollideResult result;
      PQP_Collide(&result,r,t,m_pRay,t2.orientation.matrix,p2,obst);
      
      //anaylize result (check if there are adjacent triangle)
      vector<int> tri;
      //for each tri
      for( int iT=0; iT < result.NumPairs(); iT++ ) {
	bool add = true;
	int* tri1 = poly.polygonList[result.Id2(iT)].vertexList;
	//for each checked triangle
	for( int i=0; i < tri.size(); i++){
	  int* tri2=poly.polygonList[tri[i]].vertexList;
	  //check if they share same vertices
	  for(int itri1=0; itri1 < 3; itri1++) {
	    for(int itri2=0; itri2 < 3; itri2++) {
	      if( tri2[itri2]==tri1[itri1] ) {
		add = false;
		break;
	      }
	    }
	  }
	  if( add==false ) 
	    break;
	}
	//Ok no one shares vertex with you...
	if( add==true ) 
	  tri.push_back(result.Id2(iT));
      }
      int size=tri.size();
      if( (tri.size()%2)==1 ) 
	return true;
    }//end of each part of obs
  }//end of each obstacle
  
  return false;
}//end of function


PQP_Model* 
Pqp::
BuildPQPSegment(PQP_REAL dX, PQP_REAL dY, PQP_REAL dZ) const {
  //build a narrow triangle.
  PQP_Model* pRay = new PQP_Model();
  if( pRay==NULL ) 
    return NULL;
  
  if( dY==0 && dZ==0 && dX==0 ) 
    cout<<"! CollisionDetection::BuildPQPRay Warning : All are [0]"<<endl;
  
  static PQP_REAL tiny_v = ((double)1e-20)/LONG_MAX;
  static PQP_REAL pico_v = tiny_v/2;
  PQP_REAL p1[3] = { tiny_v, tiny_v, tiny_v };
  PQP_REAL p2[3] = { pico_v, pico_v, pico_v };
  PQP_REAL p3[3] = { dX, dY, dZ};
  
  pRay->BeginModel();
  pRay->AddTri(p1, p2, p3, 0);
  pRay->EndModel();
  
  return pRay;
}


bool
Pqp::
IsInCollision(MultiBody* robot, MultiBody* obstacle, 
	      Stat_Class& Stats, CDInfo& _cdInfo, std::string *pCallName) {
    Stats.IncNumCollDetCalls( "PQP", pCallName);
	
    PQP_Model *rob, *obst;
    PQP_CollideResult result;
	
    if (_cdInfo.ret_all_info == true)
    {
		/////////////////////////////////////////////////////////////////////////////
		//	11/2/01 modified by Jyh-Ming Lien : Start
		/////////////////////////////////////////////////////////////////////////////
		
		PQP_DistanceResult res;
		double min_dist_so_far = MaxDist;
		Vector3D robot_pt, obs_pt;
		bool ret_val=false;
		
		//for each part of robot
		for(int i=0 ; i<robot->GetFreeBodyCount(); i++){
			
			rob = robot->GetFreeBody(i)->GetPqpBody();
			Transformation &t1 = robot->GetFreeBody(i)->WorldTransformation();
			t1.orientation.ConvertType(Orientation::Matrix);
			double p1[3]; for(int ip1=0; ip1<3; ip1++) p1[ip1] = t1.position[ip1];
			
			//for each part of obstacle
			for(int j=0; j<obstacle->GetBodyCount(); j++){
				
				// if robot check self collision, skip adjacent links.
				if(robot == obstacle &&
					robot->GetFreeBody(i)->isAdjacent(obstacle->GetBody(j)) )
				{
					continue;
				}
				
				obst = obstacle->GetBody(j)->GetPqpBody();
				
				Transformation &t2 = obstacle->GetBody(j)->WorldTransformation();
				t2.orientation.ConvertType(Orientation::Matrix);
				double p2[3]; for(int ip2=0; ip2<3; ip2++) p2[ip2] = t2.position[ip2];
				
				if(PQP_Distance(&res,t1.orientation.matrix,p1,rob,
					t2.orientation.matrix,p2,obst,0.0,0.0))
				{
					cout << "Error in CollisionDetection::PQP_Collide, PQP_ERR_COLLIDE_OUT_OF_MEMORY"<<endl;
					exit(1);
				}
				
				if ( res.Distance() < 0.0 )  
				{
					ret_val = true;
				}
				
				if( res.Distance()<min_dist_so_far ){
					// _cdInfo.nearest_obst_index =  is set by IsInCollision()
					// which called this function - look there for more info
					min_dist_so_far=res.Distance();
					_cdInfo.min_dist = min_dist_so_far;
					
					// change a 3 elmt array to Vector3D class
					for( int k=0;k<3;k++ ){
						robot_pt[k] = res.P1()[k];
						obs_pt[k] = res.P2()[k];
					}
					
					// transform points to world coords
					// using *_pt vars in case overloaded * was not done well.
					_cdInfo.robot_point = robot->GetFreeBody(i)->WorldTransformation() * robot_pt;
					_cdInfo.object_point = obstacle->GetBody(j)->WorldTransformation() * obs_pt;
				}
			}//end of each part of obs
		}//end of each part of robot
		return ret_val;
		
		/////////////////////////////////////////////////////////////////////////////
		//	11/2/01 modified by Jyh-Ming Lien : End
		/////////////////////////////////////////////////////////////////////////////
	}
	
    for(int i=0 ; i<robot->GetFreeBodyCount(); i++){
		
		rob = robot->GetFreeBody(i)->GetPqpBody();
		
		for(int j=0; j<obstacle->GetBodyCount(); j++){
			
            // if robot check self collision, skip adjacent links.
            if(robot == obstacle &&
				robot->GetFreeBody(i)->isAdjacent(obstacle->GetBody(j)) )
				continue;
			
            obst = obstacle->GetBody(j)->GetPqpBody();
			Transformation &t1 = robot->GetFreeBody(i)->WorldTransformation();
			Transformation &t2 = obstacle->GetBody(j)->WorldTransformation();
			t1.orientation.ConvertType(Orientation::Matrix);
			t2.orientation.ConvertType(Orientation::Matrix);
			double p1[3], p2[3];
			for(int p=0; p<3; p++) {
				p1[p] = t1.position[p];
				p2[p] = t2.position[p];
			}
			
			if(PQP_Collide(&result, t1.orientation.matrix, p1, rob,
				t2.orientation.matrix, p2, obst, PQP_FIRST_CONTACT)) {
				cout << "Error in CollisionDetection::PQP_Collide, PQP_ERR_COLLIDE_OUT_OF_MEMORY"
					<< PQP_Collide(&result, t1.orientation.matrix, p1, rob, t2.orientation.matrix, p2, obst, PQP_FIRST_CONTACT) << endl;
				exit(1);
			}
			if(result.Colliding()) {
				return true;
            }
			
		}
    }
    return false;
}
#endif


//////////


BoundingSpheres::
BoundingSpheres() : CollisionDetectionMethod() {
  type = Out;
}


BoundingSpheres::
~BoundingSpheres() {
}


char*
BoundingSpheres::
GetName() const {
  return "boundingSpheres";
}


CollisionDetectionMethod*
BoundingSpheres::
CreateCopy() {
  CollisionDetectionMethod* _copy = new BoundingSpheres(*this);
  return _copy;
}


bool
BoundingSpheres::
IsInCollision(MultiBody* robot, MultiBody* obstacle, 
	      Stat_Class& Stats, CDInfo& _cdInfo, std::string *pCallName) {
  //cout << endl << "boundingSpheres Collision Check invocation" << flush;
  Stats.IncNumCollDetCalls( "boundingSpheres", pCallName );
  
  Vector3D robot_com = robot->GetCenterOfMass();
  Vector3D obst_com  = obstacle->GetCenterOfMass();
  
  if(robot->GetFreeBodyCount())
    robot_com = robot->GetFreeBody(0)->GetWorldTransformation() * robot_com;
  if(obstacle->GetFreeBodyCount())
    obst_com  = obstacle->GetFreeBody(0)->GetWorldTransformation() * obst_com;
  
  double robot_radius = robot->GetBoundingSphereRadius();
  double obst_radius  = obstacle->GetBoundingSphereRadius();
  
  double dist = sqrt(sqr(robot_com.getX() - obst_com.getX()) +
		     sqr(robot_com.getY() - obst_com.getY()) +
		     sqr(robot_com.getZ() - obst_com.getZ()));
  
  if (dist > robot_radius+obst_radius)
    return false;
  else
    return true;
}


//////////


InsideSpheres::
InsideSpheres() : CollisionDetectionMethod() {
  type = In;
}


InsideSpheres::
~InsideSpheres() {
}


char*
InsideSpheres::
GetName() const {
  return "insideSpheres";
}

CollisionDetectionMethod*
InsideSpheres::
CreateCopy() {
  CollisionDetectionMethod* _copy = new InsideSpheres(*this);
  return _copy;
}


bool
InsideSpheres::
IsInCollision(MultiBody* robot, MultiBody* obstacle, 
	      Stat_Class& Stats, CDInfo& _cdInfo, std::string *pCallName) {
  //cout << endl << "insideSpheres Collision Check invocation";
  Stats.IncNumCollDetCalls( "insideSpheres",pCallName );
  
  Vector3D robot_com = robot->GetCenterOfMass();
  Vector3D obst_com  = obstacle->GetCenterOfMass();

  if(robot->GetFreeBodyCount())
    robot_com = robot->GetFreeBody(0)->GetWorldTransformation() * robot_com;
  if(obstacle->GetFreeBodyCount())
    obst_com  = obstacle->GetFreeBody(0)->GetWorldTransformation() * obst_com;

  double robot_radius = robot->GetInsideSphereRadius();
  double obst_radius  = obstacle->GetInsideSphereRadius();

  double dist = sqrt(sqr(robot_com.getX() - obst_com.getX()) +
		     sqr(robot_com.getY() - obst_com.getY()) +
		     sqr(robot_com.getZ() - obst_com.getZ()));
  
  if (dist > robot_radius+obst_radius)
    return false;
  else
    return true;
}


//////////


Naive::
Naive() : CollisionDetectionMethod() {
  type = Exact;
}


Naive::
~Naive() {
}


char*
Naive::
GetName() const {
  return "naive";
}


CollisionDetectionMethod*
Naive::
CreateCopy() {
  CollisionDetectionMethod* _copy = new Naive(*this);
  return _copy;
}


bool
Naive::
IsInCollision(MultiBody* robot, MultiBody* obstacle, 
	      Stat_Class& Stats, CDInfo& _cdInfo, std::string *pCallName) {
  cout << endl << "naive Collision Check invocation";
  Stats.IncNumCollDetCalls( "naive", pCallName );
  return false;
}


//////////


Quinlan::
Quinlan() : CollisionDetectionMethod() {
  type = Exact;
}


Quinlan::
~Quinlan() {
}


char* 
Quinlan::
GetName() const {
  return "quinlan";
}


CollisionDetectionMethod*
Quinlan::
CreateCopy() {
  CollisionDetectionMethod* _copy = new Quinlan(*this);
  return _copy;
}


bool
Quinlan::
IsInCollision(MultiBody* robot, MultiBody* obstacle, 
	      Stat_Class& Stats, CDInfo& _cdInfo, std::string *pCallName) {
  cout << endl << "Quinlan Collision Check invocation";
  Stats.IncNumCollDetCalls( "quinlan", pCallName );
  return false;
}

//////////


