// $Id$
/////////////////////////////////////////////////////////////////////
//
//  LocalPlanners.c
//
//  General Description
//     This set of classes supports a "Local Planning Algobase".
//     This file contains the definitions of the prototypes
//     declared in "LocalPlanners.h".
//
//  Created
//      8/7/98  Nancy Amato
//  Last Modified By:
//      8/20/98  Daniel Vallejo
/////////////////////////////////////////////////////////////////////

#include "LocalPlanners.h"
#include "DistanceMetrics.h"
#include "util.h"
#include "Stat_Class.h"
#include "Roadmap.h"
#include "ConnectMapNodes.h"

extern Stat_Class Stats;

LPInfo::LPInfo(Roadmap *rm, const CNInfo& cnInfo) {
   checkCollision = true;
   savePath = false;
   saveFailedPath = false;
   positionRes = rm->GetEnvironment()->GetPositionRes(); 
   orientationRes = rm->GetEnvironment()->GetOrientationRes(); 
   cd_cntr = 0;
   cdsetid = cnInfo.cdsetid;
   dmsetid = cnInfo.dmsetid;
}

// change this in obprm too brc
int LocalPlanners::lineSegmentLength = 0;  // default value
bool LocalPlanners::usingClearance = 0;
#ifdef USE_CSTK
cd_predefined LocalPlanners::cdtype = CSTK;
#endif

#ifndef USE_CSTK
#ifdef USE_RAPID
cd_predefined LocalPlanners::cdtype = RAPID;
#endif
#endif

#ifndef USE_CSTK
#ifndef USE_RAPID
#cd_predefined LocalPlanners::cdtype = VCLIP
#endif
#endif



/////////////////////////////////////////////////////////////////////
//
//  METHODS for class LocalPlanners
//      
/////////////////////////////////////////////////////////////////////
  //==================================
  // LocalPlanners class Methods: Constructors and Destructor
  //==================================
  // LocalPlanners();
  // ~LocalPlanners();

LocalPlanners::
LocalPlanners() {
  DefaultInit();
};

LocalPlanners::
~LocalPlanners() {
};

  //==================================
  // LocalPlanners class Methods: Local Planner Functions
  //==================================

//-----------------------------------------------
// initialize default values for local planners
//  CAUTION:  DO NOT CHANGE ORDER OF SET DEFN's
//           w/o CHANGING ENUM ORDER in "OBPRM.h"
//-----------------------------------------------
void
LocalPlanners::
DefaultInit() {
                                        // enum SL
   planners.MakeLPSet("straightline");
                                        // enum R5
   planners.MakeLPSet("rotate_at_s");
                                        // enum SL_R5
   planners.MakeLPSet("straightline rotate_at_s");
                                        // enum AD69
   planners.MakeLPSet("a_star_distance 6 9");
                                        // enum SL_R5_AD69
   planners.MakeLPSet("straightline rotate_at_s a_star_distance 6 9");
}

//-----------------------------------------------
// initialize local planners with data from command line
//  CAUTION:  DO NOT CHANGE ORDER OF SET DEFN's
//           w/o CHANGING ENUM ORDER in "OBPRM.h"
//-----------------------------------------------
void
LocalPlanners::
UserInit(Input *input, ConnectMapNodes* cn) {
   if ( input->numLPs == 0 ) {           // use default LP sets
   } else {                             // make user-defined sets
     cn->cnInfo.lpsetid=LP_USER1;
     for (int i = 0; i < input->numLPs; i++) {
       planners.MakeLPSet(input->LPstrings[i]->GetValue());
     }
   }
   lineSegmentLength = input->lineSegment.GetValue();
   usingClearance = input->usingClearance.GetValue();
   cdtype = input->cdtype;
}

  // bool IsConnected(Roadmap *rm,Cfg _c1, Cfg _c2, SID _lpsetid, LPInfo *info);
  // bool IsConnectedFindAll(Roadmap *rm,Cfg _c1, Cfg _c2, SID _lpsetid, LPInfo *info);
  // bool IsConnected_straightline(Roadmap *rm,Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);
  // bool IsConnected_rotate_at_s(Roadmap *rm,Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);
  // bool IsConnected_astar(Roadmap *rm,Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);

//
// Find 1st LP in the set that can make the connection
//
bool
LocalPlanners::
IsConnected(Environment *_env,CollisionDetection *cd,DistanceMetric *dm,Cfg _c1, Cfg _c2, SID _lpsetid, LPInfo *info) {

  vector<LP> lpset = planners.GetLPSet(_lpsetid); 
  int  lpcnt, fedge, bedge;
  bool connected = false;

  lpcnt = fedge = bedge = 0;

  if ( _c1.isWithinResolution(_c2, info->positionRes,info->orientationRes) ) { // *replace* with dm check
    fedge = fedge | lpset[lpcnt].GetFEdgeMask();
    bedge = bedge | lpset[lpcnt].GetBEdgeMask();
  } else {
    while (!connected && lpcnt < lpset.size() ) {
       info->path.erase(info->path.begin(),info->path.end());
       //LPF lpfcn = lpset[lpcnt].GetPlanner();

       if ( IsConnected(lpset[lpcnt].GetName(), _env,cd,dm,_c1,_c2,lpset[lpcnt],info) == true ) {
          fedge = fedge | lpset[lpcnt].GetFEdgeMask();
          bedge = bedge | lpset[lpcnt].GetBEdgeMask();

          connected = true;
       } else {
          lpcnt++;
       }
    } 
  }
  if ( !connected && !info->saveFailedPath ) { 
       info->path.erase(info->path.begin(),info->path.end());
  }
  int n_ticks;
  n_ticks = info->nTicks;
  WEIGHT Fedge(fedge,n_ticks);
  WEIGHT Bedge(bedge,n_ticks);
  info->edge = pair<WEIGHT,WEIGHT>(Fedge,Bedge);
  return connected;
}


// another overloaded implementation. this one check if the edge is already in graph.
// so it will need a roadmap pointer instead of a pointer to env.
bool
LocalPlanners::
IsConnected(Roadmap *rm,CollisionDetection *cd,DistanceMetric *dm,Cfg _c1, Cfg _c2, SID _lpsetid,
LPInfo *info) {

  if( rm->roadmap.IsEdge(_c1, _c2) )  // check they are already connected.
      return true;
  Environment *_env = rm->GetEnvironment();

  vector<LP> lpset = planners.GetLPSet(_lpsetid);
  int  lpcnt, fedge, bedge;
  bool connected = false;

  lpcnt = fedge = bedge = 0;

  if ( _c1.isWithinResolution(_c2, info->positionRes, info->orientationRes) ) { // *replace* with dm check
    fedge = fedge | lpset[lpcnt].GetFEdgeMask();
    bedge = bedge | lpset[lpcnt].GetBEdgeMask();
    if(!_c1.AlmostEqual(_c2)) // avoid adding edge to itself.
	connected = true;
  } else {
    while (!connected && lpcnt < lpset.size() ) {
       info->path.erase(info->path.begin(),info->path.end());
       //LPF lpfcn = lpset[lpcnt].GetPlanner();

       if ( IsConnected(lpset[lpcnt].GetName(),_env,cd,dm,_c1,_c2,lpset[lpcnt],info) == true ) {
          fedge = fedge | lpset[lpcnt].GetFEdgeMask();
          bedge = bedge | lpset[lpcnt].GetBEdgeMask();

          connected = true;
       } else {
          lpcnt++;
       }
    }
  }
  if ( !connected && !info->saveFailedPath ) {
       info->path.erase(info->path.begin(),info->path.end());
  }
  int n_ticks;
  n_ticks = info->nTicks;
  WEIGHT Fedge(fedge,n_ticks);
  WEIGHT Bedge(bedge,n_ticks);
  info->edge = pair<WEIGHT,WEIGHT>(Fedge,Bedge);
  return connected;
}

//
// Find all LPs in the set that can make the connection
//
bool
LocalPlanners::
IsConnectedFindAll(Environment *_env,CollisionDetection *cd,DistanceMetric *dm,Cfg _c1, Cfg _c2, SID _lpsetid, LPInfo *info) {

  vector<LP> lpset = planners.GetLPSet(_lpsetid);
  int  lp, fedge, bedge;
  bool connected = false;
  lp = fedge = bedge = 0;

  if ( _c1.isWithinResolution(_c2, info->positionRes,info->orientationRes) ) { // *replace* with dm check
    fedge = fedge | lpset[lp].GetFEdgeMask();
    bedge = bedge | lpset[lp].GetBEdgeMask();
  } else {
    while (lp < lpset.size() ) {
       info->path.erase(info->path.begin(),info->path.end());
       //LPF lpfcn = lpset[lp].GetPlanner();
       if ( IsConnected(lpset[lp].GetName(),_env,cd,dm,_c1,_c2,lpset[lp],info) == true ) {
          fedge = fedge | lpset[lp].GetFEdgeMask();
          bedge = bedge | lpset[lp].GetBEdgeMask();
          connected = true;
       } else {
          lp++;
       }
    }
  }
  if ( !connected && !info->saveFailedPath ) { 
       info->path.erase(info->path.begin(),info->path.end());
  }
  int n_ticks;
  n_ticks = info->nTicks;
  WEIGHT Fedge(fedge,n_ticks);
  WEIGHT Bedge(bedge,n_ticks);
  info->edge = pair<WEIGHT,WEIGHT>(Fedge,Bedge);
  return connected;
}

// Generalized form of LP functions.
bool LocalPlanners::IsConnected(char *lpName, Environment *_env,CollisionDetection *cd,
     DistanceMetric*dm,Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info) {

  if (!strcmp(lpName,"straightline")) {
	return IsConnected_straightline(_env,cd,dm,_c1,_c2,_lp,info);
  } else if(!strcmp(lpName,"rotate_at_s")) {
	return IsConnected_rotate_at_s(_env,cd,dm,_c1,_c2,_lp,info);
  } else if(!strcmp(lpName,"a_star_clearance") || !strcmp(lpName,"a_star_distance")) { // A_star
	return IsConnected_astar(_env,cd,dm,_c1,_c2,_lp,info);
  } else {
	cout << "Error: in LocalPlanners::IsConnected(char *lpName, ...), invalid lp option!" << endl;
	exit(1);
  }


}

bool LocalPlanners::IsConnected_SLclearance(Environment *_env,CollisionDetection *cd,
     DistanceMetric*,Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info) {
    
     if(info->checkCollision) {
	double clr, halfDist, halfOriDist;
	double rmax = _env->GetMultiBody(_env->GetRobotIndex())->GetBoundingSphereRadius();
	bool sameOrientation = (_c1-_c2).OrientationMagnitude() <= info->orientationRes;
	//bool sameOrientation = Cfg::isSameOrientation(_c1, _c2);
        typedef pair<Cfg,Cfg> cfgPair;
        deque<cfgPair> pairQ;
    	pairQ.push_back(cfgPair(_c1,_c2));

	while(! pairQ.empty() ) {
	   cfgPair &tmp = pairQ.front();
           // brc removed &
	   //Cfg &mid = Cfg::WeightedSum(tmp.first, tmp.second, 0.5);
	   Cfg mid = Cfg::WeightedSum(tmp.first, tmp.second, 0.5);
	   info->cd_cntr ++;
	   if((clr = mid.Clearance(_env,cd)) <= 0.001) { // 0.001 tolerance.
	      return false;
	   } else {
	      if(!sameOrientation) {
		  clr -= rmax;
		  if(clr < 0) clr = 0.0;
	      }
	      // brc removed &
              //Cfg& diff = tmp.first - tmp.second;
	      Cfg diff = tmp.first - tmp.second;
              halfDist = diff.PositionMagnitude()/2;
	      if(clr < halfDist) { 
		 // if(Cfg::isWithinResolution(tmp.first, tmp.second)) 
		 halfOriDist = diff.OrientationMagnitude()/2;
		 if(info->positionRes < halfDist || info->orientationRes < halfOriDist ) {
                    // brc removed &
		    //Cfg& tmp1 = Cfg::WeightedSum(tmp.first, mid, 1.0-clr/halfDist);
		    //Cfg& tmp2 = Cfg::WeightedSum(mid, tmp.second, clr/halfDist);
		    Cfg tmp1 = Cfg::WeightedSum(tmp.first, mid, 1.0-clr/halfDist);
		    Cfg tmp2 = Cfg::WeightedSum(mid, tmp.second, clr/halfDist);
		    pairQ.push_back(cfgPair(tmp.first, tmp1));
		    pairQ.push_back(cfgPair(tmp2, tmp.second)); 
		 }
	      }
	   }
	   pairQ.pop_front();
	}
     }
     if(info->savePath || info->saveFailedPath){
	int steps;
        // brc removed &
        //Cfg& incr = _c1.FindIncrement(_c2,&steps,info->positionRes,info->orientationRes);
        Cfg incr = _c1.FindIncrement(_c2,&steps,info->positionRes,info->orientationRes);
	Cfg tick = _c1;
	for(int i=0; i<steps; i++) {
	   tick.Increment(incr);
	   info->path.push_back(tick);
	}
     }
     return true;
}

bool LocalPlanners::lineSegmentInCollision(Environment *_env,CollisionDetection *cd,
     DistanceMetric* dm, Cfg& _c1, Cfg&_c2, LP& _lp, LPInfo *info) {
    int steps = (_c1-_c2).PositionMagnitude()/info->positionRes;
    if( steps <= lineSegmentLength ) return false;

    //before really starting to connect the two cfgs, ie. _c1, _c2.
    //First make sure that there are 'seemingly connectable'.
    //i.e. their CMSs can see each other.
    //this method seems to only work for rigid-body robot.
    //moreover, it requires robot's CMS inside the robot.
   
    //if(Cfg::GetType() == RIGID_BODY) 
    Vector3D v1 = _c1.GetRobotCenterPosition();
    Vector3D v2 = _c2.GetRobotCenterPosition();
    //Vector3D dif = v1-v2;
    //if(dif.magnitude() > 0.0001) {
    Vector3D v3 = v1 + Vector3D(0.000001, 0, 0);
    //Vector3D v3 = v1.crossProduct(v2);
    //v3.normalize();
    //v3 = v3*0.000001; // really thinny triangle.
    Vector3D center = (v1+v2+v3)/3.0;
    
    char str[200];
    sprintf(str, "%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s", "1 3 1 3 \n 1 3\n", v1[0], "  ", v1[1], 
		 "  ", v1[2], "\n", v2[0], "  ", v2[1], "  ", v2[2], "\n", v3[0], "  ", v3[1], 
		 "  ", v3[2], "\n1 2 -3 ");
    istrstream istr(str);

    MultiBody * lineSegment = new MultiBody(_env);
    FreeBody fb(lineSegment);
    fb.ReadBYU(cdtype, istr);
    // brc added temp. matrix t;
    Transformation t=Transformation(Orientation(IdentityMatrix), center);
    fb.Configure(t);
    lineSegment->AddBody(&fb);

    info->cd_cntr ++;
    if( cd->IsInCollision(_env, info->cdsetid, lineSegment) )
        return true;
    return false;
}


bool 
LocalPlanners::
IsConnected_straightline(Environment *_env,CollisionDetection *cd,DistanceMetric* dm,Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info) {

    Stats.IncLPAttempts( "Straightline" );
    info->cd_cntr = 0; 

    if(lineSegmentLength && lineSegmentInCollision(_env, cd, dm, _c1, _c2, _lp, info)) {
        Stats.IncLPCollDetCalls( "Straightline", info->cd_cntr );
	return false;
    }

    bool connected = IsConnected_straightline_simple(_env, cd, dm, _c1, _c2, _lp, info);
    if(connected)
        Stats.IncLPConnections( "Straightline" );

    Stats.IncLPCollDetCalls( "Straightline", info->cd_cntr );
    return connected;
}


bool
LocalPlanners::
IsConnected_straightline_simple(Environment *_env,CollisionDetection *cd,DistanceMetric* dm,Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info) {

    if(usingClearance) 
	return IsConnected_SLclearance(_env, cd, dm, _c1, _c2, _lp, info);

    int n_ticks;
    Cfg tick=_c1;
    Cfg incr=_c1.FindIncrement(_c2,&n_ticks,info->positionRes,info->orientationRes);

    int nTicks;
    nTicks = 0;
    for(int i = 0; i < n_ticks ; i++){
        tick.Increment(incr);

        info->cd_cntr ++;
        if(info->checkCollision){
            if(tick.isCollision(_env,cd, info->cdsetid)){
		tick.Increment(-incr);
		info->savedEdge = pair<Cfg,Cfg>(_c1, tick);
		info->nTicks = nTicks;
                return false;
            }
        }
        if(info->savePath || info->saveFailedPath){
            info->path.push_back(tick);
        }
	nTicks++;
    }
    info->nTicks = nTicks;
    return true;

};


bool
LocalPlanners::
IsConnected_rotate_at_s(Environment *_env,CollisionDetection *cd,DistanceMetric *dm,Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info) {


    int nTicks;
    nTicks = 0;

    char RatS[20] = "Rotate_at_s";
    sprintf(RatS,"%s=%3.1f",RatS,_lp.GetS());
    Stats.IncLPAttempts( RatS );
    info->cd_cntr= 0;

    if(lineSegmentLength && lineSegmentInCollision(_env, cd, dm, _c1, _c2, _lp, info)) {
	Stats.IncLPCollDetCalls( RatS, info->cd_cntr );
        return false;
    }

    vector<Cfg> sequence = _c1.GetMovingSequenceNodes(_c2, _lp.GetS());
    bool connected = true;
    for(int i=0; i<sequence.size()-1; ++i) {
	bool flag;
	flag = IsConnected_straightline_simple(_env,cd,dm, sequence[i], sequence[i+1], _lp, info);
	nTicks += info->nTicks;
	if(!flag) {
	     connected = false;
	     break;
 	}
    }
    if(connected)
        Stats.IncLPConnections( RatS );

    Stats.IncLPCollDetCalls( RatS, info->cd_cntr );
    info->nTicks = nTicks;
    return connected;

};


bool
LocalPlanners::
IsConnected_astar(Environment *_env,CollisionDetection *cd,DistanceMetric *dm,Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info) {


    Stats.IncLPAttempts( "AStar" );
    info->cd_cntr = 0;

    Cfg p=_c1;
    Cfg incr ;
    Cfg diagonal;
    vector<Cfg> neighbors;
    int n_ticks;
    double maxClearance=-MAXFLOAT;
    double minDistance=MAXFLOAT;
    int retPosition,noNeighbors=3,i;
    int typeLP=0;
    double value;
    if( !strcmp(_lp.GetName(),"a_star_distance")) typeLP=1;

    incr=_c1.FindIncrement(_c2,&n_ticks,info->positionRes,info->orientationRes);

    bool connected = true;
    int noTries=0;
    int nTicks = 0;
    do {
        /* First check the diagonal to find out if it it available */
      diagonal=p;
      diagonal.IncrementTowardsGoal(_c2,incr);

      info->cd_cntr++;
      if(!diagonal.isCollision(_env,cd, info->cdsetid)){
          p=diagonal;
      } else {
          neighbors=p.FindNeighbors(_env, _c2,incr,cd,noNeighbors,info->cdsetid);
          if (neighbors.size()==0) { 
             connected = false;
	     info->savedEdge = pair<Cfg,Cfg>(_c1, p);
	     break;
          }
				    
          maxClearance=-MAXFLOAT;
          minDistance=MAXFLOAT;
          retPosition=0;
          for(i=0;i<neighbors.size();i++) {
            if(typeLP==0) {
		   value=neighbors[i].Clearance(_env,cd);
                   if (value>maxClearance) {
                       retPosition=i;
                       maxClearance=value;
                   }
             } else if(typeLP==1) {
                    value=dm->Distance(_env,neighbors[i],_c2,info->dmsetid);
                    if (value<minDistance) {
                        retPosition=i;
                        minDistance=value;
                    }
             }

         }
         p=neighbors[retPosition];

     }
     nTicks++;   

     if(info->savePath || info->saveFailedPath)
            info->path.push_back(p);

     if ((++noTries> 6*n_ticks)) {
         connected = false;
	 info->savedEdge = pair<Cfg,Cfg>(_c1, p);
	 break;
     }

  } while(!p.AlmostEqual(_c2));

  info->nTicks = nTicks;

  Stats.IncLPCollDetCalls("AStar", info->cd_cntr );
  if(connected)
      Stats.IncLPConnections( "AStar" );
  return connected;

};


bool
LocalPlanners::
UsesPlannerOtherThan(char plannerName[], SID lpsetid){

  vector<LP> lpset = planners.GetLPSet(lpsetid);

  for (vector<LP>::iterator lp=lpset.begin();lp<lpset.end();++lp){
    cout << "\n\t UsesPlanner: "<<lp->GetName();
    if ( strcmp(lp->GetName(),plannerName) ) 
	return true;
  }
  return false;
};


/////////////////////////////////////////////////////////////////////
//
//  METHODS for class LP
//      
/////////////////////////////////////////////////////////////////////

LP::
LP() {
  strcpy(name,"");
  planner = 0; 
  sValue = tries = neighbors = 0;
  lpid = INVALID_EID;
  forwardEdge = backEdge = 0;
};

LP::
~LP() {
};

bool 
LP::
operator==(const LP& _lp) const
{
  if ( strcmp(name,_lp.name) != 0 ) {
     return false;
  } else if ( !strcmp(name,"straightline") ) {
     return true;
  } else if ( !strcmp(name,"rotate_at_s") ) {
     return ( sValue == _lp.sValue );
  } else {  // ( strstr(name,"a_star") ) 
     return ( tries==_lp.tries 
           && neighbors==_lp.neighbors );
  }
};


char* 
LP::
GetName() const {
  return const_cast<char*>(name);
};

LPF 
LP::
GetPlanner(){
  return planner;
};


double 
LP::
GetS() const {
  if ( !strcmp(name,"rotate_at_s") ) {
    return sValue;
  } else {
    return -1;
  }
};

int 
LP::
GetTries() const {
  if ( strstr(name,"a_star") ) {
    return tries;
  } else {
    return -1;
  }
};

int 
LP::
GetNeighbors() const {
  if ( strstr(name,"a_star") ) {
    return neighbors;
  } else {
    return -1;
  }
};

EID 
LP::
GetID() const {
  return lpid;
};

int 
LP::
GetFEdgeMask() const {
  return forwardEdge;
};

int 
LP::
GetBEdgeMask() const {
  return backEdge;
};


ostream& operator<< (ostream& _os, const LP& lp) {
        _os<< lp.GetName();
        if ( !strcmp(lp.GetName(),"rotate_at_s") ){
           _os<< ", s=" << lp.GetS();
        }
        if ( strstr(lp.GetName(),"a_star") ){
           _os<< ", tries=" << lp.GetTries(); 
           _os<< ", neighbors=" << lp.GetNeighbors();
        }
        _os << ", fedge=" << lp.GetFEdgeMask();
        _os << ", bedge=" << lp.GetBEdgeMask();
        return _os;
};

/////////////////////////////////////////////////////////////////////
//
//  METHODS for class LPSets
//      
/////////////////////////////////////////////////////////////////////

  //==================================
  // LPSets class Methods: Constructors and Destructor
  //==================================
  // LPSets();
  // ~LPSets();

LPSets::
LPSets(){
};

LPSets::
~LPSets(){
};

  //===================================================================
  // LPSets class Methods: Adding LPs, Making & Modifying LP sets
  //===================================================================
  // EID AddLP(const char* _lpinfo); 
  // int AddLPToSet(const SID _sid, const EID _lpid); 
  // int DeleteLPFromSet(const SID _sid, const EID _lpid); 
  // SID MakeLPSet(const char* lplist);  // make an ordered set of lps, 
  // SID MakeLPSet(istream& _myistream); //  - add lp to universe if not there
  // SID MakeLPSet(const EID _eid);
  // SID MakeLPSet(const vector<EID> _eidvector);
  // int DeleteLPSet(const SID _sid);

int 
LPSets::
AddLP(const char* _lpinfo) {
  SID sid = MakeLPSet(_lpinfo); 
  SetIDs--;
  return DeleteOSet(sid);        // delete the set, but not elements
};


int 
LPSets::
AddLPToSet(const SID _sid, const EID _lpid) {
  return AddElementToOSet(_sid,_lpid);
};

int 
LPSets::
DeleteLPFromSet(const SID _sid, const EID _lpid) {
  return DeleteElementFromOSet(_sid,_lpid);
};

SID 
LPSets::
MakeLPSet(const char* _lplist){
  istrstream  is(_lplist);
  if (!is) {
         cout << endl << "In MakeLPSet: can't open instring: " << _lplist ;
         return INVALID_SID;
  }
  return MakeLPSet(is);  
};

SID
LPSets::
MakeLPSet(const EID _eid) {
  return MakeOSet(_eid);
}

SID
LPSets::
MakeLPSet(const vector<EID> _eidvector) {
  return MakeOSet(_eidvector);
}

int
LPSets::
DeleteLPSet(const SID _sid) { 
  return DeleteOSet(_sid);
}


SID 
LPSets:: 
MakeLPSet(istream& _myistream) { 
  char lpname[100]; 
  double sValue;
  vector<EID> lpvec;  // vector of lpids for this set 

  while ( _myistream >> lpname ) { // while lps to process...  
    if (!strcmp(lpname,"straightline")) {              // STRAIGHTLINE 
       LP lp1; 
       strcpy(lp1.name,lpname);
       //lp1.planner = &(lpPtr->IsConnected_straightline);
       lp1.lpid = AddElementToUniverse(lp1); 
       lp1.forwardEdge = lp1.backEdge = 1 << lp1.lpid;
       if ( ChangeElementInfo(lp1.lpid,lp1) != OK ) {
          cout << endl << "In MakeSet: couldn't change element info";
          exit(-1);
       }
       lpvec.push_back( lp1.lpid ); 


    } else if (!strcmp(lpname,"rotate_at_s")) {          // ROTATE-AT-S
       LP lp1, lp2;
       strcpy(lp1.name,lpname);
       //lp1.planner = &(lpPtr->IsConnected_rotate_at_s);
       lp1.sValue = 2.0; 
       lp2 = lp1;
       while ( _myistream >> sValue ) { //get s values 
          if ( sValue < 0 || sValue > 1 ) {
            cout << endl << "INVALID: rotate_at_s s=" << sValue;
            exit(-1);
          } else {
            //lp1.sValue = sValue;
            lp2.sValue = 1.0 - sValue;
            lp1.sValue = 1.0 - lp2.sValue;
            lp1.lpid = AddElementToUniverse(lp1);
            lp2.lpid = AddElementToUniverse(lp2);
            lp1.forwardEdge = lp2.backEdge = 1 << lp1.lpid;
            lp1.backEdge = lp2.forwardEdge = 1 << lp2.lpid;
            if ( ChangeElementInfo(lp1.lpid,lp1) != OK 
              || ChangeElementInfo(lp2.lpid,lp2) != OK) {
               cout << endl << "In MakeSet: couldn't change element info";
               exit(-1);
            }
            lpvec.push_back( lp1.lpid );
          }
       }
       if (lp1.sValue == 2.0) {  //if no s value given, use default 0.5 
          lp1.sValue = 0.5;
          lp1.lpid = AddElementToUniverse(lp1);
          lp1.forwardEdge = lp1.backEdge = 1 << lp1.lpid;
          if ( ChangeElementInfo(lp1.lpid,lp1) != OK ) {
              cout << endl << "In MakeSet: couldn't change element info";
              exit(-1);
          }
          lpvec.push_back( lp1.lpid );
       }
       _myistream.clear(); // clear failure to read in last s value

    } else if (!strcmp(lpname,"a_star_clearance") ||      // A-STARs
               !strcmp(lpname,"a_star_distance")) {   
       LP lp1; 
       strcpy(lp1.name,lpname);
       //lp1.planner = &(lpPtr->IsConnected_astar);

       if ( _myistream >> lp1.tries
         && _myistream >> lp1.neighbors ) {
          
          if (lp1.tries < 1)         // allow between 1-20
             lp1.tries = 1;
          if (lp1.tries > 20)
             lp1.tries = 20;

          if (lp1.neighbors <= 3) {   // only allow 3, 9, or 15
             lp1.neighbors = 3;
          } else if (lp1.neighbors > 9) {
             lp1.neighbors = 15;
          } else {
             lp1.neighbors = 9;
          }

       } else { // use defaults
          lp1.tries = 6;
          lp1.neighbors = 3;
          _myistream.clear(); // clear failure to read parameters
       }

       lp1.lpid = AddElementToUniverse(lp1);
       lp1.forwardEdge = lp1.backEdge = 1 << lp1.lpid;
       if ( ChangeElementInfo(lp1.lpid,lp1) != OK ) {
          cout << endl << "In MakeSet: couldn't change element info";
          exit(-1);
       }
       lpvec.push_back( lp1.lpid );

    } else {
       cout << "INVALID: local planner name = " << lpname;
       exit(-1);
    }
  } // end while 

  return MakeOSet(lpvec);
}

  //===================================================================
  // LPSets class Methods: Getting Data & Statistics
  //===================================================================
  // LP GetLP(const EID _lpid) const;
  // vector<LP> GetLPs() const;
  // vector<LP> GetLPSet(const SID _sid) const;
  // vector<pair<SID,vector<LP> > > GetLPSets() const;

LP
LPSets::
GetLP(const EID _lpid) const {
   return GetElement(_lpid);
};

vector<LP> 
LPSets::
GetLPs() const {
  vector<LP> elts2; 
  vector<pair<EID,LP> > elts1 = GetElements(); 
  for (int i=0; i < elts1.size(); i++) 
     elts2.push_back( elts1[i].second );
  return elts2; 
};

vector<LP> 
LPSets::
GetLPSet(const SID _sid) const {
  vector<LP> elts2; 
  vector<pair<EID,LP> > elts1 = GetOSet(_sid); 
  for (int i=0; i < elts1.size(); i++) 
     elts2.push_back( elts1[i].second );
  return elts2; 
};


vector<pair<SID,vector<LP> > > 
LPSets::
GetLPSets() const {

  vector<pair<SID,vector<LP> > > s2;
  vector<LP> theselps;

  vector<pair<SID,vector<pair<EID,LP> > > > s1 = GetOSets(); 

  for (int i=0; i < s1.size(); i++)  {
    theselps.erase(theselps.begin(),theselps.end());
    for (int j=0; j < s1[i].second.size(); j++ ) 
       theselps.push_back (s1[i].second[j].second);
    s2.push_back( pair<SID,vector<LP> > (s1[i].first,theselps) );
  }
  return s2; 
};



  //===================================================================
  // LPSets class Methods: Display, Input, Output
  //===================================================================
  // void DisplayLPs() const;    
  // void DisplayLP(const EID _lpid) const;    
  // void DisplayLPSets() const; 
  // void DisplayLPSet(const SID _sid) const; 
  // void WriteLPs(const char* _fname) const;
  // void WriteLPs(ostream& _myostream) const;
  // void ReadLPs(const char* _fname) const;
  // void ReadLPs(istream& _myistream) const;



void 
LPSets::
DisplayLPs() const{
   DisplayElements();
};

void 
LPSets::
DisplayLP(const EID _lpid) const{
   DisplayElement(_lpid);
};

void 
LPSets::
DisplayLPSets() const{
   DisplayOSets();
};

void 
LPSets::
DisplayLPSet(const SID _sid) const{
   DisplayOSet(_sid);
};

void
LPSets::
WriteLPs(const char* _fname) const {

      ofstream  myofstream(_fname);
      if (!myofstream) {
         cout << endl << "In WriteLPS: can't open outfile: " << _fname ;
      }
      WriteLPs(myofstream);
      myofstream.close();
};

void
LPSets::
WriteLPs(ostream& _myostream) const {

      vector<LP> lps = GetLPs();

      _myostream << endl << "#####LPSTART#####";
      _myostream << endl << lps.size();  // number of lps

      //format: LP_NAME (a string) LP_PARMS (double, int, etc)
      for (int i = 0; i < lps.size() ; i++) {
          _myostream << endl;
          _myostream << lps[i].name << " ";
          if ( !strcmp(lps[i].name,"rotate_at_s") ) {
             _myostream << lps[i].sValue;
          }
          if ( strstr(lps[i].name,"a_star") ) {
             _myostream << lps[i].tries << " " << lps[i].neighbors;
          }
      }
      _myostream << endl << "#####LPSTOP#####";
};

void
LPSets::
ReadLPs(const char* _fname) {

      ifstream  myifstream(_fname);
      if (!myifstream) {
         cout << endl << "In ReadLPs: can't open infile: " << _fname ;
         return;
      }
      ReadLPs(myifstream);
      myifstream.close();
};

void
LPSets::
ReadLPs(istream& _myistream) {

      char tagstring[100];
      char lpdesc[100];
      int  numLPs;

      _myistream >> tagstring;
      if ( !strstr(tagstring,"LPSTART") ) {
         cout << endl << "In ReadLPs: didn't read LPSTART tag right";
         return;
      }

      _myistream >> numLPs;
      _myistream.getline(lpdesc,100,'\n');  // throw out rest of this line
      for (int i = 0; i < numLPs; i++) {
        _myistream.getline(lpdesc,100,'\n');
        AddLP(lpdesc);
      }

      _myistream >> tagstring;
      if ( !strstr(tagstring,"LPSTOP") ) {
         cout << endl << "In ReadLPs: didn't read LPSTOP tag right";
         return;
      }
};

