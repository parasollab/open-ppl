// $Id$
/////////////////////////////////////////////////////////////////////
//
//  CollisionDetection.c
//
//  General Description
//
//  Created
//      8/11/98  Daniel Vallejo
//  Last Modified By:
//      1/13/99  Guang Song 'cfg' is removed from argument list.
//               Vclip collision detection is fixed so that 
//	         Both Vclip and cstk give the SAME results for collision checking!
// 
/////////////////////////////////////////////////////////////////////

#include "CollisionDetection.h"
#include "Roadmap.h"
#include "Stat_Class.h"

extern Stat_Class Stats; 

/////////////////////////////////////////////////////////////////////
//
//  METHODS for class CollisionDetection
//      
/////////////////////////////////////////////////////////////////////
  //==================================
  // CollisionDetection class Methods: Constructors and Destructor
  //==================================
  // CollisionDetection();
  // ~CollisionDetection();

CollisionDetection::
CollisionDetection() {
    DefaultInit();
};

CollisionDetection::
~CollisionDetection() {
};

  //==================================
  // CollisionDetection class Methods: Collision Detection Functions
  //==================================

void
CollisionDetection::
DefaultInit()
{

};


void
CollisionDetection::
UserInit(Input * input,  GenerateMapNodes* gn, ConnectMapNodes* cn)
{
   //-----------------------------------------------
   // initialize collision detection
   //  CAUTION:  DO NOT CHANGE ORDER OF SET DEFN's
   //           w/o CHANGING ENUM ORDER in "OBPRM.h"
   //-----------------------------------------------

    // initialize cd sets
#ifdef USE_CSTK
    collisionCheckers.MakeCDSet("cstk");	// enum CSTK
					      	// ie,c-space toolkit
#endif
                                                
#ifdef USE_VCLIP
    collisionCheckers.MakeCDSet("vclip");    // enum VCLIP
#endif
						// ie,voronoi clip
#ifdef USE_RAPID
    collisionCheckers.MakeCDSet("RAPID");    // enum RAPID
#endif

    if( input->numCDs == 0 ){                  	// use default CD sets
    } 
    else{                                     	// make user-defined sets
        gn->gnInfo.cdsetid=CD_USER1;
        cn->cnInfo.cdsetid=CD_USER1;
     	for (int i = 0; i < input->numCDs; i++) {
       	    collisionCheckers.MakeCDSet(input->CDstrings[i]->GetValue());
     	}
    }
};


#ifdef USE_VCLIP
VclipPose CollisionDetection::GetVclipPose(const Transformation &myT, 
		const Transformation &obstT) {

  //Transformation diff2 = Transformation(myT) - obstT;
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
//----------------------------------------------------------------------
// SetLineTransformation
// Guang Song 03/02/99
// set linTrans[12], which is used by cstk collision checker.
//----------------------------------------------------------------------
void CollisionDetection::
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
CollisionDetection::
IsInCollision(Environment * env, SID _cdsetid, MultiBody * lineRobot){

    int nmulti, robot;
    nmulti = env->GetMultiBodyCount();
    robot = env->GetRobotIndex();

    MultiBody * rob = env->GetMultiBody(robot);
    if(lineRobot) // A line Segment generated on the fly, to check if 'seemingly connectable'.
        rob = lineRobot;

    for(int i = 0 ; i < nmulti ; i++){
        if(i != robot ){
            if(IsInCollision(env, rob, env->GetMultiBody(i), _cdsetid)){
                return true;
            }
        } else { 
        // robot self checking. Warning: rob and env->GetMultiBody(robot) may NOT be the same.
	    if(rob->GetBodyCount() > 1 && IsInCollision(env, rob, rob, _cdsetid))
	        return true;
	}
    }
    return false;
};


double
CollisionDetection::
Clearance(Environment * env){
#ifdef USE_CSTK

    int nmulti, robot;
    nmulti = env->GetMultiBodyCount();
    robot = env->GetRobotIndex();

    MultiBody *rob, *obst;
    rob = env->GetMultiBody(robot);

    double tmp, dist = MaxDist;
    for(int i = 0 ; i < nmulti ; i++){
        if(i != robot){
    	    obst = env->GetMultiBody(i);
    	    tmp = cstkDistance(rob, obst);
	    if(tmp < dist){
		dist = tmp;
	    }
	}
    }
    return dist;
#else
     cout << "Clearance function is not supported by current collision detection library. \n Please recompile with a supporting library.\n";
    exit(5);
#endif
}


bool
CollisionDetection::
IsInCollision(Environment * env, int robot, int obstacle, SID _cdsetid) {

    MultiBody *rob, *obst;
    rob = env->GetMultiBody(robot);
    obst = env->GetMultiBody(obstacle);

    return IsInCollision(env, rob, obst, _cdsetid);
}


bool
CollisionDetection::
IsInCollision(Environment * env, MultiBody * rob, MultiBody * obst, SID _cdsetid) {

    int nFreeRobot;
    nFreeRobot = rob->GetFreeBodyCount();
 
    vector<CD> cdset = collisionCheckers.GetCDSet(_cdsetid);
    for(int cd = 0 ; cd < cdset.size() ; cd++){

	CDF cdfcn = cdset[cd].GetCollisionDetection();
	int tp = cdset[cd].GetType();

	// Type Out: no collision sure; collision unsure.
	if((tp == Out) && (cdfcn(rob,obst,cdset[cd]) == false)){
	    return false;
	}

	// Type In: no collision unsure; collision sure.
	if((tp == In) && (cdfcn(rob,obst,cdset[cd]) == true)){
	    return true;
	}

	// Type Exact: no collision sure; collision sure.
	if(tp == Exact){
	    if(cdfcn(rob,obst,cdset[cd]) == true){
	    	return true;
	    }
	    else{	
	    	return false;
	    }
	}
    }
    return true;
}


bool 
CollisionDetection::
IsInCollision_boundingSpheres(MultiBody* robot, MultiBody* obstacle,  CD& _cd){
 	cout << endl << "boundingSpheres Collision Check invocation";
   Stats.IncNumCollDetCalls( "boundingSpheres" );
   return true;
}

bool 
CollisionDetection::
IsInCollision_insideSpheres(MultiBody* robot, MultiBody* obstacle,  CD& _cd){
 	cout << endl << "insideSpheres Collision Check invocation";
   Stats.IncNumCollDetCalls( "insideSpheres" );
   return false;
}

bool 
CollisionDetection::
IsInCollision_naive(MultiBody* robot, MultiBody* obstacle,  CD& _cd){
 	cout << endl << "naive Collision Check invocation";
   Stats.IncNumCollDetCalls( "naive" );
   return false;
}

bool 
CollisionDetection::
IsInCollision_quinlan(MultiBody* robot, MultiBody* obstacle,  CD& _cd){
 	cout << endl << "Quinlan Collision Check invocation";
   Stats.IncNumCollDetCalls( "quinlan" );
   return false;
}


// hash table used by "vclip"

#ifdef USE_VCLIP
ClosestFeaturesHT closestFeaturesHT(3000);
bool 
CollisionDetection::
IsInCollision_vclip(MultiBody* robot, MultiBody* obstacle,  CD& _cd){
   Stats.IncNumCollDetCalls( "vclip" );

    Real dist;
    VclipPose X12;
    PolyTree *rob, *obst; 
    Vect3 cp1, cp2;   // closest points between bodies, in local frame
                      // we're throwing this info away for now

    for(int i=0 ; i<robot->GetFreeBodyCount(); i++){

         rob = robot->GetFreeBody(i)->GetVclipBody();

         for(int j=0; j<obstacle->GetBodyCount(); j++){

            // if robot check self collision, skip adjacent links.
            if(robot == obstacle &&
               robot->GetFreeBody(i)->isAdjacent(obstacle->GetBody(j)) )
                   continue;

            obst = obstacle->GetBody(j)->GetVclipBody();
            X12 = GetVclipPose(robot->GetFreeBody(i)->WorldTransformation(),
                  obstacle->GetBody(j)->WorldTransformation());
            dist = PolyTree::vclip(rob,obst,X12,closestFeaturesHT, cp1, cp2);

            //if(dist < 0.001) {
	    if(dist < 0.0) {
                return (true);
            }
         }
    }
    return false;
}

#endif
#ifdef USE_RAPID
bool
CollisionDetection::
IsInCollision_RAPID(MultiBody* robot, MultiBody* obstacle,  CD& _cd){
   Stats.IncNumCollDetCalls( "RAPID" );

    RAPID_model *rob, *obst;

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
		cout << "Error in CollisionDetection::RAPID_Collide, RAPID_ERR_COLLIDE_OUT_OF_MEMORY" << RAPID_Collide(t1.orientation.matrix, p1, rob, t2.orientation.matrix, p2, obst, RAPID_FIRST_CONTACT) << endl;
		exit(1);
	    }
	    if(RAPID_num_contacts)
		return true;

         }
    }
    return false;
}
#endif

#ifdef USE_CSTK
bool 
CollisionDetection::
IsInCollision_cstk(MultiBody* robot, MultiBody* obstacle,  CD& _cd){

    Stats.IncNumCollDetCalls( "cstk" );

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
		//free (wits);
                return (true);
            }
        }
    }
    return false;
}

double
CollisionDetection::
cstkDistance(MultiBody* robot, MultiBody* obstacle){

    Stats.IncNumCollDetCalls( "cstkDistance" );

    double linTrans[12] = {1,0,0,0, 0,1,0,0, 0,0,1,0};  // Identity in row-major order

    void *rob, *obst;
    cstkReal tmp, dist = MaxDist;

    for(int i = 0 ; i < robot->GetFreeBodyCount(); i++){
        rob = robot->GetFreeBody(i)->GetCstkBody();
	SetLineTransformation(robot->GetFreeBody(i)->WorldTransformation(), linTrans);
        cstkUpdateLMovableBody(rob, linTrans);
        for(int k = 0 ; k < obstacle->GetFixedBodyCount(); k++){
            obst = obstacle->GetFixedBody(k)->GetCstkBody();
            // tmp = cstkBodyBodyDist(rob, obst, 500, 0, NULL, 0, wits);
            tmp = cstkBodyBodyDist(rob, obst, 500, 0, NULL, 0, NULL);
            if(tmp < dist){
                //free (wits);
                dist = tmp;  
            }
        }
    }
    //free (wits);
    return dist;
}

#endif




/////////////////////////////////////////////////////////////////////
//
//  METHODS for class CD
//      
/////////////////////////////////////////////////////////////////////

CD::
CD() {
  strcpy(name,"");
  collision_detection = 0;
  cdid = INVALID_EID;
  type = -1;
}

CD::
~CD() {
}

CD&
CD::
operator=(const CD& _cd) {
  strcpy(name,_cd.name);
  return *this;
};

bool 
CD::
operator==(const CD& _cd) const
{
  if ( strcmp(name,_cd.name) != 0 ) {
     return false;
  } else if ( !strcmp(name,"boundingSpheres") ) {
     return true;
  } else if ( !strcmp(name,"insideSpheres") ) {
     return true;
  } else if ( !strcmp(name,"naive") ) {
     return true;
  } else if ( !strcmp(name,"quinlan") ) {
     return true; 
  } else if ( !strcmp(name,"cstk") ) {
#ifdef USE_CSTK
     return true; 
#else
    cout << "Current compilation does not include CSTK.\n Please recompile with CSTK if you'd like to use cstk option\n";
    exit(5);
 
#endif
  } else if ( !strcmp(name,"vclip") ) {
#ifdef USE_VCLIP
     return true;
#else
    cout << "Current compilation does not include VCLIP.\n Please recompile with
VCLIP if you'd like to use VCLIP option\n";
    exit(5);
#endif

  } else if ( !strcmp(name,"RAPID") ) {
#ifdef USE_RAPID
     return true;
#else
    cout << "Current compilation does not include RAPID.\n Please recompile with
RAPID if you'd like to use cstk option\n";
    exit(5);

#endif
  } else {  
     return false; 
  }
};

char* 
CD::
GetName() const {
  return const_cast<char*>(name);
};

CDF 
CD::
GetCollisionDetection(){
  return collision_detection;
};

int
CD::
GetType() const {
  return type;
};

ostream& operator<< (ostream& _os, const CD& cd){
        _os<< cd.GetName();
        if ( !strcmp(cd.GetName(),"boundingSpheres")){
           _os << ", Type = " << cd.GetType(); 
        }
        if ( !strcmp(cd.GetName(),"insideSpheres")){
           _os << ", Type = " << cd.GetType(); 
        }
        if ( !strcmp(cd.GetName(),"naive") ){
           _os << ", Type = " << cd.GetType(); 
        }
        if ( strstr(cd.GetName(),"quinlan") ){
           _os << ", Type = " << cd.GetType(); 
        }

        if ( strstr(cd.GetName(),"cstk") ){
#ifdef USE_CSTK
           _os << ", Type = " << cd.GetType(); 
#else
    cout << "Current compilation does not include CSTK.\n Please recompile with CSTK if you'd like to use cstk option\n";
    exit(5);
#endif
        }
        if ( strstr(cd.GetName(),"vclip") ){
#ifdef USE_VCLIP
           _os << ", Type = " << cd.GetType();
#else
    cout << "Current compilation does not include VCLIP.\n Please recompile with
VCLIP if you'd like to use vclip option\n";
    exit(5);
#endif
	}
        if ( strstr(cd.GetName(),"RAPID") ){
#ifdef USE_RAPID
           _os << ", Type = " << cd.GetType();
#else
    cout << "Current compilation does not include RAPID.\n Please recompile with
RAPID if you'd like to use rapid option\n";
    exit(5);
#endif

        }
        return _os;
};



/////////////////////////////////////////////////////////////////////
//
//  METHODS for class CDSets
//      
/////////////////////////////////////////////////////////////////////

  //==================================
  // CDSets class Methods: Constructors and Destructor
  //==================================

CDSets::
CDSets(){
};

CDSets::
~CDSets(){
};


  //===================================================================
  // CDSets class Methods: Adding CDs, Making & Modifying CD sets
  //===================================================================

int
CDSets::
AddCD(const char* _cdinfo) {
  SID sid = MakeCDSet(_cdinfo);
  SetIDs--;
  return DeleteOSet(sid);        // delete the set, but not elements
};

int
CDSets::
AddCDToSet(const SID _sid, const EID _cdid) {
  return AddElementToOSet(_sid,_cdid);
};

int
CDSets::
DeleteCDFromSet(const SID _sid, const EID _cdid) {
  return DeleteElementFromOSet(_sid,_cdid);
};


SID
CDSets::
MakeCDSet(const char* _cdlist){
    istrstream  is(_cdlist);
  if (!is) {
         cout << endl << "In MakeCDSet: can't open instring: " << _cdlist ;
         return INVALID_SID;
  }
  return MakeCDSet(is);
};


SID
CDSets::
MakeCDSet(const EID _eid) {
  return MakeOSet(_eid);
}

SID
CDSets::
MakeCDSet(const vector<EID> _eidvector) {
  return MakeOSet(_eidvector);
}

SID
CDSets::
MakeCDSet(istream& _myistream) {
  char cdname[100];
  vector<EID> cdvec;  // vector of cdids for this set

  while ( _myistream >> cdname ) { // while cds to process...

    if (!strcmp(cdname,"boundingSpheres")) {              // boundingSpheres
       CD cd1;
       strcpy(cd1.name,cdname);
       cd1.collision_detection = &CollisionDetection::IsInCollision_boundingSpheres;
       cd1.type = Out;	
       cd1.cdid = AddElementToUniverse(cd1);
       if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
          cout << endl << "In MakeSet: couldn't change element info";
          exit(-1);
       }
       cdvec.push_back( cd1.cdid );

    } else if (!strcmp(cdname,"insideSpheres")) {              // insideSpheres
       CD cd1;
       strcpy(cd1.name,cdname);
       cd1.collision_detection = &CollisionDetection::IsInCollision_insideSpheres;
       cd1.type = In;	
       cd1.cdid = AddElementToUniverse(cd1);
       if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
          cout << endl << "In MakeSet: couldn't change element info";
          exit(-1);
       }
       cdvec.push_back( cd1.cdid );

    } else if (!strcmp(cdname,"naive")) {              // naive
       CD cd1;
       strcpy(cd1.name,cdname);
       cd1.collision_detection = &CollisionDetection::IsInCollision_naive;
       cd1.type = Exact;	
       cd1.cdid = AddElementToUniverse(cd1);
       if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
          cout << endl << "In MakeSet: couldn't change element info";
          exit(-1);
       }
       cdvec.push_back( cd1.cdid );

    } else if (!strcmp(cdname,"quinlan")) {          // Quinlan
       CD cd1;
       strcpy(cd1.name,cdname);
       cd1.collision_detection = & CollisionDetection::IsInCollision_quinlan;
       cd1.type = Exact;	
       cd1.cdid = AddElementToUniverse(cd1);
       if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
          cout << endl << "In MakeSet: couldn't change element info";
          exit(-1);
       }
       cdvec.push_back( cd1.cdid );

    } else if (!strcmp(cdname,"cstk")) {          // cstk
#ifdef USE_CSTK
       CD cd1;
       strcpy(cd1.name,cdname);
       cd1.collision_detection = & CollisionDetection::IsInCollision_cstk;
       cd1.type = Exact;	
       cd1.cdid = AddElementToUniverse(cd1);
       if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
          cout << endl << "In MakeSet: couldn't change element info";
          exit(-1);
       }
       cdvec.push_back( cd1.cdid );
#else
    cout << "Current compilation does not include CSTK.\n Please recompile with CSTK if you'd like to use cstk option\n";
    exit(5);
#endif

    } else if (!strcmp(cdname,"vclip")) {          // vclip
#ifdef USE_VCLIP

       CD cd1;
       strcpy(cd1.name,cdname);
       cd1.collision_detection = & CollisionDetection::IsInCollision_vclip;
       cd1.type = Exact;
       cd1.cdid = AddElementToUniverse(cd1);
       if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
          cout << endl << "In MakeSet: couldn't change element info";
          exit(-1);
       }
       cdvec.push_back( cd1.cdid );
#else
    cout << "Current compilation does not include VCLIP.\n Please recompile with
VCLIP if you'd like to use vclip option\n";
    exit(5);
#endif


    } else if (!strcmp(cdname,"RAPID")) {          // RAPID
#ifdef USE_RAPID

       CD cd1;
       strcpy(cd1.name,cdname);
       cd1.collision_detection = & CollisionDetection::IsInCollision_RAPID;
       cd1.type = Exact;
       cd1.cdid = AddElementToUniverse(cd1);
       if ( ChangeElementInfo(cd1.cdid,cd1) != OK ) {
          cout << endl << "In MakeSet: couldn't change element info";
          exit(-1);
       }
       cdvec.push_back( cd1.cdid );

#else
    cout << "Current compilation does not include RAPID.\n Please recompile with
RAPID if you'd like to use rapid option\n";
    exit(5);
#endif

    } else {
       cout << "INVALID: Collision Detection name = " << cdname;
       exit(-1);
    }
  } // end while

  return MakeOSet(cdvec);
}


int
CDSets::
DeleteCDSet(const SID _sid) {
  return DeleteOSet(_sid);
};

  //===================================================================
  // CDSets class Methods: Getting Data & Statistics
  //===================================================================

CD
CDSets::
GetCD(const EID _cdid) const {
   return GetElement(_cdid);
};

vector<CD>
CDSets::
GetCDs() const {
  vector<CD> elts2;
  vector<pair<EID,CD> > elts1 = GetElements();
  for (int i=0; i < elts1.size(); i++)
     elts2.push_back( elts1[i].second );
  return elts2;
};

vector<CD>
CDSets::
GetCDSet(const SID _sid) const {
  vector<CD> elts2;
  vector<pair<EID,CD> > elts1 = GetOSet(_sid);
  for (int i=0; i < elts1.size(); i++)
     elts2.push_back( elts1[i].second );
  return elts2;
};


vector<pair<SID,vector<CD> > >
CDSets::
GetCDSets() const {

  vector<pair<SID,vector<CD> > > s2;
  vector<CD> thesecds;

  vector<pair<SID,vector<pair<EID,CD> > > > s1 = GetOSets();

  for (int i=0; i < s1.size(); i++)  {
    thesecds.erase(thesecds.begin(),thesecds.end());
    for (int j=0; j < s1[i].second.size(); j++ )
       thesecds.push_back (s1[i].second[j].second);
    s2.push_back( pair<SID,vector<CD> > (s1[i].first,thesecds) );
  }
  return s2;
};


    //===================================================================
    // CDSets class Methods: Display, Input, Output
    //===================================================================

void
CDSets::
DisplayCDs() const{
   DisplayElements();
};

void
CDSets::
DisplayCD(const EID _cdid) const{
   DisplayElement(_cdid);
};

void
CDSets::
DisplayCDSets() const{
   DisplayOSets();
};

void
CDSets::
DisplayCDSet(const SID _sid) const{
   DisplayOSet(_sid);
};

void
CDSets::
WriteCDs(const char* _fname) const {

      ofstream  myofstream(_fname);
      if (!myofstream) {
         cout << endl << "In WriteCDS: can't open outfile: " << _fname ;
      }
      WriteCDs(myofstream);
      myofstream.close();
};

void
CDSets::
WriteCDs(ostream& _myostream) const {

      vector<CD> cds = GetCDs();

      _myostream << endl << "#####CDSTART#####";
      _myostream << endl << cds.size();  // number of cds

      //format: CD_NAME (a string) CD_PARMS (double, int, etc)
      for (int i = 0; i < cds.size() ; i++) {
          _myostream << endl;
          _myostream << cds[i].name << " ";
      }
      _myostream << endl << "#####CDSTOP#####";
};

void
CDSets::
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
CDSets::
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
        AddCD(cddesc);
      }

      _myistream >> tagstring;
      if ( !strstr(tagstring,"CDSTOP") ) {
         cout << endl << "In ReadCDs: didn't read CDSTOP tag right";
         return;
      }
};



