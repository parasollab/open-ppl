// $Id$
/////////////////////////////////////////////////////////////////////
//
//  DistanceMetrics.c
//
//  General Description
//
//  Created
//      8/21/98  Daniel Vallejo
//
/////////////////////////////////////////////////////////////////////
#include "DistanceMetrics.h"

#include "LocalPlanners.h"
#include "GenerateMapNodes.h"
#include "Environment.h"
#include "util.h"
#include "Input.h"

/////////////////////////////////////////////////////////////////////
//
//  METHODS for class DistanceMetric
//      
/////////////////////////////////////////////////////////////////////
//==================================
// DistanceMetric class Methods: Constructors and Destructor
//==================================

DistanceMetric::
DistanceMetric() {
	DefaultInit();
};

DistanceMetric::
~DistanceMetric() {
};

//==================================
// DistanceMetric class Methods: Distance Metric Functions
//==================================
//-----------------------------------------------
// initialize default values for distance metrics
//  CAUTION:  DO NOT CHANGE ORDER OF SET DEFN's
//           w/o CHANGING ENUM ORDER in "OBPRM.h"
//-----------------------------------------------
void
DistanceMetric::
DefaultInit() {
	// initialize dm sets
	// enum S_EUCLID9
	distanceMetrics.MakeDMSet("scaledEuclidean 0.9");
	// enum EUCLID
	distanceMetrics.MakeDMSet("euclidean");
	// enum minkowski
	distanceMetrics.MakeDMSet("minkowski 3 3 0.3333");
	// enum manhattan
	distanceMetrics.MakeDMSet("manhattan");
	// enum com
	distanceMetrics.MakeDMSet("com");
}

//-----------------------------------------------
// initialize distance metrics with data on command line
//  CAUTION:  DO NOT CHANGE ORDER OF SET DEFN's
//           w/o CHANGING ENUM ORDER in "OBPRM.h"
//-----------------------------------------------
void
DistanceMetric::
UserInit(Input *input, GenerateMapNodes* gn, LocalPlanners* lp) {
	if ( input->numDMs == 0 ) {           // use default DM sets
	} else {                             // make user-defined sets
		gn->gnInfo.dmsetid=DM_USER1;
		for (int i = 0; i < input->numDMs; i++) {
			distanceMetrics.MakeDMSet(input->DMstrings[i]->GetValue());
		}
	}
}

double
DistanceMetric::
Distance(Environment *env, Cfg _c1, Cfg _c2, SID _dmsetid){
	
    MultiBody* robot;
    robot = env->GetMultiBody(env->GetRobotIndex());
    double dist;
	
    vector<DM> dmset = distanceMetrics.GetDMSet(_dmsetid); 
    for(int dm = 0 ; dm < dmset.size() ; dm++){
		
		DMF dmfcn = dmset[dm].GetDistanceMetric();
        int tp = dmset[dm].GetType();
		dist = dmfcn(robot,_c1,_c2,dmset[dm]);
		
    }
    return dist;
};

double ScaledDistance(Cfg& _c1, Cfg& _c2, double sValue){
	
    Cfg tmp=_c1-_c2;
    return  sqrt(  sValue*sqr(tmp.PositionMagnitude()) + 
		(1.0 - sValue)*sqr(tmp.OrientationMagnitude()) );
}


double 
DistanceMetric::
EuclideanDistance(MultiBody* robot, Cfg& _c1, Cfg& _c2, DM& _dm){
	
    double dist;
    dist = sqrt(2.0)*ScaledDistance(_c1, _c2, 0.5); 
    return dist;
}

double
DistanceMetric::
ScaledEuclideanDistance(MultiBody* robot, Cfg& _c1, Cfg& _c2, DM& _dm){
	
    double dist;
    dist = ScaledDistance(_c1, _c2, _dm.GetS()); 
    return dist;
}
double
DistanceMetric::
MinkowskiDistance(MultiBody* robot, Cfg& _c1, Cfg& _c2, DM& _dm){
	
    double dist,pos=0,orient=0/*,d*/;
    Cfg c=_c1-_c2;
    vector<double> p=c.GetPosition(); // position values
    vector<double> o=c.GetOrientation(); //orientation values
	
	
	
    int i;
    for(i=0;i<p.size();i++) {
		if(p[i]<0) p[i]=-p[i];
		pos+=pow(p[i],_dm.GetR1());
    }
	
	for(i=0;i<o.size();i++) {
		orient+=pow(o[i],_dm.GetR2());
    }
	
	dist=pow(pos+orient,_dm.GetR3());
	return dist;
}                           

double
DistanceMetric::
ManhattanDistance(MultiBody* robot, Cfg& _c1, Cfg& _c2, DM& _dm){
	
    double dist=0;
    Cfg c=_c1-_c2;
    vector<double> dt=c.GetData(); // position values
	
	
	
    int i;
    for(i=0;i<dt.size();i++) {
		if(dt[i]<0) dist=dist-dt[i];
		else
			dist+=dt[i];
    }
	
	return dist;
}                                                                               

double DistanceMetric::
CenterOfMassDistance(MultiBody* robot, Cfg& _c1, Cfg& _c2, DM& _dm){
	
    Vector3D d=_c1.GetRobotCenterPosition()-_c2.GetRobotCenterPosition();
    return d.magnitude();
}
/////////////////////////////////////////////////////////////////////
//
//  METHODS for class DM
//      
/////////////////////////////////////////////////////////////////////

DM::
DM() {
	strcpy(name,"");
	distanceMetric = 0;
	dmid = INVALID_EID;
	type = -1;
	sValue = 0;
}

DM::
~DM() {
}

DM&
DM::
operator=(const DM& _dm) {
	strcpy(name,_dm.name);
	return *this;
};

bool 
DM::
operator==(const DM& _dm) const
{
	if ( strcmp(name,_dm.name) != 0 ) {
		return false;
	} else if ( !strcmp(name,"euclidean") ||
		!strcmp(name,"manhattan") || 
		!strcmp(name,"com")  ) {
		return true;
	} else if ( !strcmp(name,"scaledEuclidean") ) {
		return ( abs(sValue-_dm.sValue) <0.000000001);
	} else if ( !strcmp(name,"minkowski") ) {
		return ( (abs(r1 - _dm.r1 )<0.000000001) && 
			(abs(r2 - _dm.r2 ) <0.000000001)&&
			(abs(r3 - _dm.r3 )<0.000000001));
	} else {  
		return false; 
	}
};

char* 
DM::
GetName() const {
	return const_cast<char*>(name);
};

DMF 
DM::
GetDistanceMetric(){
	return distanceMetric;
};

int
DM::
GetType() const {
	return type;
};

ostream& operator<< (ostream& _os, const DM& dm){
	_os<< dm.GetName();
	if ( (!strcmp(dm.GetName(),"euclidean")) ||
		(!strcmp(dm.GetName(),"manhattan")) ||
		(!strcmp(dm.GetName(),"com"))    ){
		_os << ", Type = " << dm.GetType(); 
	}
	if ( !strcmp(dm.GetName(),"scaledEuclidean")){
		_os << ", Type = " << dm.GetType(); 
		_os << ", s = " << dm.GetS(); 
	}
	
	if ( !strcmp(dm.GetName(),"minkowski")){
		_os << ", Type = " << dm.GetType();
		_os << ", r1   = " << dm.GetR1();
		_os << ", r2   = " << dm.GetR2();
		_os << ", r3   = " << dm.GetR3();
	}
	
	return _os;
};

double
DM::
GetS() const {
    if( !strcmp(name,"scaledEuclidean") ){
		return sValue;
	} else {
		return -1;
	}
};

double
DM::
GetR1() const {
    if( !strcmp(name,"minkowski") ){
		return r1;
	} else {
		return -1;
	}
};
double
DM::
GetR2() const {
    if( !strcmp(name,"minkowski") ){
		return r2;
	} else {
		return -1;
	}
};
double
DM::
GetR3() const {
    if( !strcmp(name,"minkowski") ){
		return r3;
	} else {
		return -1;
	}
};          


/////////////////////////////////////////////////////////////////////
//
//  METHODS for class DMSets
//      
/////////////////////////////////////////////////////////////////////

//==================================
// DMSets class Methods: Constructors and Destructor
//==================================

DMSets::
DMSets(){
};

DMSets::
~DMSets(){
};


//===================================================================
// DMSets class Methods: Adding DMs, Making & Modifying DM sets
//===================================================================

int
DMSets::
AddDM(const char* _dminfo) {
	SID sid = MakeDMSet(_dminfo);
	SetIDs--;
	return DeleteOSet(sid);        // delete the set, but not elements
};

int
DMSets::
AddDMToSet(const SID _sid, const EID _dmid) {
	return AddElementToOSet(_sid,_dmid);
};

int
DMSets::
DeleteDMFromSet(const SID _sid, const EID _dmid) {
	return DeleteElementFromOSet(_sid,_dmid);
};


SID
DMSets::
MakeDMSet(const char* _dmlist){
	
	///Modified for VC
	istrstream  is( (char *) _dmlist);
	if (!is) {
		cout << endl << "In MakeDMSet: can't open instring: " << _dmlist ;
		return INVALID_SID;
	}
	
	return MakeDMSet(is);
};


SID
DMSets::
MakeDMSet(const EID _eid) {
	return MakeOSet(_eid);
}

SID
DMSets::
MakeDMSet(const vector<EID> _eidvector) {
	return MakeOSet(_eidvector);
}

SID
DMSets::
MakeDMSet(istream& _myistream) {
	char dmname[100];
	double sValue;
	vector<EID> dmvec;  // vector of dmids for this set
	
	while ( _myistream >> dmname ) { // while dms to process...
		
		if (!strcmp(dmname,"euclidean")) {              // Euclidean Distance Metric
			DM dm1;
			strcpy(dm1.name,dmname);
			dm1.distanceMetric = &DistanceMetric::EuclideanDistance;
			dm1.type = CS;	
			dm1.dmid = AddElementToUniverse(dm1);
			if ( ChangeElementInfo(dm1.dmid,dm1) != OK ) {
				cout << endl << "In MakeSet: couldn't change element info";
				exit(-1);
			}
			dmvec.push_back( dm1.dmid );
			
		} else if (!strcmp(dmname,"scaledEuclidean")){	// Scaled Euclidean
			DM dm1;
			strcpy(dm1.name,dmname);
			dm1.distanceMetric = &DistanceMetric::ScaledEuclideanDistance;
			dm1.type = CS;
			dm1.sValue = 2.0;
			while( _myistream >> sValue ) { //get s values
				if ( sValue < 0 || sValue > 1 ) {
					cout << endl << "INVALID: scaled Euclidean s= " << sValue;
					exit(-1);
				} else {
					dm1.sValue = sValue;
					dm1.dmid = AddElementToUniverse(dm1);
					if( ChangeElementInfo(dm1.dmid,dm1) != OK ) {
						cout << endl << "In MakeSet: couldn't change element info";
						exit(-1);
					}
					dmvec.push_back( dm1.dmid );
				}
			}
			if(dm1.sValue == 2.0) {  //if no s value given, use default 0.5
				dm1.sValue = 0.5;
				dm1.dmid = AddElementToUniverse(dm1);
				if( ChangeElementInfo(dm1.dmid,dm1) != OK ) {
					cout << endl << "In MakeSet: couldn't change element info";
					exit(-1);
				}
				dmvec.push_back( dm1.dmid );
			}
			_myistream.clear(); // clear failure to read in last s value
			
		} else if (!strcmp(dmname,"minkowski")){ 
			DM dm1;
			double r1,r2,r3;
			strcpy(dm1.name,dmname);
			dm1.distanceMetric = &DistanceMetric::MinkowskiDistance;
			dm1.type = CS;
			r1 = 3;
			r2 = 3;
			r3 = 1.0/3;
			if( _myistream >> r1) {
				r2=r3=r1;
				if( _myistream >> r2) {
					if (! (_myistream >> r3)) {
						cout << endl << "INVALID: If you specified 2 values please also specify the third";
						exit(-1);
					}
				}
			}
			dm1.r1 = r1;
			dm1.r2 = r2;
			dm1.r3 = r3; 
			dm1.dmid = AddElementToUniverse(dm1);
			if( ChangeElementInfo(dm1.dmid,dm1) != OK ) {
                cout << endl << "In MakeSet: couldn't change element info";
                exit(-1);
            }
			dmvec.push_back( dm1.dmid );
			
			_myistream.clear(); // clear failure to read in last r value
		}else  if (!strcmp(dmname,"manhattan")) {// manhattan
			DM dm1;
			strcpy(dm1.name,dmname);
			dm1.distanceMetric = &DistanceMetric::ManhattanDistance;
			dm1.type = CS;
			dm1.dmid = AddElementToUniverse(dm1);
			if ( ChangeElementInfo(dm1.dmid,dm1) != OK ) {
				cout << endl << "In MakeSet: couldn't change element info";
				exit(-1);
			}
			dmvec.push_back( dm1.dmid );
			
		}  else if (!strcmp(dmname,"com")) {// com
			DM dm1;
			strcpy(dm1.name,dmname);
			dm1.distanceMetric = &DistanceMetric::CenterOfMassDistance;
			dm1.type = WS;
			dm1.dmid = AddElementToUniverse(dm1);
			if ( ChangeElementInfo(dm1.dmid,dm1) != OK ) {
				cout << endl << "In MakeSet: couldn't change element info";
				exit(-1);
			}
			dmvec.push_back( dm1.dmid );
			
		} else {
			cout << "INVALID: Distance Metric name = " << dmname;
			exit(-1);
		}
  } // end while
  
  return MakeOSet(dmvec);
}


int
DMSets::
DeleteDMSet(const SID _sid) {
	return DeleteOSet(_sid);
};

//===================================================================
// DMSets class Methods: Getting Data & Statistics
//===================================================================

DM
DMSets::
GetDM(const EID _dmid) const {
	return GetElement(_dmid);
};

vector<DM>
DMSets::
GetDMs() const {
	vector<DM> elts2;
	vector<pair<EID,DM> > elts1 = GetElements();
	for (int i=0; i < elts1.size(); i++)
		elts2.push_back( elts1[i].second );
	return elts2;
};

vector<DM>
DMSets::
GetDMSet(const SID _sid) const {
	vector<DM> elts2;
	vector<pair<EID,DM> > elts1 = GetOSet(_sid);
	for (int i=0; i < elts1.size(); i++)
		elts2.push_back( elts1[i].second );
	return elts2;
};


vector<pair<SID,vector<DM> > >
DMSets::
GetDMSets() const {
	
	vector<pair<SID,vector<DM> > > s2;
	vector<DM> thesedms;
	
	vector<pair<SID,vector<pair<EID,DM> > > > s1 = GetOSets();
	
	for (int i=0; i < s1.size(); i++)  {
		thesedms.erase(thesedms.begin(),thesedms.end());
		for (int j=0; j < s1[i].second.size(); j++ )
			thesedms.push_back (s1[i].second[j].second);
		s2.push_back( pair<SID,vector<DM> > (s1[i].first,thesedms) );
	}
	return s2;
};

//===================================================================
// DMSets class Methods: Display, Input, Output
//===================================================================

void
DMSets::
DisplayDMs() const{
	DisplayElements();
};

void
DMSets::
DisplayDM(const EID _dmid) const{
	DisplayElement(_dmid);
};

void
DMSets::
DisplayDMSets() const{
	DisplayOSets();
};

void
DMSets::
DisplayDMSet(const SID _sid) const{
	DisplayOSet(_sid);
};

void
DMSets::
WriteDMs(const char* _fname) const {
	
	ofstream  myofstream(_fname);
	if (!myofstream) {
		cout << endl << "In WriteDMS: can't open outfile: " << _fname ;
	}
	WriteDMs(myofstream);
	myofstream.close();
};

void
DMSets::
WriteDMs(ostream& _myostream) const {
	
	vector<DM> dms = GetDMs();
	
	_myostream << endl << "#####DMSTART#####";
	_myostream << endl << dms.size();  // number of dms
	
	//format: DM_NAME (a string) DM_PARMS (double, int, etc)
	for (int i = 0; i < dms.size() ; i++) {
		_myostream << endl;
		_myostream << dms[i].name << " ";
		if ( !strcmp(dms[i].name,"scaledEuclidean") ) {
			_myostream << dms[i].sValue;
		}
		if ( !strcmp(dms[i].name,"minkowski") ) {
			_myostream << dms[i].r1 <<" ";
			_myostream << dms[i].r2 <<" ";
			_myostream << dms[i].r3;
		}      
	}
	_myostream << endl << "#####DMSTOP#####";
};

void
DMSets::
ReadDMs(const char* _fname) {
	
	ifstream  myifstream(_fname);
	if (!myifstream) {
		cout << endl << "In ReadDMs: can't open infile: " << _fname ;
		return;
	}
	ReadDMs(myifstream);
	myifstream.close();
};

void
DMSets::
ReadDMs(istream& _myistream) {
	
	char tagstring[100];
	char dmdesc[100];
	int  numDMs;
	
	_myistream >> tagstring;
	if ( !strstr(tagstring,"DMSTART") ) {
		cout << endl << "In ReadDMs: didn't read DMSTART tag right";
		return;
	}
	
	_myistream >> numDMs;
	_myistream.getline(dmdesc,100,'\n');  // throw out rest of this line
	for (int i = 0; i < numDMs; i++) {
        _myistream.getline(dmdesc,100,'\n');
        AddDM(dmdesc);
	}
	
	_myistream >> tagstring;
	if ( !strstr(tagstring,"DMSTOP") ) {
		cout << endl << "In ReadDMs: didn't read DMSTOP tag right";
		return;
	}
};
