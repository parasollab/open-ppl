// $Id$

/**
 * @file DistanceMetrics.c
 *
 * @author Daniel Vallejo
 * @date 8/21/1998
 */

////////////////////////////////////////////////////////////////////////////////////////////
#include "DistanceMetrics.h"

//#include "GenerateMapNodes.h"
#include "Environment.h"
#include "util.h"
#include "Input.h"


DistanceMetric::
DistanceMetric() {
  EuclideanDistance* euclidean = new EuclideanDistance();
  all.push_back(euclidean);

  ScaledEuclideanDistance* scaledEuclidean = new ScaledEuclideanDistance();
  all.push_back(scaledEuclidean);

  MinkowskiDistance* minkowski = new MinkowskiDistance();
  all.push_back(minkowski);

  ManhattanDistance* manhattan = new ManhattanDistance();
  all.push_back(manhattan);

  CenterOfMassDistance* com = new CenterOfMassDistance();
  all.push_back(com);
}


DistanceMetric::
~DistanceMetric() {
  vector<DistanceMetricMethod*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    delete *I;

  for(I=all.begin(); I!=all.end(); I++)
    delete *I;
}


vector<DistanceMetricMethod*> 
DistanceMetric::
GetDefault() {
  vector<DistanceMetricMethod*> Default;
  ScaledEuclideanDistance* scaledEuclidean = new ScaledEuclideanDistance(0.9);
  Default.push_back(scaledEuclidean);
  return Default;
}


bool 
DistanceMetric::
ParseCommandLine(int argc, char** argv) {
  bool found = FALSE;
  vector<DistanceMetricMethod*>::iterator itr;

  cout << "argc[" << argc << "]:\n";
  for(int i=0; i<argc; i++)
    cout << argv[i] << endl;
  
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
	  
	  vector<DistanceMetricMethod*>::iterator itr_names;
	  is_method_name = false;
	  for(itr_names=all.begin(); itr_names!=all.end() && cmd_begin+cmd_argc < argc; itr_names++)
	    if( !strcmp(argv[cmd_begin+cmd_argc], (*itr_names)->GetName())) {
	      is_method_name = true;
	      break;
	    } 
	} while(!is_method_name && cmd_begin+cmd_argc < argc);

	cout << "cmd_argc[" << cmd_argc << "]:\n";
	for(int i=0; i<cmd_argc; i++)
	  cout << cmd_argv[i] << endl;

	(*itr)->ParseCommandLine(cmd_argc, cmd_argv);
	selected.push_back((*itr)->CreateCopy());
	(*itr)->SetDefault();
	found = TRUE;
	break;
      }
    }
    cmd_begin = cmd_begin + cmd_argc;
  } while(cmd_begin < argc);

  return found;
}


int 
DistanceMetric::
ReadCommandLine(n_str_param* DMstrings[MAX_DM], int numDMs) {
  vector<DistanceMetricMethod*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    delete *I;
  selected.clear();

  for(int i=0; i<numDMs; i++) {
    istrstream _myistream(DMstrings[i]->GetValue());

    int argc = 0;
    char* argv[50];
    char cmdFields[50][100];
    while(_myistream >> cmdFields[argc]) {
      argv[argc] = (char*)(&cmdFields[argc]);
      argc++;
    }
   
    bool found = FALSE;
    try {
      found = ParseCommandLine(argc, argv);
      if (!found)
	throw BadUsage();
    } catch (BadUsage) {
      cerr << "Command line error" << endl;
      PrintUsage(cerr);
      exit(-1);
    }
  }

  if(selected.size() == 0)
    selected = GetDefault();
}


void 
DistanceMetric::
PrintUsage(ostream& _os) const {
  vector<DistanceMetricMethod*>::const_iterator I;
  for(I=all.begin(); I!=all.end(); I++)
    (*I)->PrintUsage(_os);
}


void 
DistanceMetric::
PrintValues(ostream& _os) const {
  vector<DistanceMetricMethod*>::const_iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    (*I)->PrintValues(_os);
}


void 
DistanceMetric::
PrintDefaults(ostream& _os) const { 
  vector<DistanceMetricMethod*> Default;
  Default = GetDefault();
  vector<DistanceMetricMethod*>::iterator I;
  for(I=Default.begin(); I!=Default.end(); I++)
    (*I)->PrintValues(_os);
  for(I=Default.begin(); I!=Default.end(); I++)
    delete (*I);
}


void 
DistanceMetric::
ReadDMs(const char* _fname) {
  ifstream  myifstream(_fname);
  if (!myifstream) {
    cout << endl << "In ReadDMs: can't open infile: " << _fname ;
    return;
  }
  ReadDMs(myifstream);
  myifstream.close();
}


void 
DistanceMetric::
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
    istrstream _dmstream(dmdesc);
    int argc = 0;
    char* argv[50];
    char cmdFields[50][100];
    while (_dmstream >> cmdFields[argc]) {
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
  if ( !strstr(tagstring,"DMSTOP") ) {
    cout << endl << "In ReadDMs: didn't read DMSTOP tag right";
    return;
  }
}


void 
DistanceMetric::
WriteDMs(const char* _fname) const {
  ofstream  myofstream(_fname);
  if (!myofstream) {
    cout << endl << "In WriteDMS: can't open outfile: " << _fname ;
  }
  WriteDMs(myofstream);
  myofstream.close();
}


void 
DistanceMetric::
WriteDMs(ostream& _myostream) const {
  _myostream << endl << "#####DMSTART#####";
  _myostream << endl << selected.size();  // number of dms
  PrintValues(_myostream);
  _myostream << endl << "#####DMSTOP#####"; 
}


double 
DistanceMetric::
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
  MultiBody* robot;
  robot = env->GetMultiBody(env->GetRobotIndex());
  return selected[0]->Distance(robot, _c1, _c2);
}


double 
DistanceMetric::
Distance(Environment* env, const Cfg* _c1, const Cfg* _c2) {
  MultiBody* robot;
  robot = env->GetMultiBody(env->GetRobotIndex());
  return selected[0]->Distance(robot, *_c1, *_c2);
}

//////////

DistanceMetricMethod::
DistanceMetricMethod() {
}


DistanceMetricMethod::
~DistanceMetricMethod() {
}


bool
DistanceMetricMethod::
operator==(const DistanceMetricMethod& dm) const {
  return ( !(strcmp(GetName(), dm.GetName())) );
}

//////////


EuclideanDistance::
EuclideanDistance() : DistanceMetricMethod() {
  type = CS;
}
 

EuclideanDistance::
~EuclideanDistance() {
}


char* 
EuclideanDistance::
GetName() const {
  return "euclidean";
}


void 
EuclideanDistance::
SetDefault() {
}


void 
EuclideanDistance::
ParseCommandLine(int argc, char** argv) {
  if(argc > 1) {
    cerr << "\nERROR ParseCommandLine: Don\'t understand \""
	 << argv << "\"\n\n";
    PrintUsage(cerr);
    cerr << endl;
    exit(-1);
  }
}

 
void 
EuclideanDistance::
PrintUsage(ostream& _os) const {
  _os.setf(ios::left,ios::adjustfield);

  _os << "\n" << GetName() << " ";

  _os.setf(ios::right,ios::adjustfield);
}


void 
EuclideanDistance::
PrintValues(ostream& _os) const {
  _os << "\n" << GetName() << " ";
  _os << endl;
}


DistanceMetricMethod* 
EuclideanDistance::
CreateCopy() {
  DistanceMetricMethod* _copy = new EuclideanDistance(*this);
  return _copy;
}


double 
EuclideanDistance::
Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2) {
  double dist;
  dist = sqrt(2.0)*ScaledDistance(_c1, _c2, 0.5);
  return dist;
}

double
EuclideanDistance::
ScaledDistance(const Cfg& _c1, const Cfg& _c2, double sValue) {
  Cfg *pTmp = _c1.CreateNewCfg();
  pTmp->subtract(_c1,_c2);
  double dReturn = sqrt(  sValue*sqr(pTmp->PositionMagnitude()) + 
                          (1.0 - sValue)*sqr(pTmp->OrientationMagnitude()) );
  delete pTmp;
  return dReturn;
}

//////////

ScaledEuclideanDistance::
ScaledEuclideanDistance() : EuclideanDistance() {
  sValue = 0.5;
}


ScaledEuclideanDistance::
ScaledEuclideanDistance(double _sValue) : EuclideanDistance() {
  if((_sValue < 0) || (_sValue > 1)) {
    cout << "\n\nERROR: sValue " << _sValue << " invalid, must be between 0 and 1\n";
    exit(-1);
  }
  sValue = _sValue;
}


ScaledEuclideanDistance::
~ScaledEuclideanDistance() {
}


char* 
ScaledEuclideanDistance::
GetName() const {
  return "scaledEuclidean";
}


void 
ScaledEuclideanDistance::
SetDefault() {
  sValue = 0.5;
}


bool
ScaledEuclideanDistance::
operator==(const ScaledEuclideanDistance& dm) const {
  if( strcmp(GetName(), dm.GetName()) ) {
    return false;
  } else {
    return ((sValue-dm.GetS() < 0.000000001) && 
            (sValue-dm.GetS() > -0.000000001));
  }
}


void 
ScaledEuclideanDistance::
ParseCommandLine(int argc, char** argv) {
  if(argc > 2) {
    cerr << "\nERROR ParseCommandLine: Don\'t understand \""
	 << argv << "\"\n\n";
    PrintUsage(cerr);
    cerr << endl;
    exit(-1);
  }

  if(argc == 2) { //read in sValue
    istrstream is(argv[1]);
    if(!(is >> sValue)) {
      cerr << "\nERROR ParseCommandLine: Don\'t understand\""
	   << argv << "\"\n\n";
      PrintUsage(cerr);
      cerr << endl;
      exit(-1);
    }
    if((sValue < 0) || (sValue > 1)) {
      cerr << "\nERROR invalid sValue " << sValue << ", must be between 0 and 1\n\n";
      exit(-1);
    }
  }
}


void 
ScaledEuclideanDistance::
PrintUsage(ostream& _os) const {
  _os.setf(ios::left,ios::adjustfield);

  _os << "\n" << GetName() << " ";
  _os << "FLOAT (default,0.5)";

  _os.setf(ios::right,ios::adjustfield);
}


void 
ScaledEuclideanDistance::
PrintValues(ostream& _os) const {
  _os << "\n" << GetName() << " ";
  _os << sValue;
  _os << endl;
}


DistanceMetricMethod* 
ScaledEuclideanDistance::
CreateCopy() {
  DistanceMetricMethod* _copy = new ScaledEuclideanDistance(*this);
  return _copy;
}


double 
ScaledEuclideanDistance::
Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2) {
  double dist;
  dist = ScaledDistance(_c1, _c2, sValue); 
  return dist;
}

//////////

MinkowskiDistance::
MinkowskiDistance() : DistanceMetricMethod() {
  type = CS;
  r1 = 3;
  r2 = 3;
  r3 = 1.0/3;
}


MinkowskiDistance::
~MinkowskiDistance() {
}


char* 
MinkowskiDistance::
GetName() const {
  return "minkowski";
}


void 
MinkowskiDistance::
SetDefault() {
  r1 = 3;
  r2 = 3;
  r3 = 0.333;
}


bool
MinkowskiDistance::
operator==(const MinkowskiDistance& dm) const {
  if( strcmp(GetName(), dm.GetName()) ) {
    return false;
  } else {
    return ( ((r1-dm.GetR1() < 0.000000001) && (r1-dm.GetR1() > -0.000000001)) &&
             ((r2-dm.GetR2() < 0.000000001) && (r2-dm.GetR2() > -0.000000001)) &&
             ((r3-dm.GetR3() < 0.000000001) && (r3-dm.GetR3() > -0.000000001)) );
  }
}


void 
MinkowskiDistance::
ParseCommandLine(int argc, char** argv) {
  if(argc > 4) {
    cerr << "\nERROR ParseCommandLine: Don\'t understand \""
	 << argv << "\"\n\n";
    PrintUsage(cerr);
    cerr << endl;
    exit(-1);
  }

  if(argc > 1) { //read in r1
    istrstream is1(argv[1]);
    if(!(is1 >> r1)) {
      cerr << "\nERROR ParseCommandLine: Don\'t understand\""
	   << argv << "\"\n\n";
      PrintUsage(cerr);
      cerr << endl;
      exit(-1);
    }
    r2 = r3 = r1;
  }

  if(argc > 2) { //read in r2 and r3
    if(argc != 4) {
      cerr << "\nERROR ParseCommandLine: If you specify 2 values, you must specify a 3rd.\n\n";
      exit(-1);
    }

    istrstream is2(argv[2]);
    if(!(is2 >> r2)) {
      cerr << "\nERROR ParseCommandLine: Don\'t understand\""
	   << argv << "\"\n\n";
      PrintUsage(cerr);
      cerr << endl;
      exit(-1);
    }

    istrstream is3(argv[3]); 
    if(!(is3 >> r3)) {
      cerr << "\nERROR ParseCommandLine: Don\'t understand\""
	   << argv << "\"\n\n";
      PrintUsage(cerr);
      cerr << endl;
      exit(-1);
    }
  }
}


void 
MinkowskiDistance::
PrintUsage(ostream& _os) const {
  _os.setf(ios::left,ios::adjustfield);

  _os << "\n" << GetName() << " ";
  _os << "FLOAT FLOAT FLOAT (default 3 3 0.3333)";

  _os.setf(ios::right,ios::adjustfield);
}

 
void 
MinkowskiDistance::
PrintValues(ostream& _os) const {
  _os << "\n" << GetName() << " ";
  _os << r1 << " " << r2 << " " << r3;
  _os << endl;
}


DistanceMetricMethod* 
MinkowskiDistance::
CreateCopy() {
  DistanceMetricMethod* _copy = new MinkowskiDistance(*this);
  return _copy;
}


double 
MinkowskiDistance::
Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2) {
  double dist,pos=0,orient=0/*,d*/;

  Cfg *pC = _c1.CreateNewCfg();
  pC->subtract(_c1,_c2);
  
  vector<double> p = pC->GetPosition(); // position values
  vector<double> o = pC->GetOrientation(); //orientation values
              
  int i;
  for(i=0; i<p.size(); i++) {
    if(p[i] < 0) 
      p[i] = -p[i];
    pos += pow(p[i], r1);
  }
  
  for(i=0;i<o.size();i++) {
    orient += pow(o[i], r2);
  }
  
  dist = pow(pos+orient, r3);
  delete pC;
  
  return dist;
}

//////////

ManhattanDistance::
ManhattanDistance() : DistanceMetricMethod() {
  type = CS;
}


ManhattanDistance::
~ManhattanDistance() {
}


char* 
ManhattanDistance::
GetName() const {
  return "manhattan";
}


void 
ManhattanDistance::
SetDefault() {
}


void 
ManhattanDistance::
ParseCommandLine(int argc, char** argv) {
  if(argc > 1) {
    cerr << "\nERROR ParseCommandLine: Don\'t understand \""
	 << argv << "\"\n\n";
    PrintUsage(cerr);
    cerr << endl;
    exit(-1);
  }
}


void 
ManhattanDistance::
PrintUsage(ostream& _os) const {
  _os.setf(ios::left,ios::adjustfield);

  _os << "\n" << GetName() << " ";

  _os.setf(ios::right,ios::adjustfield);
}


void 
ManhattanDistance::
PrintValues(ostream& _os) const {
  _os << "\n" << GetName() << " ";
  _os << endl;
}


DistanceMetricMethod* 
ManhattanDistance::
CreateCopy() {
  DistanceMetricMethod* _copy = new ManhattanDistance(*this);
  return _copy;
}


double 
ManhattanDistance::
Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2) {
  double dist = 0;
  
  Cfg *pC = _c1.CreateNewCfg();
  
  vector<double> dt = pC->GetData(); // position values
               
  for(int i=0; i < dt.size(); i++) {
    if(dt[i] < 0) 
      dist = dist-dt[i];
    else
      dist += dt[i];
  }
  
  delete pC;
  return dist;
}

//////////

CenterOfMassDistance::  
CenterOfMassDistance() : DistanceMetricMethod() {
  type = WS;
}


CenterOfMassDistance::
~CenterOfMassDistance() {
}


char* 
CenterOfMassDistance::
GetName() const {
  return "com";
}


void 
CenterOfMassDistance::
SetDefault() {
}


void 
CenterOfMassDistance::
ParseCommandLine(int argc, char** argv) {
  if(argc > 1) {
    cerr << "\nERROR ParseCommandLine: Don\'t understand \""
	 << argv << "\"\n\n";
    PrintUsage(cerr);
    cerr << endl;
    exit(-1);
  }
}


void 
CenterOfMassDistance::
PrintUsage(ostream& _os) const {
  _os.setf(ios::left,ios::adjustfield);

  _os << "\n" << GetName() << " ";

  _os.setf(ios::right,ios::adjustfield);
}


void 
CenterOfMassDistance::
PrintValues(ostream& _os) const {
  _os << "\n" << GetName() << " ";
  _os << endl;
}


DistanceMetricMethod* 
CenterOfMassDistance::
CreateCopy() {
  DistanceMetricMethod* _copy = new CenterOfMassDistance(*this);
  return _copy;
}


double 
CenterOfMassDistance::
Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2) {
  Vector3D d = _c1.GetRobotCenterPosition()-_c2.GetRobotCenterPosition();
  return d.magnitude();
}
