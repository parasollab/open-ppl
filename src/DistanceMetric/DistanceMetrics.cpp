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

#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
  ReachableDistance* rd = new ReachableDistance();
  all.push_back(rd);
#endif

  m_distance_time = 0;
}


DistanceMetric::
DistanceMetric(XMLNodeReader& in_Node, MPProblem* in_pProblem) : 
    MPBaseObject(in_Node, in_pProblem){
  LOG_DEBUG_MSG("DistanceMetric::DistanceMetric()");
  /*
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
  */

  in_Node.verifyName("distance_metrics");

  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "euclidean") {
      EuclideanDistance* euclidean = new EuclideanDistance();
      all.push_back(euclidean);
      selected.push_back(euclidean->CreateCopy());
      citr->warnUnrequestedAttributes();
    } else if(citr->getName() == "scaledEuclidean") {
      double par_scale;
      ScaledEuclideanDistance* scaledEuclidean;
      par_scale = citr->numberXMLParameter("scale",false,double(0.5),
                                          double(0.0),double(1.0),
                                          "Scale Factor");
      scaledEuclidean = new ScaledEuclideanDistance(par_scale);
      all.push_back(scaledEuclidean);
      selected.push_back(scaledEuclidean->CreateCopy());
      citr->warnUnrequestedAttributes();
    } else {
      citr->warnUnknownNode();
    }
  }

  m_distance_time = 0;
  LOG_DEBUG_MSG("~DistanceMetric::DistanceMetric()");
}


DistanceMetric::
~DistanceMetric() {
  vector<DistanceMetricMethod*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    delete *I;
  selected.clear();

  for(I=all.begin(); I!=all.end(); I++)
    delete *I;
  all.clear();
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
PrintOptions(ostream& out_os) const { 
  out_os << "  Distance Metrics" << endl;
  vector<DistanceMetricMethod*>::const_iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    (*I)->PrintOptions(out_os);
}

/**
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
*/
/*
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
    std::istringstream _dmstream(dmdesc);
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
*/
/*
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
  _myostream << "#####DMSTOP#####"; 
}
*/

double 
DistanceMetric::
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
  //MultiBody* robot;
  //robot = env->GetMultiBody(env->GetRobotIndex());
  return selected[0]->Distance(env, _c1, _c2);
}


double 
DistanceMetric::
Distance(Environment* env, const Cfg* _c1, const Cfg* _c2) {
  //MultiBody* robot;
  //robot = env->GetMultiBody(env->GetRobotIndex());
  return selected[0]->Distance(env, *_c1, *_c2);
}

void
DistanceMetric::
ScaleCfg(Environment*env, double length, Cfg& o, Cfg& c) {
  return selected[0]->ScaleCfg(env, length, o, c);
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


void 
DistanceMetricMethod::
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
DistanceMetricMethod::
PrintUsage(ostream& _os) const {
  _os.setf(ios::left,ios::adjustfield);

  _os << "\n" << GetName() << " ";

  _os.setf(ios::right,ios::adjustfield);
}


void 
DistanceMetricMethod::
PrintValues(ostream& _os) const {
  _os << "\n" << GetName() << " ";
  _os << endl;
}


void 
DistanceMetricMethod::
PrintOptions(ostream& _os) const {
  _os << "    " << GetName() << " ";
  _os << endl;
}

void
DistanceMetricMethod::
ScaleCfg(Environment* env, double length, Cfg& o, Cfg& c) {
  length = abs((int)length); //a distance must be positive

  Cfg* origin = &o;//o.CreateNewCfg();
  Cfg* outsideCfg = c.CreateNewCfg();

  // first find an outsite configuration with sufficient size
  while(Distance(env, *origin, *outsideCfg) < 2*length)
    for(int i=0; i<outsideCfg->DOF(); ++i)
      outsideCfg->SetSingleParam(i, 2*outsideCfg->GetSingleParam(i));
  
  // now, using binary search  find a configuration with the approximate
  // length
  Cfg* aboveCfg = outsideCfg->CreateNewCfg();
  Cfg* belowCfg = origin->CreateNewCfg();
  Cfg* currentCfg = c.CreateNewCfg();
  while (1) {
    for(int i=0; i<currentCfg->DOF(); ++i)
      currentCfg->SetSingleParam(i, (aboveCfg->GetSingleParam(i) + 
				     belowCfg->GetSingleParam(i)) / 2);
    double magnitude = Distance(env, *origin, *currentCfg);
    if( (magnitude >= length*0.9) && (magnitude <= length*1.1)) 
      break;
    if(magnitude>length) 
      aboveCfg->equals(*currentCfg);
    else 
      belowCfg->equals(*currentCfg); 
  }
  for(int i=0; i<c.DOF(); ++i)
    c.SetSingleParam(i, currentCfg->GetSingleParam(i));

  //delete origin;
  delete outsideCfg;
  delete aboveCfg;
  delete belowCfg;
  delete currentCfg;
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


DistanceMetricMethod* 
EuclideanDistance::
CreateCopy() {
  DistanceMetricMethod* _copy = new EuclideanDistance(*this);
  return _copy;
}


double 
EuclideanDistance::
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
  double dist;
  dist = sqrt(2.0)*ScaledDistance(env,_c1, _c2, 0.5);
  return dist;
}

double
EuclideanDistance::
ScaledDistance(Environment* env,const Cfg& _c1, const Cfg& _c2, double sValue) {
  Cfg *pTmp = _c1.CreateNewCfg();
  pTmp->subtract(_c1,_c2);
  //vector<double> normalized_vec;
  double pos_mag(0.0);
  double max_range(0.0);
  for(int i=0; i< pTmp->posDOF(); ++i) {
    std::pair<double,double> range = env->GetBoundingBox()->GetRange(i);
    double tmp_range = range.second-range.first;
    if(tmp_range > max_range) max_range = tmp_range;
  }
  //cout << "Distance Normalization" << endl;
  //cout << *pTmp << endl;
  //cout << "Max range = " << max_range << endl;

  for(int i=0; i< pTmp->posDOF(); ++i) {
    //std::pair<double,double> range = env->GetBoundingBox()->GetRange(i);
    //normalized_vec.push_back(pTmp->GetSingleParam(i) / (range.second - range.first));
     pos_mag += sqr(pTmp->GetSingleParam(i) / max_range);
    //pos_mag += sqr(pTmp->GetSingleParam(i)); // removed normalization 
  }
  //pos_mag = sqrt(pos_mag);
  //cout << "Normalized pos distance = " << sqrt(pos_mag) << endl;
  /*double dReturn = sqrt(  sValue*sqr(pTmp->PositionMagnitude()) + 
                          (1.0 - sValue)*sqr(pTmp->OrientationMagnitude()) );*/

  double dReturn = sqrt(  sValue*pos_mag + 
                          (1.0 - sValue)*sqr(pTmp->OrientationMagnitude()) );
  delete pTmp;
  return dReturn;
}

void
EuclideanDistance::
ScaleCfg(Environment* env, double length, Cfg& o, Cfg& c) {
  double original_length = this->Distance(env, o, c);
  double diff;
  do {
    for(int i=0; i<c.DOF(); ++i)
      c.SetSingleParam(i, (length/original_length)*c.GetSingleParam(i));
    original_length = this->Distance(env, o, c);
    diff = length - original_length;
  } while((diff > 0.1) || (diff < -0.1)); 
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
    cerr << "\nERROR ParseCommandLine: Don\'t understand \"";
    for(int i=0; i<argc; i++)
      cerr << argv[i] << " ";
    cerr << "\"\n\n";
    PrintUsage(cerr);
    cerr << endl;
    exit(-1);
  }

  if(argc == 2) { //read in sValue
    std::istringstream is(argv[1]);
    if(!(is >> sValue)) {
      cerr << "\nERROR ParseCommandLine: Don\'t understand\"";
      for(int i=0; i<argc; i++)
  cerr << argv[i] << " ";
      cerr << "\"\n\n";
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

void 
ScaledEuclideanDistance::
PrintOptions(ostream& _os) const {
  _os << "    " << GetName() << "::  ";
  _os << "scale = " << sValue;
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
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
  double dist;
  dist = ScaledDistance(env, _c1, _c2, sValue); 
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
    cerr << "\nERROR ParseCommandLine: Don\'t understand \"";
    for(int i=0; i<argc; i++)
      cerr << argv[i] << " ";
    cerr << "\"\n\n";
    PrintUsage(cerr);
    cerr << endl;
    exit(-1);
  }

  if(argc > 1) { //read in r1
    std::istringstream is1(argv[1]);
    if(!(is1 >> r1)) {
      cerr << "\nERROR ParseCommandLine: Don\'t understand\"";
      for(int i=0; i<argc; i++)
  cerr << argv[i] << " ";
      cerr << "\"\n\n";
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

    std::istringstream is2(argv[2]);
    if(!(is2 >> r2)) {
      cerr << "\nERROR ParseCommandLine: Don\'t understand\"";
      for(int i=0; i<argc; i++)
  cerr << argv[i] << " ";
      cerr << "\"\n\n";
      PrintUsage(cerr);
      cerr << endl;
      exit(-1);
    }

    std::istringstream is3(argv[3]); 
    if(!(is3 >> r3)) {
      cerr << "\nERROR ParseCommandLine: Don\'t understand\"";
      for(int i=0; i<argc; i++)
  cerr << argv[i] << " ";
      cerr << "\"\n\n";
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

void 
MinkowskiDistance::
PrintOptions(ostream& _os) const {
  _os << "    " << GetName() << ":: ";
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
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
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


DistanceMetricMethod* 
ManhattanDistance::
CreateCopy() {
  DistanceMetricMethod* _copy = new ManhattanDistance(*this);
  return _copy;
}


double 
ManhattanDistance::
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
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


DistanceMetricMethod* 
CenterOfMassDistance::
CreateCopy() {
  DistanceMetricMethod* _copy = new CenterOfMassDistance(*this);
  return _copy;
}


double 
CenterOfMassDistance::
Distance(const Cfg& _c1, const Cfg& _c2) {
  Vector3D d = _c1.GetRobotCenterPosition()-_c2.GetRobotCenterPosition();
  return d.magnitude();
}


//////////


#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
#include "Cfg_reach_cc.h"

double 
ReachableDistance::
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
  /*
  cout << "Computing Distance between\n";
  ((Cfg_reach_cc&)_c1).print(cout); cout << endl;
  ((Cfg_reach_cc&)_c2).print(cout); cout << endl;
  */

  //later make input param
  double s1 = 0.33;
  double s2 = 0.33;

  //get the position difference
  double d_position = 0.0;
  vector<double> v1 = _c1.GetData();
  vector<double> v2 = _c2.GetData();
  for(int i=0; i<3; ++i) {
    pair<double,double> range = env->GetBoundingBox()->GetRange(i);
    d_position += sqr(fabs(v1[i] - v2[i])/(range.second-range.first));
  }
  d_position = sqrt(d_position);
  //cout << "d_position = " << d_position << endl;

  //get the length difference
  double d_length = ((Cfg_reach_cc&)_c1).LengthDistance(_c2);
  //cout << "d_length = " << d_length << endl;

  //get the orientation difference
  double d_ori = ((Cfg_reach_cc&)_c1).OrientationDistance(_c2);
  //cout << "d_ori = " << d_ori << endl;

  return (s1*d_position + s2*d_length + (1-s1-s2)*d_ori);
}
#endif
