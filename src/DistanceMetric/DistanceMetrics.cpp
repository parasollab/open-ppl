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
#include "LocalPlanners.h"
#include "MPProblem.h"


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

  RmsdDistance* rmsd = new RmsdDistance();
  all.push_back(rmsd);

  LPSweptDistance* lp_swept = new LPSweptDistance();
  all.push_back(lp_swept);

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
    } else if(citr->getName() == "rmsd") {
      RmsdDistance* rmsd = new RmsdDistance();
      all.push_back(rmsd);
      selected.push_back(rmsd->CreateCopy());
      citr->warnUnrequestedAttributes();
    } else if(citr->getName() == "lp_swept") {
      double pos_res = citr->numberXMLParameter("pos_res", false, in_pProblem->GetEnvironment()->GetPositionRes(), 0.0, 1000.0, "position resolution");
      double ori_res = citr->numberXMLParameter("ori_res", false, in_pProblem->GetEnvironment()->GetOrientationRes(), 0.0, 1000.0, "orientation resolution");
      bool use_bbox = citr->boolXMLParameter("use_bbox", false, false, "use bbox instead of robot vertices"); 

      LocalPlanners<CfgType, WeightType>* lp;
      for(XMLNodeReader::childiterator citr2 = citr->children_begin(); citr2 != citr->children_end(); ++citr2)
        if(citr2->getName() == "lp_methods")
          lp = new LocalPlanners<CfgType, WeightType>(*citr2, in_pProblem);
      if(lp->selected.size() != 1)
      {
        cout << "\n\nError in reading local planner method for rmsdLPDistance, there should only be 1 method selected\n\n";
        exit(-1);
      }
      LPSweptDistance* lp_swept = new LPSweptDistance(lp->selected[0]->CreateCopy(), pos_res, ori_res, use_bbox);
      all.push_back(lp_swept);
      selected.push_back(lp_swept->CreateCopy());
      cout << "LPSweptDistance: lp_method = " << lp->selected[0]->GetName() << ", pos_res = " << pos_res << ", ori_res = " << ori_res << ", use_bbox = " << use_bbox << endl;
      //delete lp;
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

RmsdDistance::
RmsdDistance() : EuclideanDistance() {
}

RmsdDistance::
~RmsdDistance() {
}

char*
RmsdDistance::
GetName() const {
  return "rmsd";
}

DistanceMetricMethod*
RmsdDistance::
CreateCopy() {
  DistanceMetricMethod* _copy = new RmsdDistance(*this);
  return _copy;
}

vector<Vector3D>
RmsdDistance::
GetCoordinatesForRMSD(const Cfg& c, Environment* env) {
  c.ConfigEnvironment(env);
  MultiBody *robot = env->GetMultiBody(env->GetRobotIndex());
  vector<Vector3D> coordinates;
  for(int i=0 ; i<robot->GetFreeBodyCount(); i ++)
    coordinates.push_back(robot->GetFreeBody(i)->WorldTransformation().position); 
  return coordinates;
}


double
RmsdDistance::
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
  vector<Vector3D> x = GetCoordinatesForRMSD(_c1, env);
  vector<Vector3D> y = GetCoordinatesForRMSD(_c2, env);
  return RMSD(x,y,x.size());
}

double
RmsdDistance::
RMSD(vector<Vector3D> x, vector<Vector3D> y, int dim) {
  if(x.size() < dim || y.size() < dim || dim <= 0) {
    cout << "Error in MyDistanceMetrics::RMSD, not enough data in vectors"
         << endl;
    exit(101);
  }
  int n;
  Vector3D sumx(0,0,0), sumy(0,0,0);
  for(n=0; n<dim; ++n) {
    sumx = sumx + x[n];
    sumy = sumy + y[n];
  }
  for(n=0; n<dim; ++n) {
    x[n] = x[n] - sumx/dim;
    y[n] = y[n] - sumy/dim;
  }

  // now calc. E0 = 1/2*sum_of[xn^2 + yn^2]
  double E0 = 0.0;
  double R[3][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  for(n=0; n<dim; ++n) {
    E0 += x[n].normsqr() + y[n].normsqr();
    for(int i=0; i<3; ++i) {
      for(int j=0; j<3; ++j)
        R[i][j] += y[n][i]*x[n][j]; // *weight[n] if needed.
    }
  }
  E0 /= 2;
  //let matrix R~*R = { a[0]   d    e
  //                     d    a[1]  f
  //                     e     f    a[2] };
  //Now, decide this parameters.
  //using a matrix here would be clearer, simply S = R.transpose()*R;
  Vector3D col[3];
  double a[3], d, e, f;
  double detR; // determint of R, we need its sign later.
  for(int i=0; i<3; ++i) {
    col[i] = Vector3D(R[0][i], R[1][i], R[2][i]);
    a[i] = col[i].normsqr();
  }
  d = col[0].dotProduct(col[1]);
  e = col[0].dotProduct(col[2]);
  f = col[1].dotProduct(col[2]);
  Vector3D col1X2 = col[1].crossProduct(col[2]);
  detR = col[0].dotProduct(col1X2);

  // now solve for the eigenvalues of the matrix, since we
  // know we have three non-negative eigenvalue, we can directly
  // solve a cubic equation for the roots...
  // the equation looks like: z^3 + a2*z^2 + a1*z + a0 = 0
  // in our case,
  double a2 = -(a[0] + a[1] + a[2]);
  double a1 = a[0]*a[1] + a[1]*a[2] + a[2]*a[0] - d*d - e*e - f*f;
  double a0 = a[0]*f*f + a[1]*e*e + a[2]*d*d - a[0]*a[1]*a[2] - 2*d*e*f;

  // reference for cubic equation solution:
  // http://mathworld.wolfram.com/CubicEquation.html
  // following the symbols using there, define:
  double Q = (3*a1 - a2*a2) / 9;
  double RR = (9.0*a2*a1 - 27.0*a0 - 2*a2*a2*a2) / 54;

  // if our case, since we know there are three real roots,
  // so D = Q^3 + R^2 <= 0,
  double z1, z2, z3;
  if(Q == 0) { // which means three identical roots,
    z1 = z2 = z3 = -a2/3;
  } else { // Q < 0
    double rootmq = sqrt(-Q);
    double ceta = acos(-RR/Q/rootmq);
    double cc3 = cos(ceta/3);      // = cos(ceta/3)
    double sc3 = sqrt(1-cc3*cc3);  // = sin(ceta/3)
    z1 = 2*rootmq*cc3 - a2/3;
    z2 = rootmq*(-cc3 + sc3*1.7320508) - a2/3;
    z3 = rootmq*(-cc3 - sc3*1.7320508) - a2/3;
  }
  if(z3 < 0) // small numercal error
    z3 = 0;

  int sign = detR > 0 ? 1 : -1;
  double E = E0 - sqrt(z1) - sqrt(z2) - sqrt(z3)*sign;
  if(E<0) // small numercal error
    return 0;

  // since E = 1/2 * sum_of[(Uxn - yn)^2], so rmsd is:
  double rmsd = sqrt(E*2/dim);

  return rmsd;
}


//////////

LPSweptDistance::
LPSweptDistance() : DistanceMetricMethod() {
}

LPSweptDistance::
LPSweptDistance(LocalPlannerMethod<CfgType, WeightType>* _lp_method) : DistanceMetricMethod(), lp_method(_lp_method), positionRes(0.1), orientationRes(0.1), use_bbox(false) {
}

LPSweptDistance::
LPSweptDistance(LocalPlannerMethod<CfgType, WeightType>* _lp_method, double pos_res, double ori_res, bool bbox) : DistanceMetricMethod(), lp_method(_lp_method), positionRes(pos_res), orientationRes(ori_res), use_bbox(bbox) {
}

LPSweptDistance::
~LPSweptDistance() {
}

char*
LPSweptDistance::
GetName() const {
  return "lp_swept";
}

void 
LPSweptDistance::
SetDefault() {
}

DistanceMetricMethod*
LPSweptDistance::
CreateCopy() {
  DistanceMetricMethod* _copy = new LPSweptDistance(*this);
  return _copy;
}

double
LPSweptDistance::
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
  Stat_Class Stats;
  CollisionDetection cd;
  DistanceMetric dm;
  LPOutput<CfgType, WeightType> lpOutput;
  if(lp_method == NULL)
  {
    cerr << "\n\nAttempting to call LPSweptDistance::Distance() without setting the appropriate LP method\n\n";
    exit(-1);
  }
  lp_method->IsConnected(env, Stats, &dm, _c1, _c2, &lpOutput, positionRes, orientationRes, false, true);
  //vector<CfgType> cfgs = lpOutput.path;
  //lpPath does not include _c1 and _c2, so adding them manually
  vector<CfgType> cfgs(1, _c1);
  cfgs.insert(cfgs.end(), lpOutput.path.begin(), lpOutput.path.end());
  cfgs.push_back(_c2);

  double d = 0;
  vector<GMSPolyhedron> poly2;
  int robot = env->GetRobotIndex();
  int body_count = env->GetMultiBody(robot)->GetFreeBodyCount();
  cfgs.begin()->ConfigEnvironment(env);
  for(int b=0; b<body_count; ++b)
    if(use_bbox)
      poly2.push_back(env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldBoundingBox());
    else
      poly2.push_back(env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldPolyhedron());
  for(vector<CfgType>::const_iterator C = cfgs.begin(); C+1 != cfgs.end(); ++C)
  {
    vector<GMSPolyhedron> poly1(poly2);
    poly2.clear();
    (C+1)->ConfigEnvironment(env);
    for(int b=0; b<body_count; ++b)
      if(use_bbox)
        poly2.push_back(env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldBoundingBox());
      else
        poly2.push_back(env->GetMultiBody(robot)->GetFreeBody(b)->GetWorldPolyhedron());
    double _d = SweptDistance(env, poly1, poly2);
    d += SweptDistance(env, poly1, poly2);
  }
  return d;
}

double
LPSweptDistance::
SweptDistance(Environment* env, const vector<GMSPolyhedron>& poly1, const vector<GMSPolyhedron>& poly2) {
  double d = 0;
  int count = 0;
  for(int b=0; b<poly1.size(); ++b)
    for(int i=0; i<poly1[b].numVertices; ++i)
    {
      d += (poly1[b].vertexList[i] - poly2[b].vertexList[i]).magnitude();
      count++;
    }
  return d/(double)count;
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
