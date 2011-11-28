#include "DistanceMetricMethod.h"
#include "Cfg.h"
#include "Environment.h"
#include "MetricUtils.h"
#include "MPProblem.h"
#include "CollisionDetection.h"
#include "LocalPlanners.h"

DistanceMetricMethod::
DistanceMetricMethod() {}


DistanceMetricMethod::
DistanceMetricMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn) : MPBaseObject(in_Node, in_pProblem) {
  if(warn)
    in_Node.warnUnrequestedAttributes();
}


DistanceMetricMethod::
~DistanceMetricMethod() {
}


bool
DistanceMetricMethod::
operator==(const DistanceMetricMethod& dm) const {
  return GetName() == dm.GetName();
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
  name = "euclidean";
}


EuclideanDistance::
EuclideanDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn) : DistanceMetricMethod(in_Node, in_pProblem, warn)
{
  type = CS;
  name = "euclidean";
}


EuclideanDistance::
~EuclideanDistance() {
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


void
EuclideanDistance::
PrintOptions(ostream& os) const
{
  os << "    " << GetName() << "::  " << endl;
}

double 
EuclideanDistance::
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
  double dist(0.0);
  dist = sqrt(2.0)*ScaledDistance(env,_c1, _c2, 0.5);
  //HACK to test same Euclidean distance as CGAL ... REMOVE BEFORE COMMITING!!!!!
  //for(int i=0; i<_c1.DOF(); ++i) {
  //   double diff = _c1.GetSingleParam(i) - _c2.GetSingleParam(i);
  //   dist += diff * diff;
  //}
  //dist = sqrt(dist);
  return dist;
}


#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
double
EuclideanDistance::
ScaledDistance(Environment* env, const Cfg& _c1, const Cfg& _c2, double sValue) {
  Cfg_free_tree _c1_linkage(_c1.GetData());
  Cfg_free_tree _c2_linkage(_c2.GetData());
  return _ScaledDistance(env, _c1_linkage, _c2_linkage, sValue);
}
#else
double
EuclideanDistance::
ScaledDistance(Environment* env, const Cfg& _c1, const Cfg& _c2, double sValue) {
  return _ScaledDistance(env, _c1, _c2, sValue);
}
#endif


double
EuclideanDistance::
_ScaledDistance(Environment* env,const Cfg& _c1, const Cfg& _c2, double sValue) {
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
ScaledEuclideanDistance() : EuclideanDistance(), sValue(0.5)
{
  name = "scaledEuclidean";
}


ScaledEuclideanDistance::
ScaledEuclideanDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn) : EuclideanDistance(in_Node, in_pProblem, false)
{
  name = "scaledEuclidean";
  sValue = in_Node.numberXMLParameter("scale", false, 0.5, 0.0, 1.0, "Scale Factor");
  if(warn)
    in_Node.warnUnrequestedAttributes();
}


ScaledEuclideanDistance::
~ScaledEuclideanDistance() {
}


void 
ScaledEuclideanDistance::
SetDefault() {
  sValue = 0.5;
}


bool
ScaledEuclideanDistance::
operator==(const ScaledEuclideanDistance& dm) const {
  if(GetName() != dm.GetName()) {
    return false;
  } else {
    return ((sValue-dm.GetS() < 0.000000001) && 
            (sValue-dm.GetS() > -0.000000001));
  }
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

/*=============================================================
Knot Theory Dm
===============================================================*/

KnotTheoryDistance::
KnotTheoryDistance() : EuclideanDistance()
{
  name = "KnotTheory";  
}

KnotTheoryDistance::
KnotTheoryDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn): EuclideanDistance(in_Node, in_pProblem, warn){
  name = "KnotTheory";  
}

KnotTheoryDistance::
~KnotTheoryDistance(){
}

double
KnotTheoryDistance ::
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2)
{
 double dist;
 double sum =0.0, sign = 0,sign_2 =0, temp1;
   vector <Vector3D> unitVect(3000), unitVect2(3000);
  //double  dist2,writhe1,writhe2, temp2;
  dist =EuclideanDistance::Distance(env, _c1,_c2);
    Vector3D n1,n2;
 double n_2;
  vector<Vector3D> c11;
   vector<Vector3D> c21 ;
    
 

     vector<Vector3D> c1=_c1.PolyApprox(env);
     vector<Vector3D> c2=_c2.PolyApprox(env);



if (c1.empty()|| c1.size()<2)
{
  cerr << "\n\nError in KnotTheoryDistance::Distance(), c1 has too few links, exiting.\n";
  exit(-1);
}


   for(size_t i =0; i < c1.size()-1;i++){

   if((c1[i+1]-c1[i]).magnitude() != 0)
    unitVect[i] = (c1[i+1]+c1[i]);
  else
    unitVect[i] = 0;

    if((c2[i+1]-c2[i]).magnitude() !=0)
       unitVect2[i]= (c2[i+1]+c2[i]);

       else unitVect2[i]=0;

        if((unitVect[i].crossProduct(unitVect2[i])).magnitude() != 0)
    n1= (unitVect[i].crossProduct(unitVect2[i]))/
      (unitVect[i].crossProduct(unitVect2[i])).magnitude();

  else
    n1 = 0.0;
  n_2 =unitVect[i].crossProduct(unitVect2[i]).magnitude();

  if((unitVect2[i].crossProduct(unitVect[i])).magnitude() != 0)
    n2 =  (unitVect2[i].crossProduct(unitVect[i]))/
      (unitVect2[i].crossProduct(unitVect[i])).magnitude();
  else
    n2 = 0.0;

 if(n1.dotProduct(n2)>1)
     temp1 = 1.0;
  else
    temp1 = n1.dotProduct(n2);


 sign =(n1.crossProduct(n2)).dotProduct(unitVect[i]); 
  sum += n_2; 
                                       }
  
sign_2 = sign/fabs(sign);

return  fabs((sum/(4.0))*sign_2); 
 

}




void
KnotTheoryDistance::
SetDefault() {
}

void
KnotTheoryDistance::
PrintOptions(ostream& _os) const {
  _os << "\n" << GetName() << " ";
  _os << endl;
}

DistanceMetricMethod*
KnotTheoryDistance::
CreateCopy() {
  DistanceMetricMethod* _copy = new KnotTheoryDistance(*this);
  return _copy;
}

/////////////////////////////////////////////////////////////////////
//End Knot Theory
/////////////////////////////////////////////////////////////////////

/////////

UniformEuclideanDistance::
UniformEuclideanDistance(bool _useRotational) : DistanceMetricMethod(), useRotational(_useRotational) {
  cout << "UniformEuclideanDistance::UniformEuclideanDistance() - useRotational = " << _useRotational << endl;
  name = "uniformEuclidean";
  type = CS;
}


UniformEuclideanDistance::
~UniformEuclideanDistance() {
}

void 
UniformEuclideanDistance::
PrintOptions(ostream& _os) const {
  _os << GetName() << ":: ";
  _os << "useRotational = " << useRotational;
  _os << endl;
}

void 
UniformEuclideanDistance::
SetDefault() {
  useRotational = false;
}


DistanceMetricMethod*
UniformEuclideanDistance::
CreateCopy() {
  DistanceMetricMethod* _copy = new UniformEuclideanDistance(*this);
  return _copy;
}


double 
UniformEuclideanDistance::
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
  double dist(0.0);
  
  double max_range(0.0);
  for(int i=0; i< _c1.posDOF(); ++i) {
    std::pair<double,double> range = env->GetBoundingBox()->GetRange(i);
    double tmp_range = range.second-range.first;
    if(tmp_range > max_range) max_range = tmp_range;
  }

  //cout << "useRotational = " << useRotational << endl;
  //cout << "Calculating distance between " << _c1 << " and " << _c2 << endl;
  for(int i=0; i<_c1.DOF(); ++i) {
    
    // calculate distance for positional coordinate
    if (i < _c1.posDOF()) {
      double diff = (_c1.GetSingleParam(i) - _c2.GetSingleParam(i))/max_range;
      //cout << "  " << i + 1 << ": diff = " << diff << endl;
      if(useRotational)
        diff *= 2*PI;
      dist += diff * diff;
    }
    // calculate distance for rotational coordinate
    else {
      if (useRotational) {  // multiplying by 2*PI to make this like MPNN
        double diff1 = 2*PI*(_c1.GetSingleParam(i) - _c2.GetSingleParam(i));
        if (diff1 < 0) diff1 *= -1;
        double diff2 = 2*PI - diff1;
        double diff = (diff1 < diff2) ? diff1 : diff2;
        //cout << "  " << i + 1 << ": diff = " << diff << endl;
        dist += diff * diff;
      }
      else {  // not multiplying by 2*PI to make this like CGAL
        double diff = _c1.GetSingleParam(i) - _c2.GetSingleParam(i);
        //cout << "  " << i + 1 << ": diff = " << diff << endl;
        dist += diff * diff;
      }
    }
  }
  
  dist = sqrt(dist);
  //cout << "dist = " << dist << endl;
  return dist;
}


//////////

PureEuclideanDistance::
PureEuclideanDistance() : DistanceMetricMethod() 
{
  name = "pureEuclidean";
  type = CS;
}
 
 
PureEuclideanDistance::
~PureEuclideanDistance() {
}

void 
PureEuclideanDistance::
PrintOptions(ostream& _os) const {
  _os << GetName();
  _os << endl;
}

void 
PureEuclideanDistance::
SetDefault() {
}


DistanceMetricMethod*
PureEuclideanDistance::
CreateCopy() {
  DistanceMetricMethod* _copy = new PureEuclideanDistance(*this);
  return _copy;
}


double 
PureEuclideanDistance::
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
  double dist(0.0);
  for(int i=0; i<_c1.DOF(); ++i) {
      double diff = _c1.GetSingleParam(i) - _c2.GetSingleParam(i);
      dist += diff * diff;
    }
  dist = sqrt(dist);
  return dist;
}


//////////


MinkowskiDistance::
MinkowskiDistance() : DistanceMetricMethod() {
  name = "minkowski";
  type = CS;
  r1 = 3;
  r2 = 3;
  r3 = 1.0/3;
}


MinkowskiDistance::
MinkowskiDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn) : DistanceMetricMethod(in_Node, in_pProblem, false) {
  name = "minkowski";
  type = CS;
  r1 = in_Node.numberXMLParameter("r1", false, 3.0, 0.0, 1000.0, "r1");
  r2 = in_Node.numberXMLParameter("r2", false, 3.0, 0.0, 1000.0, "r2");
  r3 = in_Node.numberXMLParameter("r3", false, 1.0/3.0, 0.0, 1000.0, "r3");
  if(warn)
    in_Node.warnUnrequestedAttributes();
}


MinkowskiDistance::
~MinkowskiDistance() {
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
  if(GetName() != dm.GetName()) {
    return false;
  } else {
    return ( ((r1-dm.GetR1() < 0.000000001) && (r1-dm.GetR1() > -0.000000001)) &&
             ((r2-dm.GetR2() < 0.000000001) && (r2-dm.GetR2() > -0.000000001)) &&
             ((r3-dm.GetR3() < 0.000000001) && (r3-dm.GetR3() > -0.000000001)) );
  }
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
              
  for(size_t i=0; i<p.size(); i++) {
    if(p[i] < 0) 
      p[i] = -p[i];
    pos += pow(p[i], r1);
  }
  
  for(size_t i=0;i<o.size();i++) {
    orient += pow(o[i], r2);
  }
  
  dist = pow(pos+orient, r3);
  delete pC;
  
  return dist;
}
/////////////

ManhattanDistance::
ManhattanDistance() : DistanceMetricMethod() {
  name = "manhattan";
  type = CS;
}


ManhattanDistance::
ManhattanDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn) : DistanceMetricMethod(in_Node, in_pProblem, warn) {
  name = "manhattan";
  type = CS;
}


ManhattanDistance::
~ManhattanDistance() {
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


void
ManhattanDistance::
PrintOptions(ostream& os) const
{
  os << "    " << GetName() << "::  " << endl;
}


double 
ManhattanDistance::
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
  double dist = 0;
  
  Cfg *pC = _c1.CreateNewCfg();
  
  vector<double> dt = pC->GetData(); // position values
               
  for(size_t i=0; i < dt.size(); i++) {
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
CenterOfMassDistance() : DistanceMetricMethod() 
{
  name = "com";
  type = WS;
}

CenterOfMassDistance::  
CenterOfMassDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn) : DistanceMetricMethod(in_Node, in_pProblem, warn) {
  name = "com";
  type = WS;
}


CenterOfMassDistance::
~CenterOfMassDistance() {
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


void
CenterOfMassDistance::
PrintOptions(ostream& os) const
{
  os << "    " << GetName() << "::  " << endl;
}


double 
CenterOfMassDistance::
Distance(const Cfg& _c1, const Cfg& _c2) {
  Vector3D d = _c1.GetRobotCenterPosition()-_c2.GetRobotCenterPosition();
  return d.magnitude();
}

//////////

RmsdDistance::
RmsdDistance() : EuclideanDistance() 
{
  name = "rmsd";
}

RmsdDistance::
RmsdDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn) : EuclideanDistance(in_Node, in_pProblem, warn) {
  name = "rmsd";
}

RmsdDistance::
~RmsdDistance() {
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
  //MultiBody *robot = env->GetMultiBody(env->GetRobotIndex());
  vector<Vector3D> coordinates;
  for(int i=0 ; i< env->GetMultiBody(env->GetRobotIndex())->GetFreeBodyCount(); i ++)
    coordinates.push_back(env->GetMultiBody(env->GetRobotIndex())->GetFreeBody(i)->WorldTransformation().position); 
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
  if((int)x.size() < dim || (int)y.size() < dim || dim <= 0) {
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
LPSweptDistance() : DistanceMetricMethod() 
{
  name = "lp_swept";
}

LPSweptDistance::
LPSweptDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn) : DistanceMetricMethod(in_Node, in_pProblem, false) 
{
  name = "lp_swept";
  positionRes = in_Node.numberXMLParameter("pos_res", false, in_pProblem->GetEnvironment()->GetPositionRes(), 0.0, 1000.0, "position resolution");
  orientationRes = in_Node.numberXMLParameter("ori_res", false, in_pProblem->GetEnvironment()->GetOrientationRes(), 0.0, 1000.0, "orientation resolution");
  use_bbox = in_Node.boolXMLParameter("use_bbox", false, false, "use bbox instead of robot vertices");

  for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr)
    if(citr->getName() == "lp_methods") {
      LocalPlanners<CfgType, WeightType>* lp = new LocalPlanners<CfgType, WeightType>(*citr, in_pProblem);
      lp_method = lp->GetLocalPlannerMethod("dm_lp");
      
    } else {
      if(warn)
        citr->warnUnknownNode();
    }

  if(warn)
    in_Node.warnUnrequestedAttributes();
}

LPSweptDistance::
LPSweptDistance(LocalPlannerPointer _lp_method, double pos_res, double ori_res, bool bbox) : DistanceMetricMethod(), lp_method(_lp_method), positionRes(pos_res), orientationRes(ori_res), use_bbox(bbox) {
}

LPSweptDistance::
~LPSweptDistance() {
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


void
LPSweptDistance::
PrintOptions(ostream& os) const
{
  os << "    " << this->GetName() << "::  ";
  os << "\tlp_method = "; 
  lp_method->PrintOptions(os);
  os << "\tpositionRes = " << positionRes;
  os << "\torientationRes = " << orientationRes;
  os << "\tuse_bbox = " << use_bbox;
  os << endl;
}


double
LPSweptDistance::
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
  Stat_Class Stats;
  CollisionDetection cd;
  shared_ptr<DistanceMetricMethod > dm;
  LPOutput<CfgType, WeightType> lpOutput;
  if(lp_method == NULL)
  {
    cerr << "\n\nAttempting to call LPSweptDistance::Distance() without setting the appropriate LP method\n\n";
    exit(-1);
  }
  CfgType dummy;
  lp_method->IsConnected(env, Stats, dm, _c1, _c2, dummy, &lpOutput, positionRes, orientationRes, false, true);
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
    d += SweptDistance(env, poly1, poly2);
  }
  return d;
}

double
LPSweptDistance::
SweptDistance(Environment* env, const vector<GMSPolyhedron>& poly1, const vector<GMSPolyhedron>& poly2) {
  double d = 0;
  int count = 0;
  for(size_t b=0; b<poly1.size(); ++b)
    for(size_t i=0; i<poly1[b].vertexList.size(); ++i)
    {
      d += (poly1[b].vertexList[i] - poly2[b].vertexList[i]).magnitude();
      count++;
    }
  return d/(double)count;
}

//////////

BinaryLPSweptDistance::
BinaryLPSweptDistance() : DistanceMetricMethod() 
{
  name = "binary_lp_swept";
}


BinaryLPSweptDistance::
BinaryLPSweptDistance(LocalPlannerPointer _lp_method, double pos_res, double ori_res, double tolerance, int max_attempts, bool bbox) : DistanceMetricMethod(), lp_method(_lp_method), positionRes(pos_res), orientationRes(ori_res), tolerance(tolerance), max_attempts(max_attempts), dist_calls_count(0), use_bbox(bbox) {
  name = "binary_lp_swept";
}

BinaryLPSweptDistance::
BinaryLPSweptDistance(XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warn) : DistanceMetricMethod(in_Node, in_pProblem, false) 
{
  name = "binary_lp_swept";
  positionRes = in_Node.numberXMLParameter("pos_res", false, in_pProblem->GetEnvironment()->GetPositionRes() * 50, 0.0, 1000.0, "position resolution");
  orientationRes = in_Node.numberXMLParameter("ori_res", false, in_pProblem->GetEnvironment()->GetOrientationRes() * 50, 0.0, 1000.0, "orientation resolution");
  tolerance = in_Node.numberXMLParameter("tolerance", false, 0.01, 0.0, 1000.0, "tolerance");
  max_attempts = in_Node.numberXMLParameter("max_attempts", false, 10, 1, 100, "maximum depth of lp_swept distance search");
  use_bbox = in_Node.boolXMLParameter("use_bbox", false, false, "use bbox instead of robot vertices");

  for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr != in_Node.children_end(); ++citr)
    if (citr->getName() == "lp_methods") {
      LocalPlanners<CfgType, WeightType>* lp = new LocalPlanners<CfgType, WeightType>(*citr, in_pProblem);
      lp_method = lp->GetLocalPlannerMethod("dm_lp");      
    } else {
      if(warn)
        citr->warnUnknownNode();
    }

  if(warn)
    in_Node.warnUnrequestedAttributes();
}

BinaryLPSweptDistance::
~BinaryLPSweptDistance() {
}

void 
BinaryLPSweptDistance::
SetDefault() {
}

DistanceMetricMethod*
BinaryLPSweptDistance::
CreateCopy() {
  DistanceMetricMethod* _copy = new BinaryLPSweptDistance(*this);
  return _copy;
}

void
BinaryLPSweptDistance::
PrintOptions(ostream& os) const
{
  os << "    " << this->GetName() << "::  ";
  os << "\tlp_method = "; lp_method->PrintOptions(os);
  os << "\tpositionRes = " << positionRes;
  os << "\torientationRes = " << orientationRes;
  os << "\tuse_bbox = " << use_bbox;
  os << "\ttolerance = " << tolerance;
  os << "\tmax_attempts = " << max_attempts;
  os << endl;
}

double
BinaryLPSweptDistance::
Distance(Environment* env, const Cfg& _c1, const Cfg& _c2) {
  double posRes = positionRes;
  double oriRes = orientationRes;
  double old_dist = DistanceCalc(env, _c1, _c2, posRes, oriRes);
  double new_dist = 0.0;
  int match_count = 1;
//  cout << "  (" << posRes << " | " << oriRes << ") = " << old_dist << endl;
  for (int i = 1; i < max_attempts; i++) {
    posRes /= 2.0;
    oriRes /= 2.0;
    if (posRes < env->GetPositionRes())
      posRes = env->GetPositionRes();
    if (oriRes < env->GetOrientationRes())
      oriRes = env->GetOrientationRes();
      
    new_dist = DistanceCalc(env, _c1, _c2, posRes, oriRes);
//    cout << "  (" << posRes << " | " << oriRes << ") = " << new_dist << endl;
    if (new_dist - old_dist < tolerance)
      match_count++;
    else
      match_count = 1;
      
    if (match_count == 3)
      break;
    if (posRes == env->GetPositionRes() && oriRes == env->GetOrientationRes())
      break;
      
    old_dist = new_dist;
  }

  return new_dist;
}

double
BinaryLPSweptDistance::
DistanceCalc(Environment* env, const Cfg& _c1, const Cfg& _c2, double posRes, double oriRes) {
  //cout << "BinaryLPSweptDistance::DistanceCalc()" << endl;
  Stat_Class Stats;
  shared_ptr<DistanceMetricMethod > dm;
  LPOutput<CfgType, WeightType> lpOutput;
  if(lp_method == NULL)
  {
    cerr << "\n\nAttempting to call LPSweptDistance::Distance() without setting the appropriate LP method\n\n";
    exit(-1);
  }
  CfgType _col;
  lp_method->IsConnected(env, Stats, dm, _c1, _c2, _col, &lpOutput, posRes, oriRes, false, true);
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
    d += SweptDistance(env, poly1, poly2);
  }
//  cout << "\tDistance between " << _c1 << " and " << _c2 << " (" << posRes << " | " << oriRes << ") = " << d << endl;
  dist_calls_count++;
  return d;
}

double
BinaryLPSweptDistance::
SweptDistance(Environment* env, const vector<GMSPolyhedron>& poly1, const vector<GMSPolyhedron>& poly2) {
  double d = 0;
  int count = 0;
  for(size_t b=0; b<poly1.size(); ++b) {
    for(size_t i=0; i<poly1[b].vertexList.size(); ++i)
    {
      d += (poly1[b].vertexList[i] - poly2[b].vertexList[i]).magnitude();
      count++;
    }
  }
  return d/(double)count;
}

//////////


#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
#include "Cfg_reach_cc.h"

void
ReachableDistance::
PrintOptions(ostream& os) const
{
  os << "    " << GetName() << "::  " << endl;
}


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
