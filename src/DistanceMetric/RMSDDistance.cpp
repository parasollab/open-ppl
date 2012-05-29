#include "RMSDDistance.h"


RMSDDistance::RMSDDistance() : DistanceMetricMethod() {
  m_name = "rmsd";
}

RMSDDistance::
RMSDDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : DistanceMetricMethod(_node, _problem, _warn) {
  m_name = "rmsd";
}

RMSDDistance::~RMSDDistance() {
}

vector<Vector3D> RMSDDistance::GetCoordinatesForRMSD(const Cfg& _c, Environment* _env) {
  _c.ConfigEnvironment(_env);
  vector<Vector3D> coordinates;
  for(int i=0; i< _env->GetMultiBody(_env->GetRobotIndex())->GetFreeBodyCount(); ++i)
    coordinates.push_back(_env->GetMultiBody(_env->GetRobotIndex())->GetFreeBody(i)->WorldTransformation().position); 
  return coordinates;
}

double RMSDDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  vector<Vector3D> x = GetCoordinatesForRMSD(_c1, _env);
  vector<Vector3D> y = GetCoordinatesForRMSD(_c2, _env);
  return RMSD(x,y,x.size());
}

double RMSDDistance::RMSD(vector<Vector3D> _x, vector<Vector3D> _y, int _dim) {
  if((int)_x.size() < _dim || (int)_y.size() < _dim || _dim <= 0) {
    cout << "Error in MyDistanceMetrics::RMSD, not enough data in vectors" << endl;
    exit(101);
  }
  
  //rmsd = sqrt( sum_of[(U*xn - yn)^2]/N )
  //where U is the rotation that minimizes rmsd
  //reference: B.Kabsch '78. Acta Cryst. (1978) A34 page 827-828

  //first step, remove any translation between x and y.
  int n;
  Vector3D sumx(0,0,0), sumy(0,0,0);
  for(n=0; n<_dim; ++n) {
    sumx = sumx + _x[n];
    sumy = sumy + _y[n];
  }
  for(n=0; n<_dim; ++n) {
    _x[n] = _x[n] - sumx/_dim;
    _y[n] = _y[n] - sumy/_dim;
  }

  // now calc. e0 = 1/2*sum_of[xn^2 + yn^2]
  double e0 = 0.0;
  double r[3][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  for(n=0; n<_dim; ++n) {
    e0 += _x[n].normsqr() + _y[n].normsqr();
    for(int i=0; i<3; ++i) {
      for(int j=0; j<3; ++j)
        r[i][j] += _y[n][i]*_x[n][j]; // *weight[n] if needed.
    }
  }
  e0 /= 2;

  //let matrix r~*r = { a[0]   d    e
  //                     d    a[1]  f
  //                     e     f    a[2] };
  //Now, decide this parameters.
  //using a matrix here would be clearer, simply s = r.transpose()*R;
  Vector3D col[3];
  double a[3], d, e, f;
  double detR; // determinant of r, we need its sign later.
  for(int i=0; i<3; ++i) {
    col[i] = Vector3D(r[0][i], r[1][i], r[2][i]);
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
  double q = (3*a1 - a2*a2) / 9;
  double rr = (9.0*a2*a1 - 27.0*a0 - 2*a2*a2*a2) / 54;

  // if our case, since we know there are three real roots,
  // so D = q^3 + r^2 <= 0,
  double z1, z2, z3;
  if(q == 0) { // which means three identical roots,
    z1 = z2 = z3 = -a2/3;
  } else { // q < 0
    double rootmq = sqrt(-q);
    double ceta = acos(-rr/q/rootmq);
    double cc3 = cos(ceta/3);      // = cos(ceta/3)
    double sc3 = sqrt(1-cc3*cc3);  // = sin(ceta/3)
    z1 = 2*rootmq*cc3 - a2/3;
    z2 = rootmq*(-cc3 + sc3*1.7320508) - a2/3;
    z3 = rootmq*(-cc3 - sc3*1.7320508) - a2/3;
  }
  if(z3 < 0) // small numercal error
    z3 = 0;

  int sign = detR > 0 ? 1 : -1;
  double ee = e0 - sqrt(z1) - sqrt(z2) - sqrt(z3)*sign;
  if(ee<0) // small numercal error
    return 0;

  // since ee = 1/2 * sum_of[(Uxn - yn)^2], so rmsd is:
  double rmsd = sqrt(ee*2/_dim);

  return rmsd;
}
