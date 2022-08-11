#ifndef PMPL_RMSD_DISTANCE_H_
#define PMPL_RMSD_DISTANCE_H_

#include "DistanceMetricMethod.h"

#include "MPProblem/Environment/Environment.h"


////////////////////////////////////////////////////////////////////////////////
/// @todo Properly document. This looks like a very complex way to assess RMS
///       distance between body translations.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class RMSDDistance : virtual public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    RMSDDistance();
    RMSDDistance(XMLNode& _node);
    virtual ~RMSDDistance() = default;

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2);

    // Overrides
    virtual void ScaleCfg(double _length, CfgType& _c, const CfgType& _o) override;

  private:

    ///@name Helpers
    ///@{

    vector<Vector3d> GetCoordinatesForRMSD(const CfgType& _c);
    double RMSD(vector<Vector3d> _x, vector<Vector3d> _y, int _dim);

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
RMSDDistance<MPTraits>::
RMSDDistance() : DistanceMetricMethod<MPTraits>() {
  this->SetName("RMSD");
}


template <typename MPTraits>
RMSDDistance<MPTraits>::
RMSDDistance(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node) {
  this->SetName("RMSD");
}

/*----------------------------- Distance Interface ---------------------------*/

template <typename MPTraits>
double
RMSDDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  auto x = GetCoordinatesForRMSD(_c1),
       y = GetCoordinatesForRMSD(_c2);
  return RMSD(x,y,x.size());
}

/*----------------------------- Overrides ---------------------------*/

template <typename MPTraits>
void
RMSDDistance<MPTraits>::
ScaleCfg(double _length, CfgType& _c, const CfgType& _o) {
  // TODO: this method does not exist for RMSDDistance yet
  return;
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
vector<Vector3d>
RMSDDistance<MPTraits>::
GetCoordinatesForRMSD(const CfgType& _c) {
  _c.ConfigureRobot();
  std::vector<Vector3d> coordinates;
  for(size_t i = 0; i < _c.GetMultiBody()->GetNumBodies(); ++i)
    coordinates.push_back(_c.GetMultiBody()->GetBody(i)->
        GetWorldTransformation().translation());
  return coordinates;
}


template <typename MPTraits>
double
RMSDDistance<MPTraits>::
RMSD(vector<Vector3d> _x, vector<Vector3d> _y, int _dim) {
  if((int)_x.size() < _dim || (int)_y.size() < _dim || _dim <= 0) {
    cout << "Error in MyDistanceMetrics::RMSD, not enough data in vectors"
         << endl;
    exit(-1);
  }

  //rmsd = sqrt( sum_of[(U*xn - yn)^2]/N )
  //where U is the rotation that minimizes rmsd
  //reference: B.Kabsch '78. Acta Cryst. (1978) A34 page 827-828

  //first step, remove any translation between x and y.
  int n;
  Vector3d sumx(0,0,0), sumy(0,0,0);
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
  Vector3d col[3];
  double a[3], d, e, f;
  double detR; // determinant of r, we need its sign later.
  for(int i=0; i<3; ++i) {
    col[i] = Vector3d(r[0][i], r[1][i], r[2][i]);
    a[i] = col[i].normsqr();
  }
  d = col[0]*col[1];
  e = col[0]*col[2];
  f = col[1]*col[2];
  Vector3d col1X2 = col[1] % col[2];
  detR = col[0]*col1X2;

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
  }
  else { // q < 0
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

/*----------------------------------------------------------------------------*/

#endif
