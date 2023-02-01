#ifndef ROTATIONCONVERSIONS_H_
#define ROTATIONCONVERSIONS_H_

#include "EulerAngle.h"
#include "Matrix.h"
#include "Quaternion.h"

namespace mathtool {

  //////////////////////////////////////////////////////////////////////////////
  // Conversions to Matrix3x3
  //////////////////////////////////////////////////////////////////////////////

  /// Convert a vector _v into the skew-symmetric matrix representation
  /// _m = [_v]x for computing cross products.
  /// For some other vector u, _m*u = _v x u.
  inline
  Matrix3x3&
  crossProductMatrix(Matrix3x3& _m, const Vector3d& _v)
  {
    _m[0][0] =     0.; _m[0][1] = -_v[2]; _m[0][2] =  _v[1];
    _m[1][0] =  _v[2]; _m[1][1] =     0.; _m[1][2] = -_v[0];
    _m[2][0] = -_v[1]; _m[2][1] =  _v[0]; _m[2][2] =     0.;
    return _m;
  }


  inline
  Matrix3x3&
  convertFromQuaternion(Matrix3x3& _m, const Quaternion& _q)
  {
    const double w = _q.real(),
                 x = _q.imaginary()[0],
                 y = _q.imaginary()[1],
                 z = _q.imaginary()[2];

    _m[0][0] = 1.0 - 2.0*(y*y + z*z);
    _m[0][1] = 2.0*(x*y - w*z);
    _m[0][2] = 2.0*(x*z + w*y);

    _m[1][0] = 2.0*(x*y + w*z);
    _m[1][1] = 1.0 - 2.0*(x*x + z*z);
    _m[1][2] = 2.0*(y*z - w*x);

    _m[2][0] = 2.0*(x*z - w*y);
    _m[2][1] = 2.0*(y*z + w*x);
    _m[2][2] = 1.0 - 2.0*(x*x + y*y);

    return _m;
  }


  inline
  Matrix3x3&
  convertFromEulerAngle(Matrix3x3& _m, const EulerAngle& _e)
  {
    const double sa = std::sin(_e.alpha()),
                 ca = std::cos(_e.alpha()),
                 sb = std::sin(_e.beta()),
                 cb = std::cos(_e.beta()),
                 sg = std::sin(_e.gamma()),
                 cg = std::cos(_e.gamma());

    _m[0][0] = ca*cb;
    _m[0][1] = ca*sb*sg - sa*cg;
    _m[0][2] = ca*sb*cg + sa*sg;

    _m[1][0] = sa*cb;
    _m[1][1] = sa*sb*sg + ca*cg;
    _m[1][2] = sa*sb*cg - ca*sg;

    _m[2][0] = -sb;
    _m[2][1] = cb*sg;
    _m[2][2] = cb*cg;

    return _m;
  }


  inline
  Matrix3x3&
  convertFromEulerVector(Matrix3x3& _m, const EulerVector& _e)
  {
    const Vector3d e = _e.normalize();
    const double magnitude = _e.norm(),
                 sinM = std::sin(magnitude),
                 cosM = std::cos(magnitude),
                 x    = 1. - cosM;

    _m[0][0] = x*(e[0]*e[0]) + cosM;
    _m[1][0] = x*(e[0]*e[1]) + sinM*e[2];
    _m[2][0] = x*(e[0]*e[2]) - sinM*e[1];

    _m[0][1] = x*(e[1]*e[0]) - sinM*e[2];
    _m[1][1] = x*(e[1]*e[1]) + cosM;
    _m[2][1] = x*(e[1]*e[2]) + sinM*e[0];

    _m[0][2] = x*(e[2]*e[0]) + sinM*e[1];
    _m[1][2] = x*(e[2]*e[1]) - sinM*e[0];
    _m[2][2] = x*(e[2]*e[2]) + cosM;

    return _m;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Conversions to Quaternions
  //////////////////////////////////////////////////////////////////////////////

  inline
  Quaternion&
  convertFromMatrix(Quaternion& _q, const Matrix3x3& _m)
  {
    double t = trace(_m); //trace of the matrix
    double s, x, y, z, w;

    if(t > 0) {
      s = 0.5 / std::sqrt(1 + t);
      w = 0.25 / s;
      x = (_m[2][1] - _m[1][2]) * s;
      y = (_m[0][2] - _m[2][0]) * s;
      z = (_m[1][0] - _m[0][1]) * s;
    }
    else{ //if(t <= 0)
      //column 0
      if(_m[0][0] > _m[1][1] && _m[0][0] > _m[2][2]) {
        s = std::sqrt(1.0 + _m[0][0] - _m[1][1] - _m[2][2]) * 2;
        w = (_m[2][1] - _m[1][2]) / s;
        x = 0.25 * s;
        y = (_m[0][1] + _m[1][0]) / s;
        z = (_m[0][2] + _m[2][0]) / s;
      }
      //column 1
      else if (_m[1][1] > _m[2][2]) {
        s = std::sqrt(1.0 + _m[1][1] - _m[0][0] - _m[2][2]) * 2;
        w = (_m[0][2] - _m[2][0]) / s;
        x = (_m[0][1] + _m[1][0]) / s;
        y = 0.25 * s;
        z = (_m[1][2] + _m[2][1]) / s;
      }
      //column 2
      else {
        s = std::sqrt(1.0 + _m[2][2] - _m[0][0] - _m[1][1]) * 2;
        w = (_m[1][0] - _m[0][1]) / s;
        x = (_m[0][2] + _m[2][0]) / s;
        y = (_m[1][2] + _m[2][1]) / s;
        z = 0.25 * s;
      }
    }

    //The quaternion is then defined as: Q = | W X Y Z |
    _q.real() = w;
    _q.imaginary()(x, y, z);
    return _q;
  }


  inline
  Quaternion&
  convertFromEulerAngle(Quaternion& _q, const EulerAngle& _e)
  {
    const double cosG = std::cos(_e.gamma() / 2.),
                 cosB = std::cos(_e.beta()  / 2.),
                 cosA = std::cos(_e.alpha() / 2.),
                 sinG = std::sin(_e.gamma() / 2.),
                 sinB = std::sin(_e.beta()  / 2.),
                 sinA = std::sin(_e.alpha() / 2.);

    return _q = Quaternion(cosG * cosB * cosA + sinG * sinB * sinA,
                          {sinG * cosB * cosA - cosG * sinB * sinA,
                           cosG * sinB * cosA + sinG * cosB * sinA,
                           cosG * cosB * sinA - sinG * sinB * cosA});
  }


  inline
  Quaternion&
  convertFromEulerVector(Quaternion& _q, const EulerVector& _v)
  {
    const double magnitude = _v.norm();
    if(mathtool::approx(magnitude, 0.))
      return _q = Quaternion();

    const double sin       = std::sin(magnitude / 2.),
                 cos       = std::cos(magnitude / 2.);
    return _q = Quaternion(cos, _v * (sin / magnitude));
  }

  //////////////////////////////////////////////////////////////////////////////
  // Conversions to EulerAngles
  //////////////////////////////////////////////////////////////////////////////

  inline
  EulerAngle&
  convertFromQuaternion(EulerAngle& _e, const Quaternion& _q)
  {
    const double r = _q.real(),
                 i = _q.imaginary()[0],
                 j = _q.imaginary()[1],
                 k = _q.imaginary()[2];

    const double yaw   = std::atan2(2. * (r*k + i*j), 1. - 2. * (j*j + k*k)),
                 pitch = std::asin( 2. * (r*j - k*i)),
                 roll  = std::atan2(2. * (r*i + j*k), 1. - 2. * (i*i + j*j));

    return _e = EulerAngle(yaw, pitch, roll);
  }


  inline
  EulerAngle&
  convertFromMatrix(EulerAngle& _e, const Matrix3x3& _m)
  {
    _e.beta() = std::atan2(-_m[2][0],
                           std::sqrt(_m[2][1]*_m[2][1] + _m[2][2]*_m[2][2]));
    if(std::cos(_e.beta()) > 0) {
      _e.alpha() = std::atan2(_m[1][0], _m[0][0]);
      _e.gamma() = std::atan2(_m[2][1], _m[2][2]);
    }
    else {
      _e.alpha() = std::atan2(-_m[1][0], -_m[0][0]);
      _e.gamma() = std::atan2(-_m[2][1], -_m[2][2]);
    }
    return _e;
  }


  inline
  EulerAngle&
  convertFromEulerVector(EulerAngle& _e, const EulerVector& _v)
  {
    Quaternion q;
    convertFromEulerVector(q, _v);
    return convertFromQuaternion(_e, q);
  }


  //////////////////////////////////////////////////////////////////////////////
  // Conversions to EulerVectors
  //////////////////////////////////////////////////////////////////////////////

  inline
  EulerVector&
  convertFromQuaternion(EulerVector& _v, const Quaternion& _q)
  {
    const double norm = _q.imaginary().norm();
    if(mathtool::approx(norm, 0.))
      return _v(0, 0, 0);
    _v = (2. * std::acos(_q.real()) / norm) * _q.imaginary();
    return _v.eulerSelfNormalize();
  }


  inline
  EulerVector&
  convertFromMatrix(EulerVector& _v, const Matrix3x3& _m)
  {
    // Use intermediate conversion to quaternion to avoid singularity cases.
    Quaternion q;
    convertFromMatrix(q, _m);
    return convertFromQuaternion(_v, q);
  }


  inline
  EulerVector&
  convertFromEulerAngle(EulerVector& _v, const EulerAngle& _e)
  {
    // Use intermediate conversion to quaternion to avoid singularity cases.
    Quaternion q;
    convertFromEulerAngle(q, _e);
    return convertFromQuaternion(_v, q);
  }

}

#endif
