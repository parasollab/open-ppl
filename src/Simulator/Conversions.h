#ifndef SIMULATOR_CONVERSIONS_H_
#define SIMULATOR_CONVERSIONS_H_

#include <iostream>

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"

#include "Matrix.h"
#include "Quaternion.h"
#include "Transformation.h"
#include "Vector.h"

using namespace mathtool;


/*----------------------------- Bullet Helpers -------------------------------*/

//For changing the friction of an entire btMultiBody (all links + base):
inline
void
SetUniformFrictionOnBulletMultiBody(
    btMultiBody* _body, const double _frictionCoeff) {
  // Note that btMultiBody has links, and each link (including the base) will
  // have its own btMultiBodyLinkCollider, and each one has its own friction
  // setting. This means we must hit each one and update our uniform friction:
  _body->getBaseCollider()->setFriction(_frictionCoeff);
  for(int i = 0; i < _body->getNumLinks(); i++)
    _body->getLink(i).m_collider->setFriction(_frictionCoeff);
}

/*--------------------- Conversion from PMPL to Bullet -----------------------*/

inline
btVector3
ToBullet(const Vector3d& _v) {
  return btVector3(_v[0], _v[1], _v[2]);
}


inline
btQuaternion
ToBullet(const Quaternion& _q) {
  const Vector3d imag = _q.imaginary();
  return btQuaternion(imag[0], imag[1], imag[2], _q.real());
  //This was the wrong constructor:
  //btQuaternion(ToBullet(_q.imaginary()), _q.real());
}


inline
btMatrix3x3
ToBullet(const Matrix3x3& _m) {
  return btMatrix3x3(_m[0][0],_m[0][1],_m[0][2],
                     _m[1][0],_m[1][1],_m[1][2],
                     _m[2][0],_m[2][1],_m[2][2]
                    );
}


inline
btTransform
ToBullet(const Transformation& _t) {
  return btTransform(ToBullet(_t.rotation().matrix()),
      ToBullet(_t.translation()));
}

/*--------------------- Conversion from Bullet to PMPL -----------------------*/

inline
Vector3d
ToPMPL(const btVector3& _v) {
  return Vector3d(_v[0], _v[1], _v[2]);
}


inline
Matrix3x3
ToPMPL(const btMatrix3x3& _m) {
  double temp[3][3] = {{_m[0][0], _m[0][1], _m[0][2]},
                       {_m[1][0], _m[1][1], _m[1][2]},
                       {_m[2][0], _m[2][1], _m[2][2]}};
  return Matrix3x3(temp);
}


inline
Transformation
ToPMPL(const btTransform& _t) {
  return Transformation(ToPMPL(_t.getOrigin()), ToPMPL(_t.getBasis()));
}

/*------------------------ Output for Bullet Objects -------------------------*/

inline
std::ostream&
operator<<(std::ostream& _out, const btVector3& _v) {
  _out << "(" << _v[0] << ", " << _v[1] << ", " << _v[2] << ")";
  return _out;
}


inline
std::ostream&
operator<<(std::ostream& _out, const btQuaternion& _q) {
  _out << "(" << _q.getX() << ", " << _q.getY() << ", " << _q.getZ() << ", "
       << _q.getW() << ")";
  return _out;
}

/*----------------------------------------------------------------------------*/

#endif
