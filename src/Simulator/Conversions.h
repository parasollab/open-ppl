#include "btBulletDynamicsCommon.h"

#include "Transformation.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"

#include <iostream>


using namespace mathtool;

btVector3
ToBullet(const Vector3d& _v) {
  return btVector3(_v[0], _v[1], _v[2]);
}

btQuaternion
ToBullet(const Quaternion& _q) {
  return btQuaternion(ToBullet(_q.imaginary()), _q.real());
}

btMatrix3x3 
ToBullet(const Matrix3x3& _m) {
  return btMatrix3x3(_m[0][0],_m[0][1],_m[0][2],
                     _m[1][0],_m[1][1],_m[1][2],
                     _m[2][0],_m[2][1],_m[2][2]
                    );
}

btTransform
ToBullet(const Transformation& _t) {
  btTransform trans;
  trans.setIdentity();
  trans.setOrigin(ToBullet(_t.translation()));
  return trans;
  //return btTransform(ToBullet(_t.rotation().matrix()), ToBullet(_t.translation()));
}


std::ostream& operator<< (std::ostream& _out, const btVector3& _v) {
  _out << _v[0] << ',' << _v[1] << ',' << _v[2];
  return _out;
}
