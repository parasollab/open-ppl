/* Defines 3D orientations through rotation matrices. Handles conversion between
 * types of rotations. Input and output is in Euler Angles.
 *
 * Many kinds of representation are implemented here, such as
 * - Euler Angle
 * - Matrix
 * - Quaternion
 * Operations for their representations are provided.
 * Coversion between these representation types are also available.
 */

#ifndef ORIENTATION_H_
#define ORIENTATION_H_

#include "RotationConversions.h"
#include "Vector.h"

namespace mathtool {

  //////////////////////////////////////////////////////////////////////////////
  // MatrixOrientation - class for orientations completely through matrices
  //////////////////////////////////////////////////////////////////////////////
  class MatrixOrientation {
    public:

      //Constructor for EulerAngles
      //Also default constructor
      MatrixOrientation(const EulerAngle& _e = EulerAngle()) {convertFromEulerAngle(m_matrix, _e);}

      //Constructor for Matrix
      MatrixOrientation(const Matrix3x3& _m) : m_matrix(_m) {}

      //Constructor for Quaternions
      MatrixOrientation(const Quaternion& _q) {convertFromQuaternion(m_matrix, _q);}

      //access
      Matrix3x3& matrix() {return m_matrix;}
      const Matrix3x3& matrix() const {return m_matrix;}

      /// Get a basis vector for the rotated frame.
      /// @param _i The dimension.
      /// @return The basis vector for dimension _i.
      Vector3d getBasis(const size_t _i) const {
        return Vector3d(m_matrix[0][_i], m_matrix[1][_i], m_matrix[2][_i]);
      }

      //equality - if they aren't the same type: convert _o to type of *this and
      //compare
      bool operator==(const MatrixOrientation& _o) const {
        return m_matrix == _o.m_matrix;
      }
      //inequality
      bool operator!=(const MatrixOrientation& _o) const {
        return !(*this == _o);
      }

      //Matrix times vector. If type is Euler convert to matrix then multiply.
      //Both quaternion and matrix apply directly the rotation.
      Vector3d operator*(const Vector3d & _v) const {
        return m_matrix * _v;
      }

      //Self Multiplication.
      //convert the orientations to the same type and perform composition.
      MatrixOrientation& operator*=(const MatrixOrientation& _o) {
        m_matrix = m_matrix * _o.m_matrix;
        return *this;
      }

      //inverse
      MatrixOrientation operator-() const {
        MatrixOrientation o;
        o.m_matrix = m_matrix.transpose();
        return o;
      }
      //multiplication
      MatrixOrientation operator*(const MatrixOrientation& _o) const {
        MatrixOrientation o(*this);
        o *= _o;
        return o;
      }

    private:

      Matrix3x3 m_matrix;
  };

  inline Vector3d operator*(const Vector3d& _v, const MatrixOrientation& _o) {
    return _o * _v;
  }

  inline std::istream& operator>>(std::istream& _is, MatrixOrientation& _o) {
    EulerAngle e;
    _is >> e;
    Matrix3x3 m;
    convertFromEulerAngle(m, e);
    _o.matrix() = m;
    return _is;
  }

  inline std::ostream& operator<<(std::ostream& _os, const MatrixOrientation& _o) {
    EulerAngle e;
    convertFromMatrix(e, _o.matrix());
    return _os << e;
  }

  //////////////////////////////////////////////////////////////////////////////
  // QuaternionOrientation
  //////////////////////////////////////////////////////////////////////////////
  class QuaternionOrientation {
    public:

      // The type of Orientation instance.
      enum OrientationType {
        EULER,
        MATRIX,
        QUATERNION
      };

      //Constructor for EulerAngles
      //Also default constructor
      QuaternionOrientation(const EulerAngle& _e = EulerAngle()) {convertFromEulerAngle(m_quaternion, _e);}

      //Constructor for Matrix
      QuaternionOrientation(const Matrix3x3& _m) {convertFromMatrix(m_quaternion, _m);}

      //Constructor for Quaternions
      QuaternionOrientation(const Quaternion& _q) : m_quaternion(_q) {}

      Quaternion& quaternion() {return m_quaternion;}
      const Quaternion& quaternion() const {return m_quaternion;}

      //equality - if they aren't the same type: convert _o to type of *this and
      //compare
      bool operator==(const QuaternionOrientation& _o) const {
        return m_quaternion == _o.m_quaternion;
      }
      //inequality
      bool operator!=(const QuaternionOrientation& _o) const {
        return !(*this == _o);
      }

      //Matrix times vector. If type is Euler convert to matrix then multiply.
      //Both quaternion and matrix apply directly the rotation.
      Vector3d operator*(const Vector3d & _v) const {
        Quaternion q = m_quaternion.normalized();
        return (q * _v * -q).imaginary();
      }

      //Self Multiplication.
      //convert the orientations to the same type and perform composition.
      QuaternionOrientation& operator*=(const QuaternionOrientation& _o) {
        m_quaternion *= _o.m_quaternion;
        return *this;
      }

      //inverse
      QuaternionOrientation operator-() const {
        QuaternionOrientation o;
        o.m_quaternion = -m_quaternion;
        return o;
      }
      //multiplication
      QuaternionOrientation operator*(const QuaternionOrientation& _o) const {
        QuaternionOrientation o(*this);
        o *= _o;
        return o;
      }

    private:

      Quaternion m_quaternion;
  };

  inline Vector3d operator*(const Vector3d& _v, const QuaternionOrientation& _o) {
    return _o * _v;
  }

  inline std::istream& operator>>(std::istream& _is, QuaternionOrientation& _o) {
    EulerAngle e;
    _is >> e;
    Quaternion q;
    convertFromEulerAngle(q, e);
    _o.quaternion() = q;
    return _is;
  }

  inline std::ostream& operator<<(std::ostream& _os, const QuaternionOrientation& _o) {
    EulerAngle e;
    convertFromQuaternion(e, _o.quaternion());
    return _os << e;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Typedef to Orientation
  //////////////////////////////////////////////////////////////////////////////
  typedef MatrixOrientation Orientation;
}

#endif

