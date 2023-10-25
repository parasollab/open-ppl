/* This class contains transformational information and operations
 * Orientation and position of object are stored in the instance of this class.
 * Position of object are stored as instance of Vector3D.
 */

#ifndef TRANSFORMATION_H_
#define TRANSFORMATION_H_

#include <vector>

#include "Vector.h"
#include "Orientation.h"

namespace mathtool {

  class Transformation {
    public:

      Transformation(
          const Vector3d& _position = Vector3d(),
          const Orientation& _orientation = Orientation()) :
        m_translation(_position), m_rotation(_orientation) {}

      //access
      Vector3d& translation() {return m_translation;}
      const Vector3d& translation() const {return m_translation;}
      Orientation& rotation() {return m_rotation;}
      const Orientation& rotation() const {return m_rotation;}

      //equality
      bool operator==(const Transformation& _t) const {
        return m_translation == _t.m_translation && m_rotation == _t.m_rotation;
      }
      //inequality
      bool operator!=(const Transformation& _t) const {
        return !(*this == _t);
      }

      //self multiplication
      //  Refer to Craig Eq 2.45
      Transformation& operator*=(const Transformation& _t) {
        m_translation = m_rotation * _t.m_translation + m_translation;
        m_rotation = m_rotation * _t.m_rotation;
        return *this;
      }

      //multiplication
      Transformation operator*(const Transformation& _t) const {
        Transformation t(*this);
        t *= _t;
        return t;
      }

      //vector multiplication
      Vector3d operator*(const Vector3d& _v) const {
        return m_rotation * _v + m_translation;
      }

      //inversion
      //  Creates the reverse transformation of "this"
      //  Refer to Craig Eq 2.45
      Transformation operator-() const {
        Orientation inverse = -m_rotation;
        return Transformation(-(inverse * m_translation), inverse);
      }

      /// Get a 6-dof configuration-space representation of this.
      std::vector<double> GetCfg() const {
        EulerAngle e;
        convertFromMatrix(e, m_rotation.matrix());
        return std::vector<double>{m_translation[0],
                                   m_translation[1],
                                   m_translation[2],
                                   e.gamma() / PI,
                                   e.beta() / PI,
                                   e.alpha() / PI};
      }

      friend std::istream& operator>>(std::istream& _is, Transformation& _t);

    private:
      Vector3d m_translation; //Translation
      Orientation m_rotation; //Rotation
  };

  inline Vector3d operator*(const Vector3d& _v, const Transformation& _t) {
    return _t * _v;
  }

  inline std::istream& operator>>(std::istream& _is, Transformation& _t) {
    return _is >> _t.m_translation >> _t.m_rotation;
  }

  inline std::ostream& operator<<(std::ostream& _os, const Transformation& _t) {
    return _os << _t.translation() << _t.rotation();
  }

}

#endif
