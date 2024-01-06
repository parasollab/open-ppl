#ifndef EULERANGLE_H_
#define EULERANGLE_H_

#include "Basic.h"

namespace mathtool {

  //////////////////////////////////////////////////////////////////////////////
  /// An Euler angle expressed in intrinsic ZYX format (also known as tait-bryan
  /// angles or yaw-pitch-roll). Alpha is the first rotation about the original
  /// Z axis, followed by beta about the new Y axis, and finally gamma about the
  /// new X axis. As applied to aircraft this assumes that +x is the forward
  /// direction and +z is up.
  //////////////////////////////////////////////////////////////////////////////
  class EulerAngle {

    public:

      EulerAngle(double _alpha = 0.0, double _beta = 0.0, double _gamma = 0.0) {
        operator()(_alpha, _beta, _gamma);
      }

      //assignment
      EulerAngle& operator=(const EulerAngle& _e){
        return operator()(_e.m_alpha, _e.m_beta, _e.m_gamma);
      }
      //set the values of the EulerAngle
      EulerAngle& operator()(double _alpha = 0.0, double _beta = 0.0, double _gamma = 0.0) {
        m_alpha = _alpha; m_beta = _beta; m_gamma = _gamma;
        return *this;
      }

      //access
      double alpha() const noexcept {return m_alpha;}
      double beta()  const noexcept {return m_beta;}
      double gamma() const noexcept {return m_gamma;}
      double& alpha() noexcept {return m_alpha;}
      double& beta()  noexcept {return m_beta;}
      double& gamma() noexcept {return m_gamma;}

      //equality
      bool operator==(const EulerAngle& _e) const {
        return m_alpha == _e.m_alpha && m_beta == _e.m_beta && m_gamma == _e.m_gamma;
      }
      //inequality
      bool operator!=(const EulerAngle& _e) const {
        return !(*this == _e);
      }

      //self addition
      EulerAngle& operator+=(const EulerAngle& _e) {
        m_alpha = fmod(_e.m_alpha + m_alpha, TWOPI);
        m_beta = fmod(_e.m_beta + m_beta, TWOPI);
        m_gamma = fmod(_e.m_gamma + m_gamma, TWOPI);
        return *this;
      }
      //self subtraction
      EulerAngle& operator-=(const EulerAngle& _e) {
        m_alpha = fmod(_e.m_alpha - m_alpha, TWOPI);
        m_beta = fmod(_e.m_beta - m_beta, TWOPI);
        m_gamma = fmod(_e.m_gamma - m_gamma, TWOPI);
        return *this;
      }

      //inversion
      EulerAngle operator-() const {
        return EulerAngle(fmod(-m_alpha + PI, TWOPI), m_beta, fmod(-m_gamma + PI, TWOPI));
      }
      //addition
      EulerAngle operator+(const EulerAngle& _e) const {
        EulerAngle e(*this);
        e += _e;
        return e;
      }
      //subtraction
      EulerAngle operator-(const EulerAngle& _e) const {
        EulerAngle e(*this);
        e -= _e;
        return e;
      }

    private:

      double m_alpha, m_beta, m_gamma;
  };


  /// Read in an Euler angle specified in degrees.
  /// @WARNING This reads the values backwards in the order {gamma, beta, alpha}
  ///          which corresponds to the X, Y, and Z components. Before we can
  ///          correct this we need a provide a script to fix old environment
  ///          files automatically.
  inline std::istream& operator>>(std::istream& _is, EulerAngle& _e) {
    double a, b, g;
    _is >> g >> b >> a;
    _e.alpha() = std::fmod(degToRad(a), TWOPI);
    _e.beta()  = std::fmod(degToRad(b), TWOPI);
    _e.gamma() = std::fmod(degToRad(g), TWOPI);
    return _is;
  }

  /// Output an Euler angle in degrees.
  /// @WARNING This writes the values backwards in the order {gamma, beta, alpha}
  ///          which corresponds to the X, Y, and Z components. Before we can
  ///          correct this we need a provide a script to fix old environment
  ///          files automatically.
  inline std::ostream& operator<<(std::ostream& _os, const EulerAngle& _e) {
    std::ios::fmtflags f(_os.flags());
    _os << std::fixed
        << std::setprecision(16) << radToDeg(std::fmod(_e.gamma(), TWOPI)) << " "
        << std::setprecision(16) << radToDeg(std::fmod(_e.beta(),  TWOPI)) << " "
        << std::setprecision(16) << radToDeg(std::fmod(_e.alpha(), TWOPI));
    _os.flags(f);
    return _os;
  }
}

#endif
