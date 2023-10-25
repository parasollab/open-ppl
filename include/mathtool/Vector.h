#ifndef VECTOR_H_
#define VECTOR_H_

#include "Basic.h"

namespace mathtool{

  template<class T, class Func>
  using CanEvaluateTo = typename std::enable_if<
      std::is_convertible<decltype(std::declval<Func>()()), T>::value>::type;

  //////////////////////////////////////////////////////////////////////////////
  // General D-dimensional Vector
  //////////////////////////////////////////////////////////////////////////////
  template<class T, size_t D>
  class Vector {
    public:

      typedef T* iterator;
      typedef const T* const_iterator;

      //construction
      Vector() {
        for(size_t i = 0; i<D; ++i) m_v[i] = T();
      }
      Vector(const Vector& _v) {
        for(size_t i = 0; i<D; ++i) m_v[i] = _v[i];
      }
      explicit Vector(const T _t[D]) {
        for(size_t i = 0; i<D; ++i) m_v[i] = _t[i];
      }
      template<class Generator, class = CanEvaluateTo<T, Generator>>
      Vector(const Generator& g){
        for(size_t i = 0; i<D; ++i) m_v[i] = g();
      }

      //assignment
      Vector& operator=(const Vector& _v) {
        for(size_t i = 0; i<D; ++i) m_v[i] = _v[i];
        return *this;
      }
      Vector& operator=(const T (&_t)[D]) {
        for(size_t i = 0; i<D; ++i) m_v[i] = _t[i];
        return *this;
      }

      //access
      typedef T (&arr)[D];
      typedef const T (&carr)[D];
      operator arr() {return m_v;}
      operator carr() const {return m_v;}
      const_iterator begin() const {return m_v;}
      const_iterator end() const {return begin()+D;}
      iterator begin() {return m_v;}
      iterator end() {return begin()+D;}

      //equality
      bool operator==(const Vector& _v) const {
        for(size_t i = 0; i < D; ++i)
          if(!approx(m_v[i], _v[i])) return false;
        return true;
      }
      //inequality
      bool operator!=(const Vector& _v) const {
        return !(*this == _v);
      }

      //self addition
      Vector& operator+=(const Vector& _v) {
        for(size_t i = 0; i<D; ++i) m_v[i] += _v[i];
        return *this;
      }
      //self subtraction
      Vector& operator-=(const Vector& _v) {
        for(size_t i = 0; i<D; ++i) m_v[i] -= _v[i];
        return *this;
      }
      //self scalar multiply
      Vector& operator*=(const T& _d) {
        for(size_t i = 0; i<D; ++i) m_v[i] *= _d;
        return *this;
      }
      //self scalar divide
      Vector& operator/=(const T& _d) {
        for(size_t i = 0; i<D; ++i) m_v[i] /= _d;
        return *this;
      }
      //self component *
      Vector& operator^=(const Vector& _v) {
        for(size_t i = 0; i<D; ++i) m_v[i] *= _v[i];
        return *this;
      }

      //negation
      Vector operator-() const {
        Vector v;
        for(size_t i = 0; i<D; ++i) v.m_v[i] = -m_v[i];
        return v;
      }
      //addition
      Vector operator+(const Vector& _v) const {
        Vector v(*this);
        return v += _v;
      }
      //subtraction
      Vector operator-(const Vector& _v) const {
        Vector v(*this);
        return v -= _v;
      }
      //scalar multiply
      Vector operator*(const T& _d) const {
        Vector v(*this);
        return v *= _d;
      }
      //scalar divide
      Vector operator/(const T& _d) const {
        Vector v(*this);
        return v /= _d;
      }
      //component *
      Vector operator^(const Vector& _v) const {
        Vector v(*this);
        return v ^= _v;
      }

      //dot product
      T operator*(const Vector& _v) const {
        T dot = 0;
        for(size_t i = 0; i<D; ++i) dot += m_v[i] * _v[i];
        return dot;
      }
      //magnitude
      T norm() const {
        return std::sqrt(normsqr());
      }
      //magnitude squared
      T normsqr() const {
        return (*this)*(*this);
      }
      //normalized vector
      Vector& selfNormalize() {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return *this = Vector();
        return *this /= n;
      }
      Vector normalize() const {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return Vector();
        return *this / n;
      }
      //Projections
      //find |component| of this along _v's direction
      T comp(const Vector& _v) const {
        return (*this) * _v.normalize();
      }
      //find vector component of this in _v's direction
      Vector proj(const Vector& _v) const {
        return (*this * _v)/(_v * _v) * _v;
      }
      //find vector component of this orthogonal to _v's direction
      Vector orth(const Vector& _v) const {
        return *this - proj(_v);
      }
      //scale vector
      Vector& selfScale(const T& _l) {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return *this = Vector();
        return *this *= (_l/n);
      }
      Vector scale(const T& _l) const {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return Vector();
        return *this * (_l/n);
      }

    private:
      T m_v[D];
  };

  //////////////////////////////////////////////////////////////////////////////
  // Specialized 2-dimensional Vector
  //////////////////////////////////////////////////////////////////////////////
  template<class T>
  class Vector<T,2> {
    public:

      typedef T* iterator;
      typedef const T* const_iterator;

      //construction
      explicit Vector(const T& _x = T()) : Vector(_x, T()) { }
      Vector(const T& _x, const T& _y) {
        m_v[0] = _x; m_v[1] = _y;
      }
      Vector(const Vector& _v) {
        m_v[0] = _v[0]; m_v[1] = _v[1];
      }
      explicit Vector(const T _t[2]) {
        m_v[0] = _t[0]; m_v[1] = _t[1];
      }
      template <class Generator, class = CanEvaluateTo<T, Generator>>
      Vector(const Generator& g){
        m_v[0] = g(); m_v[1] = g();
      }

      //assignment
      Vector& operator()(const T& _x, const T& _y){
        m_v[0] = _x; m_v[1] = _y;
        return *this;
      }
      Vector& operator=(const Vector& _v) {
        m_v[0] = _v[0]; m_v[1] = _v[1];
        return *this;
      }
      Vector& operator=(const T (&_t)[2]) {
        m_v[0] = _t[0]; m_v[1] = _t[1];
        return *this;
      }

      //access
      typedef T (&arr)[2];
      typedef const T (&carr)[2];
      operator arr() {return m_v;}
      operator carr() const {return m_v;}
      const_iterator begin() const {return m_v;}
      const_iterator end() const {return begin()+2;}
      iterator begin() {return m_v;}
      iterator end() {return begin()+2;}

      //equality
      bool operator==(const Vector& _v) const {
        return approx(m_v[0], _v[0]) && approx(m_v[1], _v[1]);
      }
      //inequality
      bool operator!=(const Vector& _v) const {
        return !(*this == _v);
      }

      //self addition
      Vector& operator+=(const Vector& _v) {
        m_v[0] += _v[0]; m_v[1] += _v[1];
        return *this;
      }
      //self subtraction
      Vector& operator-=(const Vector& _v) {
        m_v[0] -= _v[0]; m_v[1] -= _v[1];
        return *this;
      }
      //self scalar multiply
      Vector& operator*=(const T& _d) {
        m_v[0] *= _d; m_v[1] *= _d;
        return *this;
      }
      //self scalar divide
      Vector& operator/=(const T& _d) {
        m_v[0] /= _d; m_v[1] /= _d;
        return *this;
      }
      //self component *
      Vector& operator^=(const Vector& _v) {
        m_v[0] *= _v[0]; m_v[1] *= _v[1];
        return *this;
      }

      //negation
      Vector operator-() const {
        return Vector(-m_v[0], -m_v[1]);
      }
      //addition
      Vector operator+(const Vector& _v) const {
        return Vector(m_v[0] + _v[0], m_v[1] + _v[1]);
      }
      //subtraction
      Vector operator-(const Vector& _v) const {
        return Vector(m_v[0] - _v[0], m_v[1] - _v[1]);
      }
      //scalar multiply
      Vector operator*(const T& _d) const {
        return Vector(m_v[0] * _d, m_v[1] * _d);
      }
      //scalar divide
      Vector operator/(const T& _d) const {
        return Vector(m_v[0] / _d, m_v[1] / _d);
      }
      //component *
      Vector operator^(const Vector& _v) const {
        return Vector(m_v[0] * _v[0], m_v[1] * _v[1]);
      }
      //cross product magnitude
      T operator%(const Vector& _v) const {
        return m_v[0]*_v[1] - m_v[1]*_v[0];
      }

      //dot product
      T operator*(const Vector& _v) const {
        return m_v[0]*_v[0] + m_v[1]*_v[1];
      }
      //magnitude
      T norm() const {
        return std::sqrt(normsqr());
      }
      //magnitude squared
      T normsqr() const {
        return (*this)*(*this);
      }
      //normalized vector
      Vector& selfNormalize() {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return *this = Vector();
        return *this /= n;
      }
      Vector normalize() const {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return Vector();
        return *this / n;
      }
      //Projections
      //find |component| of this along _v's direction
      T comp(const Vector& _v) const {
        return (*this) * _v.normalize();
      }
      //find vector component of this in _v's direction
      Vector proj(const Vector& _v) const {
        return (*this * _v)/(_v * _v) * _v;
      }
      //find vector component of this orthogonal to _v's direction
      Vector orth(const Vector& _v) const {
        return *this - proj(_v);
      }
      //scale vector
      Vector& selfScale(const T& _l) {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return *this = Vector();
        return *this *= (_l/n);
      }
      Vector scale(const T& _l) const {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return Vector();
        return *this * (_l/n);
      }

      //vector angle
      double getAngle() const {return atan2(m_v[1], m_v[0]);}
      double getAngled() const {return radToDeg(getAngle());}
      //rotate vector
      //TODO: make 2d rotations match 3d rotations
      Vector& selfRotate(double _rad){
        double c = cos(_rad), s = sin(_rad);
        return operator()(m_v[0]*c - m_v[1]*s, m_v[0]*s + m_v[1]*c);
      }
      Vector& selfRotated(double _deg){return selfRotate(degToRad(_deg));}
      Vector& rotate(double _rad) const{ return Vector(*this).selfRotate(_rad);  }
      Vector& rotated(double _deg) const{ return Vector(*this).selfRotated(_deg);  }

    private:
      T m_v[2];
  };

  //////////////////////////////////////////////////////////////////////////////
  // Specialized 3-dimensional Vector
  //////////////////////////////////////////////////////////////////////////////
  template<class T>
  class Vector<T,3> {
    public:

      typedef T* iterator;
      typedef const T* const_iterator;

      //construction
      explicit Vector(const T& _x = T(), const T& _y = T())
        : Vector(_x, _y, T()) { }
      Vector(const T& _x, const T& _y, const T& _z) {
        m_v[0] = _x; m_v[1] = _y; m_v[2] = _z;
      }
      Vector(const Vector& _v) {
        m_v[0] = _v[0]; m_v[1] = _v[1]; m_v[2] = _v[2];
      }
      explicit Vector(const T _t[3]) {
        m_v[0] = _t[0]; m_v[1] = _t[1]; m_v[2] = _t[2];
      }
      template<class Generator, class = CanEvaluateTo<T, Generator>>
      Vector(const Generator& g){
        m_v[0] = g(); m_v[1] = g(); m_v[2] = g();
      }

      //assignment
      Vector& operator()(const T& _x, const T& _y, const T& _z){
        m_v[0] = _x; m_v[1] = _y; m_v[2] = _z;
        return *this;
      }
      Vector& operator=(const Vector& _v) {
        m_v[0] = _v[0]; m_v[1] = _v[1]; m_v[2] = _v[2];
        return *this;
      }
      Vector& operator=(const T _v[3]) {
        m_v[0] = _v[0]; m_v[1] = _v[1]; m_v[2] = _v[2];
        return *this;
      }

      //access
      typedef T (&arr)[3];
      typedef const T (&carr)[3];
      operator arr() {return m_v;}
      operator carr() const {return m_v;}
      const_iterator begin() const {return m_v;}
      const_iterator end() const {return begin()+3;}
      iterator begin() {return m_v;}
      iterator end() {return begin()+3;}

      //equality
      bool operator==(const Vector& _v) const {
        return approx(m_v[0], _v[0]) && approx(m_v[1], _v[1]) &&
            approx(m_v[2], _v[2]);
      }
      //inequality
      bool operator!=(const Vector& _v) const {
        return !(*this == _v);
      }

      //self addition
      Vector& operator+=(const Vector& _v) {
        m_v[0] += _v[0]; m_v[1] += _v[1]; m_v[2] += _v[2];
        return *this;
      }
      //self subtraction
      Vector& operator-=(const Vector& _v) {
        m_v[0] -= _v[0]; m_v[1] -= _v[1]; m_v[2] -= _v[2];
        return *this;
      }
      //self scalar multiply
      Vector& operator*=(const T& _d) {
        m_v[0] *= _d; m_v[1] *= _d; m_v[2] *= _d;
        return *this;
      }
      //self scalar divide
      Vector& operator/=(const T& _d) {
        m_v[0] /= _d; m_v[1] /= _d; m_v[2] /= _d;
        return *this;
      }
      //self component *
      Vector& operator^=(const Vector& _v) {
        m_v[0] *= _v[0]; m_v[1] *= _v[1]; m_v[2] *= _v[2];
        return *this;
      }
      //self cross product
      Vector& operator%=(const Vector& _v) {
        T v0 = m_v[0], v1 = m_v[1], v2 = m_v[2];
        m_v[0] = v1 * _v[2] - v2 * _v[1];
        m_v[1] = v2 * _v[0] - v0 * _v[2];
        m_v[2] = v0 * _v[1] - v1 * _v[0];
        return *this;
      }

      //negation
      Vector operator-() const {
        return Vector(-m_v[0], -m_v[1], -m_v[2]);
      }
      //addition
      Vector operator+(const Vector& _v) const {
        return Vector(m_v[0] + _v[0], m_v[1] + _v[1], m_v[2] + _v[2]);
      }
      //subtraction
      Vector operator-(const Vector& _v) const {
        return Vector(m_v[0] - _v[0], m_v[1] - _v[1], m_v[2] - _v[2]);
      }
      //scalar multiply
      Vector operator*(const T& _d) const {
        return Vector(m_v[0] * _d, m_v[1] * _d, m_v[2] * _d);
      }
      //scalar divide
      Vector operator/(const T& _d) const {
        return Vector(m_v[0] / _d, m_v[1] / _d, m_v[2] / _d);
      }
      //component *
      Vector operator^(const Vector& _v) const {
        return Vector(m_v[0] * _v[0], m_v[1] * _v[1], m_v[2] * _v[2]);
      }
      //cross product
      Vector operator%(const Vector& _v) const {
        Vector v(*this);
        return v %= _v;
      }

      //dot product
      T operator*(const Vector& _v) const {
        return m_v[0]*_v[0] + m_v[1]*_v[1] + m_v[2]*_v[2];
      }
      //magnitude
      T norm() const {
        return std::sqrt(normsqr());
      }
      //magnitude squared
      T normsqr() const {
        return (*this)*(*this);
      }
      //normalized vector
      Vector& selfNormalize() {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return *this = Vector();
        return *this /= n;
      }
      Vector normalize() const {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return Vector();
        return *this / n;
      }
      //Projections
      //find |component| of this along _v's direction
      T comp(const Vector& _v) const {
        return (*this) * _v.normalize();
      }
      //find vector component of this in _v's direction
      Vector proj(const Vector& _v) const {
        return (*this * _v)/(_v * _v) * _v;
      }
      //find vector component of this orthogonal to _v's direction
      Vector orth(const Vector& _v) const {
        return *this - proj(_v);
      }
      //scale vector
      Vector& selfScale(const T& _l) {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return *this = Vector();
        return *this *= (_l/n);
      }
      Vector scale(const T& _l) const {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return Vector();
        return *this * (_l/n);
      }

      /// Normalize this vector to the range [-pi,pi] by adjusting the magnitude
      /// by some multiple of 2pi.
      Vector eulerNormalize() {
        const T magnitude  = norm(),
                equivalent = std::fmod(magnitude + PI, TWOPI) - PI;
        return *this * (equivalent / magnitude);
      }
      /// Normalize this vector to the range [-pi,pi] by adjusting the magnitude
      /// by some multiple of 2pi.
      Vector& eulerSelfNormalize() {
        const T magnitude  = norm();
        if(approx(magnitude, 0.))
          return this->operator()(0., 0., 0.);
        const T equivalent = std::fmod(magnitude + PI, TWOPI) - PI;
        return *this *= (equivalent / magnitude);
      }

      //rotate vector
      //TODO: make 2d rotations match 3d rotations
      ////////////////////////////////////////////////////////////////////////
      /// \brief Rotate \c this counter-clockwise about a reference axis.
      /// \param[in] _axis The reference axis.
      /// \param[in] _rad  The angle of rotation, with positive values
      ///                  corresponding to counter-clockwise rotation.
      /// \return          A self-reference after rotation.
      Vector& rotate(const Vector& _axis, const T& _rad) {
        Vector pivot  = this->proj(_axis);
        Vector arm    = *this - pivot;
        Vector xPrime = cos(_rad) * arm;
        Vector yPrime = sin(_rad) * (_axis.normalize() % arm);
        return *this = pivot + xPrime + yPrime;
      }
      Vector& rotated(const Vector& _axis, const T& _deg) {
        return rotate(_axis, degToRad(_deg));
      }
      Vector& rotateX(double _rad) {
        double c = cos(_rad), s = sin(_rad);
        return operator()(m_v[0], m_v[1]*c - m_v[2]*s, m_v[1]*s + m_v[2]*c);
      }
      Vector& rotateXd(double _deg) {return rotateX(degToRad(_deg));}
      Vector& rotateY(double _rad) {
        double c = cos(_rad), s = sin(_rad);
        return operator()(m_v[0]*c + m_v[2]*s, m_v[1], -m_v[0]*s + m_v[2]*c);
      }
      Vector& rotateYd(double _deg) {return rotateY(degToRad(_deg));}
      Vector& rotateZ(double _rad) {
        double c = cos(_rad), s = sin(_rad);
        return operator()(m_v[0]*c - m_v[1]*s, m_v[0]*s + m_v[1]*c, m_v[2]);
      }
      Vector& rotateZd(double _deg) {return rotateZ(degToRad(_deg));}

    private:
      T m_v[3];
  };

  //////////////////////////////////////////////////////////////////////////////
  // Specialized 4-dimensional Vector
  //////////////////////////////////////////////////////////////////////////////
  template<class T>
  class Vector<T,4> {
    public:

      typedef T* iterator;
      typedef const T* const_iterator;

      //construction
      explicit Vector(const T& _x = T(), const T& _y = T(), const T& _z = T())
        : Vector(_x, _y, _z, T()) { }
      Vector(const T& _x, const T& _y, const T& _z, const T& _w) {
        m_v[0] = _x; m_v[1] = _y; m_v[2] = _z; m_v[3] = _w;
      }
      Vector(const Vector& _v) {
        m_v[0] = _v[0]; m_v[1] = _v[1]; m_v[2] = _v[2]; m_v[3] = _v[3];
      }
      explicit Vector(const T _t[4]) {
        m_v[0] = _t[0]; m_v[1] = _t[1]; m_v[2] = _t[2]; m_v[3] = _t[3];
      }
      template<class Generator, class = CanEvaluateTo<T, Generator>>
      Vector(const Generator& g){
        m_v[0] = g(); m_v[1] = g(); m_v[2] = g(); m_v[3] = g();
      }

      //assignment
      Vector& operator()(const T& _x, const T& _y, const T& _z, const T& _w){
        m_v[0] = _x; m_v[1] = _y; m_v[2] = _z; m_v[3] = _w;
        return *this;
      }
      Vector& operator=(const Vector& _v) {
        m_v[0] = _v[0]; m_v[1] = _v[1]; m_v[2] = _v[2]; m_v[3] = _v[3];
        return *this;
      }
      Vector& operator=(const T (&_t)[4]) {
        m_v[0] = _t[0]; m_v[1] = _t[1]; m_v[2] = _t[2]; m_v[3] = _t[3];
        return *this;
      }

      //access
      typedef T (&arr)[4];
      typedef const T (&carr)[4];
      operator arr() {return m_v;}
      operator carr() const {return m_v;}
      const_iterator begin() const {return m_v;}
      const_iterator end() const {return begin()+4;}
      iterator begin() {return m_v;}
      iterator end() {return begin()+4;}

      //equality
      bool operator==(const Vector& _v) const {
        return approx(m_v[0], _v[0])
            && approx(m_v[1], _v[1])
            && approx(m_v[2], _v[2])
            && approx(m_v[3], _v[3]);
      }
      //inequality
      bool operator!=(const Vector& _v) const {
        return !(*this == _v);
      }

      //self addition
      Vector& operator+=(const Vector& _v) {
        m_v[0] += _v[0]; m_v[1] += _v[1]; m_v[2] += _v[2]; m_v[3] += _v[3];
        return *this;
      }
      //self subtraction
      Vector& operator-=(const Vector& _v) {
        m_v[0] -= _v[0]; m_v[1] -= _v[1]; m_v[2] -= _v[2]; m_v[2] -= _v[2];
        return *this;
      }
      //self scalar multiply
      Vector& operator*=(const T& _d) {
        m_v[0] *= _d; m_v[1] *= _d; m_v[2] *= _d; m_v[3] *= _d;
        return *this;
      }
      //self scalar divide
      Vector& operator/=(const T& _d) {
        m_v[0] /= _d; m_v[1] /= _d; m_v[2] /= _d; m_v[3] /= _d;
        return *this;
      }
      //self component *
      Vector& operator^=(const Vector& _v) {
        m_v[0] *= _v[0]; m_v[1] *= _v[1]; m_v[2] *= _v[2]; m_v[3] *= _v[3];
        return *this;
      }
      //negation
      Vector operator-() const {
        return Vector(-m_v[0], -m_v[1], -m_v[2],-m_v[3]);
      }
      //addition
      Vector operator+(const Vector& _v) const {
        return Vector(m_v[0] + _v[0], m_v[1] + _v[1], m_v[2] + _v[2], m_v[3] + _v[3]);
      }
      //subtraction
      Vector operator-(const Vector& _v) const {
        return Vector(m_v[0] - _v[0], m_v[1] - _v[1], m_v[2] - _v[2],  m_v[3] - _v[3]);
      }
      //scalar multiply
      Vector operator*(const T& _d) const {
        return Vector(m_v[0] * _d, m_v[1] * _d, m_v[2] * _d, m_v[3] * _d);
      }
      //scalar divide
      Vector operator/(const T& _d) const {
        return Vector(m_v[0] / _d, m_v[1] / _d, m_v[2] / _d, m_v[3] / _d);
      }
      //component *
      Vector operator^(const Vector& _v) const {
        return Vector(m_v[0] * _v[0], m_v[1] * _v[1], m_v[2] * _v[2], m_v[3] * _v[3]);
      }

      //dot product
      T operator*(const Vector& _v) const {
        return m_v[0]*_v[0] + m_v[1]*_v[1] + m_v[2]*_v[2] + m_v[3]*_v[3];
      }
      //magnitude
      T norm() const {
        return std::sqrt(normsqr());
      }
      //magnitude squared
      T normsqr() const {
        return (*this)*(*this);
      }
      //normalized vector
      Vector& selfNormalize() {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return *this = Vector();
        return *this /= n;
      }
      Vector normalize() const {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return Vector();
        return *this / n;
      }
      //Projections
      //find |component| of this along _v's direction
      T comp(const Vector& _v) const {
        return (*this) * _v.normalize();
      }
      //find vector component of this in _v's direction
      Vector proj(const Vector& _v) const {
        return (*this * _v)/(_v * _v) * _v;
      }
      //find vector component of this orthogonal to _v's direction
      Vector orth(const Vector& _v) const {
        return *this - proj(_v);
      }
      //scale vector
      Vector& selfScale(const T& _l) {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return *this = Vector();
        return *this *= (_l/n);
      }
      Vector scale(const T& _l) const {
        T n = norm();
        if(n < std::numeric_limits<T>::epsilon())
          return Vector();
        return *this * (_l/n);
      }

    private:
      T m_v[4];
  };

  //////////////////////////////////////////////////////////////////////////////
  // Useful functions. Input/Output and commutativity on multiply
  //////////////////////////////////////////////////////////////////////////////
  //for commutativity of scalar multiply
  template<class T, size_t D>
  inline Vector<T,D> operator*(const T& _d, const Vector<T,D>& _v) {
    return _v*_d;
  }

  template<class T, size_t D>
  inline std::ostream& operator<<(std::ostream& _os, const Vector<T,D>& _v) {
    for(size_t i = 0; i<D; ++i) _os << _v[i] << " ";
    return _os;
  }

  template<class T, size_t D>
  inline std::istream& operator>>(std::istream& _is, Vector<T,D>& _v) {
    for(size_t i=0; i<D; ++i) _is >> _v[i];
    return _is;
  }

  template<class T>
  inline Vector<T,2> unitVector(double _dirRads){
    return Vector<T,2>(cos(_dirRads), sin(_dirRads));
  }
  template<class T>
  inline Vector<T,2> unitVectord(double _dirDeg){
    return unitVector<T>(degToRad(_dirDeg));
  }

  //////////////////////////////////////////////////////////////////////////////
  // Typedef common used vector type
  //////////////////////////////////////////////////////////////////////////////
  typedef Vector<double,2> Vector2d;
  typedef Vector2d Point2d;
  typedef Vector<double,3> Vector3d;
  typedef Vector3d Point3d;
  typedef Vector3d EulerVector;
  typedef Vector<double,4> Vector4d;
  typedef Vector4d Point4d;

}

namespace std {

  //////////////////////////////////////////////////////////////////////////////
  /// Specialization of std::less for Vector objects of the same size.
  ///
  /// The values are considered in order to determine whether one Vector is
  /// 'less than' another. This is useful for containers based on binary search
  /// trees, like maps and sets.
  //////////////////////////////////////////////////////////////////////////////
  template <typename T, size_t N>
  struct less<mathtool::Vector<T, N>> {

    typedef mathtool::Vector<T, N> first_argument_type;
    typedef mathtool::Vector<T, N> second_argument_type;
    typedef bool                   result_type;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Scan each element pair until the first non-equal pair is found.
    /// @return True if a is less than b, false otherwise.
    bool operator()(const first_argument_type& _v1,
                    const second_argument_type& _v2) const {
      for(size_t i = 0; i < N; ++i) {
        const auto& a = _v1[i];
        const auto& b = _v2[i];
        if(a < b)
          return true;
        else if(b < a)
          return false;
      }
      return false;
    }
  };

}

#endif
