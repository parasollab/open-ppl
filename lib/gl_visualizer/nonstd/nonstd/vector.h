#ifndef NONSTD_VECTOR_TYPE_H_
#define NONSTD_VECTOR_TYPE_H_

#include <ctgmath>
#include <cstdlib>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <string>
#include <sstream>
#include <utility>

#include "nonstd/runtime.h"
#include "nonstd/numerics.h"

namespace nonstd {

  //////////////////////////////////////////////////////////////////////////////
  /// A 1-vector of dimension N. The type T must be a floating-point type with
  /// standard numeric operators.
  //////////////////////////////////////////////////////////////////////////////
  template <typename T, size_t N>
  class vector_type final
  {

    static_assert(N > 0, "nonstd::vector_type error: refusing request to "
        "instantiate zero-length vector.");

    ///@name Local Types
    ///@{

    typedef T (& element_array)[N];
    typedef const T (& const_element_array)[N];

    ///@}
    ///@name Internal State
    ///@{

    T m_elements[N];  ///< Element storage.

    ///@}

    public:

      ///@name Construction
      ///@{

      /// Call T's default constructor for each element.
      vector_type() noexcept;

      /// Construct from an array of T.
      vector_type(const T (& _array)[N]) noexcept;

      /// Construct from an initializer list.
      vector_type(const std::initializer_list<T>&) noexcept;

      ///@}
      ///@name Conversion
      ///@{

      /// Convert to a C-style array reference.
      explicit operator element_array() noexcept;

      /// Convert to a const C-style array reference.
      explicit operator const_element_array() const noexcept;

      /// Convert to a vector of another numeric type.
      template <typename T2>
      explicit operator vector_type<T2, N>() const noexcept;

      ///@}
      ///@name Accessors
      ///@{

      /// Get the vector dimension.
      constexpr size_t size() const noexcept;

      /// Get a mutable element reference.
      T& operator[](const size_t _i) noexcept;

      /// Get a constant element reference.
      const T& operator[](const size_t _i) const noexcept;

      ///@}
      ///@name Iteration
      ///@{

      typedef T* iterator;
      typedef const T* const_iterator;

      iterator begin() noexcept;
      iterator end() noexcept;

      const_iterator begin() const noexcept;
      const_iterator end() const noexcept;

      ///@}
      ///@name Assignment
      ///@{
      /// Assign values to all elements of a vector.

      /// Set each element to zero.
      vector_type& zero() noexcept;

      /// Copy each element from another vector.
      vector_type& operator=(const vector_type& _v) noexcept;

      /// Copy each element from an array.
      vector_type& operator=(const T (& _array)[N]) noexcept;

      /// Copy each element from an initializer list.
      vector_type& operator=(const std::initializer_list<T>& _list) noexcept;

      ///@}
      ///@name Arithmetic Assignment
      ///@{
      /// Modify all elements in the vector.

      /// Vector sum assignment.
      vector_type& operator+=(const vector_type& _v) noexcept;

      /// Vector difference assignment.
      vector_type& operator-=(const vector_type& _v) noexcept;

      /// Scalar product assignment.
      vector_type& operator*=(const T _t) noexcept;

      /// Scalar quotient assignment.
      vector_type& operator/=(const T _t) noexcept;

      ///@}
      ///@name Arithmetic
      ///@{

      /// Vector addition.
      vector_type operator+(const vector_type& _v) const noexcept;

      /// Vector subtraction.
      vector_type operator-(const vector_type& _v) const noexcept;

      /// Unary negative.
      vector_type operator-() const noexcept;

      /// Scalar multiplication.
      vector_type operator*(const T _t) const noexcept;

      /// Scalar division.
      vector_type operator/(const T _t) const noexcept;

      /// Vector dot product.
      T operator*(const vector_type& _v) const noexcept;

      /// Vector cross product. Only supported for 3d vectors.
      vector_type operator%(const vector_type& _v) const noexcept;

      ///@}
      ///@name Equality
      ///@{

      /// Element-wise equality with 10-epsilon tolerance.
      bool operator==(const vector_type& _v) const noexcept;

      /// Element-wise inequality with 10-epsilon tolerance.
      bool operator!=(const vector_type& _v) const noexcept;

      ///@}
      ///@name Ordering
      ///@{

      /// Defines a weak ordering to allow sorting.
      bool operator<(const vector_type& _v) const noexcept;

      ///@}
      ///@name Magnitude Functions
      ///@{

      /// Return length squared.
      T mag_sqr() const noexcept;

      /// Return length.
      T mag() const noexcept;

      /// Return the unit vector in this direction.
      vector_type hat() const noexcept;

      /// Normalize this.
      vector_type& normalize() noexcept;

      /// Set length to _mag.
      vector_type& scale(const T _mag) noexcept;

      ///@}
      ///@name Projections
      ///@{

      /// Scalar projection of this on to _v.
      T comp(const vector_type& _v) const noexcept;

      /// Vector projection of this on to _v.
      vector_type proj(const vector_type& _v) const noexcept;

      /// Orthogonal projection of this on to _v.
      vector_type orth(const vector_type& _v) const noexcept;

      ///@}
      ///@name Utility Functions
      ///@{

      /// Modifies this to have a random direction and length _mag.
      vector_type& rand(const T _mag) noexcept;

      /// Rotate this about _axis by _radians. Only works for 3d vectors.
      vector_type& rotate(const vector_type& _axis, const T _radians) noexcept;

      /// Rotate this about an Euler vector _axis. Only works for 3d vectors.
      vector_type& rotate(const vector_type& _axis) noexcept;

      ///@}
      ///@name Generators
      ///@{
      /// Generate specific kinds of vectors.

      /// Generate a basis vector for index _i.
      static vector_type make_basis(const size_t _i) noexcept;

      ///@}

  };

  /*----------------------------- Construction -----------------------------*/

  template <typename T, size_t N>
  inline
  vector_type<T, N>::
  vector_type() noexcept
  {
    zero();
  }


  template <typename T, size_t N>
  inline
  vector_type<T, N>::
  vector_type(const T (& _array)[N]) noexcept
  {
    operator=(_array);
  }


  template <typename T, size_t N>
  inline
  vector_type<T, N>::
  vector_type(const std::initializer_list<T>& _list) noexcept
  {
    operator=(_list);
  }

  /*------------------------------- Conversion -------------------------------*/

  template <typename T, size_t N>
  vector_type<T, N>::
  operator element_array() noexcept
  {
    return m_elements;
  }


  template <typename T, size_t N>
  vector_type<T, N>::
  operator const_element_array() const noexcept
  {
    return m_elements;
  }


  template <typename T, size_t N>
  template <typename T2>
  vector_type<T, N>::
  operator vector_type<T2, N>() const noexcept
  {
    vector_type<T2, N> out;
    for(size_t i = 0; i < N; ++i)
      out[i] = static_cast<T2>(m_elements[i]);
    return out;
  }

  /*------------------------------- Accessors --------------------------------*/

  template <typename T, size_t N>
  inline constexpr
  size_t
  vector_type<T, N>::
  size() const noexcept
  {
    return N;
  }


  template <typename T, size_t N>
  inline
  T&
  vector_type<T, N>::
  operator[](const size_t _i) noexcept
  {
    return m_elements[_i];
  }


  template <typename T, size_t N>
  inline
  const T&
  vector_type<T, N>::
  operator[](const size_t _i) const noexcept
  {
    return m_elements[_i];
  }


  template <typename T, size_t N>
  inline
  typename vector_type<T, N>::iterator
  vector_type<T, N>::
  begin() noexcept
  {
    return m_elements;
  }


  template <typename T, size_t N>
  inline
  typename vector_type<T, N>::iterator
  vector_type<T, N>::
  end() noexcept
  {
    return m_elements + N;
  }


  template <typename T, size_t N>
  inline
  typename vector_type<T, N>::const_iterator
  vector_type<T, N>::
  begin() const noexcept
  {
    return m_elements;
  }


  template <typename T, size_t N>
  inline
  typename vector_type<T, N>::const_iterator
  vector_type<T, N>::
  end() const noexcept
  {
    return m_elements + N;
  }

  /*------------------------------ Assignment --------------------------------*/

  template <typename T, size_t N>
  inline
  vector_type<T, N>&
  vector_type<T, N>::
  zero() noexcept
  {
    for(size_t i = 0; i < N; ++i)
      m_elements[i] = T(0);
    return *this;
  }


  template <typename T, size_t N>
  inline
  vector_type<T, N>&
  vector_type<T, N>::
  operator=(const vector_type& _v) noexcept
  {
    for(size_t i = 0; i < N; ++i)
      m_elements[i] = _v[i];
    return *this;
  }


  template <typename T, size_t N>
  inline
  vector_type<T, N>&
  vector_type<T, N>::
  operator=(const T (& _array)[N]) noexcept
  {
    for(size_t i = 0; i < N; ++i)
      m_elements[i] = _array[i];
    return *this;
  }


  template <typename T, size_t N>
  inline
  vector_type<T, N>&
  vector_type<T, N>::
  operator=(const std::initializer_list<T>& _list) noexcept
  {
    assert_msg(N == _list.size(), "nonstd::vector_type error: can't assign a "
        "list of " + std::to_string(_list.size()) + " elements to a vector "
        "of size " + std::to_string(N) + "!");

    auto iter = _list.begin();
    for(size_t i = 0; i < N; ++i, ++iter)
      m_elements[i] = *iter;
    return *this;
  }

  /*------------------------ Arithmetic Assignment ---------------------------*/

  template <typename T, size_t N>
  inline
  vector_type<T, N>&
  vector_type<T, N>::
  operator+=(const vector_type& _v) noexcept
  {
    for(size_t i = 0; i < N; ++i)
      m_elements[i] += _v[i];
    return *this;
  }


  template <typename T, size_t N>
  inline
  vector_type<T, N>&
  vector_type<T, N>::
  operator-=(const vector_type& _v) noexcept
  {
    for(size_t i = 0; i < N; ++i)
      m_elements[i] -= _v[i];
    return *this;
  }


  template <typename T, size_t N>
  inline
  vector_type<T, N>&
  vector_type<T, N>::
  operator*=(const T _t) noexcept
  {
    for(size_t i = 0; i < N; ++i)
      m_elements[i] *= _t;
    return *this;
  }


  template <typename T, size_t N>
  inline
  vector_type<T, N>&
  vector_type<T, N>::
  operator/=(const T _t) noexcept
  {
    for(size_t i = 0; i < N; ++i)
      m_elements[i] /= _t;
    return *this;
  }

  /*------------------------------- Arithmetic -------------------------------*/

  template <typename T, size_t N>
  vector_type<T, N>
  vector_type<T, N>::
  operator+(const vector_type& _v) const noexcept
  {
    return vector_type(*this) += _v;
  }


  template <typename T, size_t N>
  vector_type<T, N>
  vector_type<T, N>::
  operator-(const vector_type& _v) const noexcept
  {
    return vector_type(*this) -= _v;
  }


  template <typename T, size_t N>
  vector_type<T, N>
  vector_type<T, N>::
  operator-() const noexcept
  {
    vector_type negative(*this);
    for(size_t i = 0; i < size(); ++i)
      negative[i] = -negative[i];
    return negative;
  }


  template <typename T, size_t N>
  vector_type<T, N>
  vector_type<T, N>::
  operator*(const T _t) const noexcept
  {
    return vector_type(*this) *= _t;
  }


  template <typename T, size_t N>
  vector_type<T, N>
  vector_type<T, N>::
  operator/(const T _t) const noexcept
  {
    return vector_type(*this) /= _t;
  }


  template <typename T, size_t N>
  T
  vector_type<T, N>::
  operator*(const vector_type& _v) const noexcept
  {
    T product = T(0);
    for(size_t i = 0; i < N; ++i)
      product += m_elements[i] * _v[i];
    return product;
  }


  template <typename T, size_t N>
  vector_type<T, N>
  vector_type<T, N>::
  operator%(const vector_type& _v) const noexcept
  {
    static_assert(N == 3, "nonstd::vector_type::operator\% error: cross-product "
        "is not defined for fixed-size vectors of size != 3.");

    return vector_type{m_elements[1] * _v[2] - m_elements[2] * _v[1],
                       m_elements[2] * _v[0] - m_elements[0] * _v[2],
                       m_elements[0] * _v[1] - m_elements[1] * _v[0]};
  }

  /*-------------------------------- Equality --------------------------------*/

  template <typename T, size_t N>
  bool
  vector_type<T, N>::
  operator==(const vector_type& _v) const noexcept
  {
    for(size_t i = 0; i < N; ++i)
      if(!approx(m_elements[i], _v[i]))
        return false;
    return true;
  }


  template <typename T, size_t N>
  bool
  vector_type<T, N>::
  operator!=(const vector_type& _v) const noexcept
  {
    return !(*this == _v);
  }

  /*-------------------------------- Ordering --------------------------------*/

  template <typename T, size_t N>
  bool
  vector_type<T, N>::
  operator<(const vector_type& _v) const noexcept
  {
    for(size_t i = 0; i < N; ++i) {
      if(m_elements[i] < _v[i])
        return true;
      else if(_v[i] < m_elements[i])
        return false;
    }
    return false;
  }

  /*--------------------------- Magnitude Functions --------------------------*/

  template <typename T, size_t N>
  inline
  T
  vector_type<T, N>::
  mag_sqr() const noexcept
  {
    return (*this) * (*this);
  }


  template <typename T, size_t N>
  inline
  T
  vector_type<T, N>::
  mag() const noexcept
  {
    return sqrt(mag_sqr());
  }


  template <typename T, size_t N>
  vector_type<T, N>
  vector_type<T, N>::
  hat() const noexcept
  {
    return vector_type(*this).normalize();
  }


  template <typename T, size_t N>
  vector_type<T, N>&
  vector_type<T, N>::
  normalize() noexcept
  {
    T magnitude = this->mag();
    if(magnitude < std::numeric_limits<T>::epsilon())
      return this->zero();
    return *this /= magnitude;
  }


  template <typename T, size_t N>
  vector_type<T, N>&
  vector_type<T, N>::
  scale(const T _mag) noexcept
  {
    T magnitude = this->mag();
    if(magnitude < std::numeric_limits<T>::epsilon() ||
        _mag < std::numeric_limits<T>::epsilon())
      return this->zero();
    return *this *= (_mag / magnitude);
  }

  /*------------------------------ Projections -------------------------------*/

  template <typename T, size_t N>
  T
  vector_type<T, N>::
  comp(const vector_type& _v) const noexcept
  {
    return *this * _v.hat();
  }


  template <typename T, size_t N>
  vector_type<T, N>
  vector_type<T, N>::
  proj(const vector_type& _v) const noexcept
  {
    return _v * ((*this * _v) / _v.mag_sqr());
  }


  template <typename T, size_t N>
  vector_type<T, N>
  vector_type<T, N>::
  orth(const vector_type& _v) const noexcept
  {
    return *this - this->proj(_v);
  }

  /*---------------------------- Utility Functions ---------------------------*/

  template <typename T, size_t N>
  vector_type<T, N>&
  vector_type<T, N>::
  rand(const T _mag) noexcept
  {
    for(size_t i = 0; i < size(); ++i)
      m_elements[i] = 2. * drand48() - 1.;
    return this->scale(_mag);
  }


  template <typename T, size_t N>
  vector_type<T, N>&
  vector_type<T, N>::
  rotate(const vector_type& _axis, const T _radians) noexcept
  {
    static_assert(N == 3, "nonstd::vector_type::rotate error: not supported for "
        "vectors of fixed size != 3.");

    const vector_type pivot = this->proj(_axis);
    const vector_type arm = *this - pivot;
    const vector_type xPrime = std::cos(_radians) * arm;
    const vector_type yPrime = std::sin(_radians) * (_axis.hat() % arm);
    return *this = pivot + xPrime + yPrime;
  }


  template <typename T, size_t N>
  vector_type<T, N>&
  vector_type<T, N>::
  rotate(const vector_type& _axis) noexcept
  {
    return rotate(_axis, _axis.mag());
  }

  /*------------------------------- Generators -------------------------------*/

  template <typename T, size_t N>
  vector_type<T, N>
  vector_type<T, N>::
  make_basis(const size_t _i) noexcept
  {
    vector_type basis;
    basis[_i] = T(1);
    return basis;
  }

  /*------------------------- Non-member Functions ---------------------------*/

  /// Scalar multiplication with the scalar first. _s is a long double instead
  /// of a template parameter to prevent MatrixType commutativity.
  template<typename T, size_t N>
  inline
  const vector_type<T, N>
  operator*(const long double& _s, const vector_type<T, N>& _v) noexcept
  {
    return _v * _s;
  }

  /*--------------------------------------------------------------------------*/

}


namespace std {

  /// Print _v in the format (x, y, z, ...).
  template<typename T, size_t N>
  inline
  ostream&
  operator<<(ostream& _os, const nonstd::vector_type<T, N>& _v) {
    for(size_t i = 0; i < _v.size(); ++i)
      _os << (i == 0 ? "(" : "") << _v[i] << (i == _v.size() - 1 ? ")" : ", ");
    return _os;
  }


  /// Read in a vector from an istream.
  template<typename T, size_t N>
  inline
  istream&
  operator>>(istream& _is, nonstd::vector_type<T, N>& _v) {
    for(size_t i = 0; i < _v.size(); ++i)
      _is >> _v[i];
    return _is;
  }


  /// Convert a vector to a std string.
  template <typename T, size_t N>
  string
  to_string(const nonstd::vector_type<T, N>& _v)
  {
    ostringstream oss;
    oss << _v << flush;
    return oss.str();
  }

}

#endif
