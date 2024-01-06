#ifndef NONSTD_MATRIX_TYPE_H_
#define NONSTD_MATRIX_TYPE_H_

///@warning Including this file will remove the 'minor' macro that is defined by
///         GCC from the rest of the translation unit. Uncomment the preseve
///         define here to save the macro minor and instead make
///         matrix_type::minor unavailable outside this file. The better
///         solution would be to use 'gnu_dev_minor' instead of the old name
///         'minor' for the macro.
//#define preserve_minor_macro

#include <cmath>
#include <cstddef>
#include <limits>
#include <iostream>
#include <sstream>
#include <type_traits>

#ifdef minor
  #ifdef preserve_minor_macro
    #pragma push_macro("minor")
  #endif
  #undef minor
#endif

#include "nonstd/numerics.h"
#include "nonstd/runtime.h"
#include "nonstd/vector.h"

namespace nonstd {

  //////////////////////////////////////////////////////////////////////////////
  /// A (Rows)x(Cols) matrix of T's.
  //////////////////////////////////////////////////////////////////////////////
  template <typename T, size_t Rows, size_t Cols>
  class matrix_type
  {

    protected:

      ///@name Internal State
      ///@{

      T m_elements[Rows][Cols];                     ///< Element storage.
      static constexpr bool m_square{Rows == Cols}; ///< Allow square matrix ops.

      ///@}
      ///@name Local Types
      ///@{

      typedef T (&matrix_array_2d)[Rows][Cols];
      typedef const T (&const_matrix_array_2d)[Rows][Cols];

      ///@}

    public:

      ///@name Construction
      ///@{

      matrix_type() noexcept;
      matrix_type(const matrix_type&) noexcept;
      matrix_type(const T (& _array)[Rows][Cols]) noexcept;
      matrix_type(const T (& _array)[Rows * Cols]) noexcept;

      virtual ~matrix_type() = default;

      ///@}
      ///@name Conversion to C-Style Arrays
      ///@{

      operator matrix_array_2d() noexcept;
      operator const_matrix_array_2d() const noexcept;

      ///@}
      ///@name Accessors
      ///@{

      constexpr size_t size() const noexcept;

      constexpr size_t rows() const noexcept;
      constexpr size_t cols() const noexcept;

      T& operator()(const size_t, const size_t) noexcept;
      const T& operator()(const size_t, const size_t) const noexcept;

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

      /// Call T's default constructor for each element.
      matrix_type& zero() noexcept;

      /// Copy each element from _other.
      matrix_type& operator=(const matrix_type& _other) noexcept;

      /// Copy each element from a 2D _array.
      matrix_type& operator=(const T (& _array)[Rows][Cols]) noexcept;

      /// Copy each element from a 1D _array.
      matrix_type& operator=(const T (& _array)[Rows * Cols]) noexcept;

      ///@}
      ///@name Arithmetic Assignment
      ///@{

      /// Matrix sum assignment.
      matrix_type& operator+=(const matrix_type& _other) noexcept;

      /// Matrix difference assignment.
      matrix_type& operator-=(const matrix_type& _other) noexcept;

      /// Scalar product assignment.
      matrix_type& operator*=(const T& _s) noexcept;

      /// Scalar quotient assignment.
      matrix_type& operator/=(const T& _s) noexcept;

      ///@}
      ///@name Arithmetic
      ///@{

      /// Matrix addition.
      matrix_type operator+(const matrix_type& _other) const noexcept;

      /// Matrix subtraction.
      matrix_type operator-(const matrix_type& _other) const noexcept;

      /// Matrix unary subtraction.
      matrix_type operator-() const noexcept;

      /// Scalar multiplication.
      matrix_type operator*(const T& _s) const noexcept;

      /// Scalar division.
      matrix_type operator/(const T& _s) const noexcept;

      /// Matrix multiplication of (this)[Rows x Cols] by (_other)[Cols x X].
      template <size_t X>
      matrix_type<T, Rows, X>
      operator*(const matrix_type<T, Cols, X>& _other) const noexcept;

      /// Vector multiplication.
      template <typename T2>
      vector_type<T2, Rows>
      operator*(const vector_type<T2, Cols>&) const noexcept;

      /// Compute the transpose.
      matrix_type<T, Cols, Rows> transpose() const noexcept;

      ///@}
      ///@name Equality
      ///@{

      /// Element-wise equality.
      bool operator==(const matrix_type& _other) const noexcept;

      /// Element-wise inequality.
      bool operator!=(const matrix_type& _other) const noexcept;

      ///@}
      ///@name Display
      ///@{

      /// Print this in block format.
      void print(std::ostream& _os) const noexcept;

      ///@}
      ///@name Square Matrix Types
      ///@{

      typedef typename std::conditional<m_square && Rows != 1,
              matrix_type<T, Rows - 1, Cols - 1>,
              T>::type minor_type;

      ///@}
      ///@name Square Matrix Functions
      ///@{
      /// These functions are enabled only for square matrices.

      /// Set this to the identity matrix.
      matrix_type& identity() noexcept;

      /// Compute the determinant.
      T det() const noexcept;

      /// Compute the inverse.
      matrix_type inverse() const noexcept;

      /// Compute the cofactor.
      matrix_type cofactor() const noexcept;

      /// Compute the transposed cofactor.
      matrix_type cofactor_transpose() const noexcept;

      /// Compute the minor of this by removing row _i and column _j.
      minor_type minor(const size_t _i, const size_t _j) const noexcept;

      ///@}
  };


  /*-------------------------------- Construction ----------------------------*/

  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>::
  matrix_type() noexcept
  {
    zero();
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>::
  matrix_type(const matrix_type& _other) noexcept
  {
    operator=(_other);
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>::
  matrix_type(const T (& _array)[Rows][Cols]) noexcept
  {
    operator=(_array);
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>::
  matrix_type(const T (& _array)[Rows * Cols]) noexcept
  {
    operator=(_array);
  }

  /*-------------------------------- Conversion ------------------------------*/

  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>::
  operator matrix_array_2d() noexcept
  {
    return m_elements;
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>::
  operator const_matrix_array_2d() const noexcept
  {
    return m_elements;
  }

  /*--------------------------------- Accessors ------------------------------*/

  template <typename T, size_t Rows, size_t Cols>
  inline constexpr
  size_t
  matrix_type<T, Rows, Cols>::
  size() const noexcept
  {
    return Rows * Cols;
  }


  template <typename T, size_t Rows, size_t Cols>
  inline constexpr
  size_t
  matrix_type<T, Rows, Cols>::
  rows() const noexcept
  {
    return Rows;
  }


  template <typename T, size_t Rows, size_t Cols>
  inline constexpr
  size_t
  matrix_type<T, Rows, Cols>::
  cols() const noexcept
  {
    return Cols;
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  T&
  matrix_type<T, Rows, Cols>::
  operator()(const size_t _i, const size_t _j) noexcept
  {
    return m_elements[_i][_j];
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  const T&
  matrix_type<T, Rows, Cols>::
  operator()(const size_t _i, const size_t _j) const noexcept
  {
    return m_elements[_i][_j];
  }

  /*--------------------------------- Iteration ------------------------------*/

  template <typename T, size_t Rows, size_t Cols>
  inline
  typename matrix_type<T, Rows, Cols>::iterator
  matrix_type<T, Rows, Cols>::
  begin() noexcept
  {
    return &m_elements[0][0];
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  typename matrix_type<T, Rows, Cols>::iterator
  matrix_type<T, Rows, Cols>::
  end() noexcept
  {
    return &m_elements[0][0] + (Rows * Cols);
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  typename matrix_type<T, Rows, Cols>::const_iterator
  matrix_type<T, Rows, Cols>::
  begin() const noexcept
  {
    return &m_elements[0][0];
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  typename matrix_type<T, Rows, Cols>::const_iterator
  matrix_type<T, Rows, Cols>::
  end() const noexcept
  {
    return &m_elements[0][0] + (Rows * Cols);
  }

  /*-------------------------------- Assignment ------------------------------*/

  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>&
  matrix_type<T, Rows, Cols>::
  zero() noexcept
  {
    for(auto i = begin(); i != end(); ++i)
      *i = T(0);
    return *this;
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>&
  matrix_type<T, Rows, Cols>::
  operator=(const matrix_type& _other) noexcept {
    for(auto i = begin(), o = _other.begin(); i != end(); ++i, ++o)
      *i = *o;
    return *this;
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>&
  matrix_type<T, Rows, Cols>::
  operator=(const T (& _array)[Rows][Cols]) noexcept {
    const T* a = &_array[0][0];
    for(auto i = begin(); i != end(); ++i, ++a)
      *i = *a;
    return *this;
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>&
  matrix_type<T, Rows, Cols>::
  operator=(const T (& _array)[Rows * Cols]) noexcept {
    const T* a = &_array[0];
    for(auto i = begin(); i != end(); ++i, ++a)
      *i = *a;
    return *this;
  }

  /*--------------------------- Arithmetic Assignment ------------------------*/

  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>&
  matrix_type<T, Rows, Cols>::
  operator+=(const matrix_type& _other) noexcept
  {
    for(auto i = begin(), o = _other.begin(); i != end(); ++i, ++o)
      *i += *o;
    return *this;
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>&
  matrix_type<T, Rows, Cols>::
  operator-=(const matrix_type& _other) noexcept
  {
    for(auto i = begin(), o = _other.begin(); i != end(); ++i, ++o)
      *i -= *o;
    return *this;
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>&
  matrix_type<T, Rows, Cols>::
  operator*=(const T& _s) noexcept
  {
    for(auto i = begin(); i != end(); ++i)
      *i *= _s;
    return *this;
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>&
  matrix_type<T, Rows, Cols>::
  operator/=(const T& _s) noexcept
  {
    for(auto i = begin(); i != end(); ++i)
      *i /= _s;
    return *this;
  }

  /*-------------------------------- Arithmetic ------------------------------*/

  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>
  matrix_type<T, Rows, Cols>::
  operator+(const matrix_type& _other) const noexcept
  {
    matrix_type sum(*this);
    return sum += _other;
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>
  matrix_type<T, Rows, Cols>::
  operator-(const matrix_type& _other) const noexcept
  {
    matrix_type difference(*this);
    return difference -= _other;
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>
  matrix_type<T, Rows, Cols>::
  operator-() const noexcept
  {
    return (*this * T(-1));
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>
  matrix_type<T, Rows, Cols>::
  operator*(const T& _s) const noexcept
  {
    matrix_type product(*this);
    return product *= _s;
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>
  matrix_type<T, Rows, Cols>::
  operator/(const T& _s) const noexcept
  {
    matrix_type quotient(*this);
    return quotient /= _s;
  }


  template <typename T, size_t Rows, size_t Cols>
  template <size_t X>
  matrix_type<T, Rows, X>
  matrix_type<T, Rows, Cols>::
  operator*(const matrix_type<T, Cols, X>& _other) const noexcept
  {
    matrix_type<T, Rows, X> product;
    for(size_t i = 0; i < Rows; ++i)
      for(size_t j = 0; j < X; ++j)
        for(size_t k = 0; k < Cols; ++k)
          product(i,j) += m_elements[i][k] * _other(k,j);
    return product;
  }


  template <typename T, size_t Rows, size_t Cols>
  template <typename T2>
  vector_type<T2, Rows>
  matrix_type<T, Rows, Cols>::
  operator*(const vector_type<T2, Cols>& _v) const noexcept
  {
    vector_type<T2, Rows> product;
    for(size_t i = 0; i < Rows; ++i)
      for(size_t j = 0; j < Cols; ++j)
        product[i] += m_elements[i][j] * _v[j];
    return product;
  }


  template <typename T, size_t Rows, size_t Cols>
  matrix_type<T, Cols, Rows>
  matrix_type<T, Rows, Cols>::
  transpose() const noexcept
  {
    matrix_type<T, Cols, Rows> trans;
    for(size_t i = 0; i < Rows; ++i)
      for(size_t j = 0; j < Cols; ++j)
        trans(j,i) = this->m_elements[i][j];
    return trans;
  }

  /*------------------------------- Equality ---------------------------------*/

  template <typename T, size_t Rows, size_t Cols>
  bool
  matrix_type<T, Rows, Cols>::
  operator==(const matrix_type& _other) const noexcept
  {
    for(auto i = begin(), o = _other.begin(); i != end(); ++i, ++o)
      if(!approx(*i, *o))
        return false;
    return true;
  }


  template <typename T, size_t Rows, size_t Cols>
  bool
  matrix_type<T, Rows, Cols>::
  operator!=(const matrix_type& _other) const noexcept
  {
    return !(*this == _other);
  }

  /*------------------------------- Display ----------------------------------*/

  template <typename T, size_t Rows, size_t Cols>
  void
  matrix_type<T, Rows, Cols>::
  print(std::ostream& _os) const noexcept
  {
    for(size_t i = 0; i < Rows; ++i) {
      _os << "[";
      for(size_t j = 0; j < Cols - 1; ++j)
        _os << m_elements[i][j] << ", ";
      _os << m_elements[i][Cols - 1] << "]" << std::endl;
    }
  }

  /*-------------------------- Square Matrix Functions -----------------------*/

  template <typename T, size_t Rows, size_t Cols>
  matrix_type<T, Rows, Cols>&
  matrix_type<T, Rows, Cols>::
  identity() noexcept
  {
    // Refuse to compile for non-square matrices.
    constexpr typename std::enable_if<m_square, size_t>::type N = Rows;

    zero();
    for(size_t i = 0; i < N; ++i)
      m_elements[i][i] = T(1);
    return *this;
  }


  template <typename T, size_t Rows, size_t Cols>
  T
  matrix_type<T, Rows, Cols>::
  det() const noexcept
  {
    // Refuse to compile for non-square matrices.
    constexpr typename std::enable_if<m_square, size_t>::type N = Rows;

    // 1x1 Optimization
    if(N == 1)
      return m_elements[0][0];

    // 2x2 Optimization
    if(N == 2)
      return m_elements[0][0] * m_elements[1][1] -
             m_elements[1][0] * m_elements[0][1];

    T d(0);
    for(size_t j = 0; j < N; ++j)
      d += pow(-1, j) * m_elements[0][j] * minor(0,j).det();
    return d;
  }


  template <typename T, size_t Rows, size_t Cols>
  matrix_type<T, Rows, Cols>
  matrix_type<T, Rows, Cols>::
  inverse() const noexcept
  {
    // Refuse to compile for non-square matrices.
    constexpr typename std::enable_if<m_square, size_t>::type N = Rows;

    // Assert this is invertible
    T d = this->det();
    assert_msg(!approx(d, 0), "matrix_type::inverse() error: attempted to "
        "invert a matrix with det = 0.");

    // 1x1 special case
    if(Rows == 1)
      return matrix_type(T(1) / d);

    return this->cofactor_transpose() / d;
  }


  template <typename T, size_t Rows, size_t Cols>
  matrix_type<T, Rows, Cols>
  matrix_type<T, Rows, Cols>::
  cofactor() const noexcept
  {
    // Refuse to compile for non-square matrices.
    constexpr typename std::enable_if<m_square, size_t>::type N = Rows;

    matrix_type cf;
    for(size_t i = 0; i < N; ++i)
      for(size_t j = 0; j < N; ++j)
        cf(i,j) = pow(-1, i + j) * minor(i,j).det();
    return cf;
  }


  template <typename T, size_t Rows, size_t Cols>
  matrix_type<T, Rows, Cols>
  matrix_type<T, Rows, Cols>::
  cofactor_transpose() const noexcept
  {
    // Refuse to compile for non-square matrices.
    constexpr typename std::enable_if<m_square, size_t>::type N = Rows;

    matrix_type cft;
    for(size_t i = 0; i < N; ++i)
      for(size_t j = 0; j < N; ++j)
        cft(j,i) = pow(-1, i + j) * minor(i,j).det();
    return cft;
  }


  template <typename T, size_t Rows, size_t Cols>
  typename matrix_type<T, Rows, Cols>::minor_type
  matrix_type<T, Rows, Cols>::
  minor(const size_t _i, const size_t _j) const noexcept
  {
    // Refuse to compile for non-square matrices or 1x1's.
    constexpr typename std::enable_if<m_square && Rows != 1, size_t>::type N =
        Rows;

    // 2x2 Optimization
    if(N == 2)
      return minor_type(this->m_elements[1 - _i][1 - _j]);

    minor_type m;
    for(size_t i = 0; i < N; ++i) {
      // Skip row _i. m(i, j) -> m(i-1, j) from now on
      if(i == _i) continue;

      for(size_t j = 0; j < N; ++j) {
        // Skip col _j. m(i, j) -> m(i, j-1) until the end of the row
        if(j == _j) continue;

        // Copy this element to the minor
        m(i - (i > _i ? 1 : 0), j - (j > _j ? 1 : 0)) = m_elements[i][j];
      }
    }
    return m;
  }

  /*------------------------- Non-member Functions ---------------------------*/

  /// Scalar multiplication w/ scalar _s first. _s is a long double instead
  /// of a template parameter to prevent matrix_type commutativity.
  template <typename T, size_t Rows, size_t Cols>
  inline
  matrix_type<T, Rows, Cols>
  operator*(const double _s, const matrix_type<T, Rows, Cols>& _m) noexcept
  {
    return _m * _s;
  }

  /*--------------------------------------------------------------------------*/

}


namespace std {

  /*-------------------------- Display Functions -----------------------------*/

  /// Prints a matrix A in [[A00, A01], [A10, A11]] format.
  template <typename T, size_t Rows, size_t Cols>
  inline
  ostream&
  operator<<(ostream& _os, const nonstd::matrix_type<T, Rows, Cols>& _m)
  {
    _os << "[";
    for(size_t i = 0; i < Rows; ++i) {
      _os << "[";
      for(size_t j = 0; j < Cols - 1; ++j)
        _os << _m(i,j) << ", ";
      _os << _m(i, Cols - 1) << "]";
      if(i != Rows - 1)
        _os << ", ";
    }
    _os << "]";
    return _os;
  }


  template <typename T, size_t Rows, size_t Cols>
  inline
  string
  to_string(const nonstd::matrix_type<T, Rows, Cols>& _m)
  {
    ostringstream oss;
    oss.clear();
    oss << _m << flush;
    return oss.str();
  }

  /*--------------------------------------------------------------------------*/

}

#ifdef preserve_minor_macro
  #pragma pop_macro("minor")
#endif
#endif
