#ifndef MATRIX_H_
#define MATRIX_H_

#include "Basic.h"
#include "Vector.h"

namespace mathtool {

  template<size_t N, size_t M = N>
    class Matrix {
      public:
        Matrix() {
          for(size_t i = 0; i<N; ++i)
            for(size_t j = 0; j<M; ++j)
              m_m[i][j] = 0.0;
        }
        Matrix(const double _m[N][M]){
          for(size_t i = 0; i<N; ++i)
            for(size_t j = 0; j<M; ++j)
              m_m[i][j] = _m[i][j];
        }
        Matrix(const Matrix& _m) {
          for(size_t i = 0; i<N; ++i)
            for(size_t j = 0; j<M; ++j)
              m_m[i][j] = _m[i][j];
        }

        //assignment
        Matrix& operator=(const Matrix& _m) {
          for(size_t i = 0; i<N; ++i)
            for(size_t j = 0; j<M; ++j)
              m_m[i][j] = _m[i][j];
          return *this;
        }

        //access
        typedef double (&mat)[N][M];
        typedef const double (&cmat)[N][M];
        operator mat() {return m_m;}
        operator cmat() const {return m_m;}

        //equality
        bool operator==(const Matrix& _m) const {
          for(size_t i = 0; i<N; ++i)
            for(size_t j = 0; j<M; ++j)
              if(m_m[i][j] != _m[i][j])
                return false;
          return true;
        }
        //inequality
        bool operator!=(const Matrix& _m) const {
          return !(*this == _m);
        }

        //self addition
        Matrix& operator+=(const Matrix& _m) {
          for(size_t i = 0; i<N; ++i)
            for(size_t j = 0; j<M; ++j)
              m_m[i][j] += _m[i][j];
          return *this;
        }
        //self subtraction
        Matrix& operator-=(const Matrix& _m) {
          for(size_t i = 0; i<N; ++i)
            for(size_t j = 0; j<M; ++j)
              m_m[i][j] -= _m[i][j];
          return *this;
        }
        //self multiplication
        Matrix& operator*=(double _d) {
          for(size_t i = 0; i<N; ++i)
            for(size_t j = 0; j<M; ++j)
              m_m[i][j] *= _d;
          return *this;
        }
        //self division
        Matrix& operator/=(double _d) {
          for(size_t i = 0; i<N; ++i)
            for(size_t j = 0; j<M; ++j)
              m_m[i][j] /= _d;
          return *this;
        }

        //addition
        Matrix operator+(const Matrix& _m) const {
          Matrix m(*this);
          m += _m;
          return m;
        }
        //subtraction
        Matrix operator-(const Matrix& _m) const {
          Matrix m(*this);
          m -= _m;
          return m;
        }
        //scalar multiplication
        Matrix operator*(double _d) {
          Matrix m(*this);
          m *= _d;
          return m;
        }
        //scalar division
        Matrix operator/(double _d) {
          Matrix m(*this);
          m /= _d;
          return m;
        }

        Matrix<M, N> transpose() const {
          Matrix<M, N> m;
          for(size_t i = 0; i<N; ++i)
            for(size_t j = 0; j<M; ++j)
              m[j][i] = m_m[i][j];
          return m;
        }

      private:
        double m_m[N][M];
    };

  //////////////////////////////////////////////////////////////////////////////
  // Matrix utilities and operators
  //////////////////////////////////////////////////////////////////////////////

  //Grab identity matrix
  template<size_t N>
    inline void identity(Matrix<N>& _m) {
      for(size_t i = 0; i<N; ++i)
        for(size_t j = 0; j<N; ++j)
          if(i == j) _m[i][j] = 1.0;
          else _m[i][j] = 0.0;
    }

  //get a 2x2 matrix from 4 doubles
  inline void getMatrix2x2(Matrix<2>& _m,
      double _m00, double _m01,
      double _m10, double _m11) {
    _m[0][0] = _m00; _m[0][1] = _m01;
    _m[1][0] = _m10; _m[1][1] = _m11;
  }

  //get a 3x3 matrix from 9 doubles
  inline void getMatrix3x3(Matrix<3>& _m,
      double _m00, double _m01, double _m02,
      double _m10, double _m11, double _m12,
      double _m20, double _m21, double _m22) {
    _m[0][0] = _m00; _m[0][1] = _m01; _m[0][2] = _m02;
    _m[1][0] = _m10; _m[1][1] = _m11; _m[1][2] = _m12;
    _m[2][0] = _m20; _m[2][1] = _m21; _m[2][2] = _m22;
  }

  //get a 4x4 matrix from 16 doubles
  inline void getMatrix4x4(Matrix<4>& _m,
      double _m00, double _m01, double _m02, double _m03,
      double _m10, double _m11, double _m12, double _m13,
      double _m20, double _m21, double _m22, double _m23,
      double _m30, double _m31, double _m32, double _m33) {
    _m[0][0] = _m00; _m[0][1] = _m01; _m[0][2] = _m02; _m[0][3] = _m03;
    _m[1][0] = _m10; _m[1][1] = _m11; _m[1][2] = _m12; _m[1][3] = _m13;
    _m[2][0] = _m20; _m[2][1] = _m21; _m[2][2] = _m22; _m[2][3] = _m23;
    _m[3][0] = _m30; _m[3][1] = _m31; _m[3][2] = _m32; _m[3][3] = _m33;
  }

  //vector multiplication
  template<size_t N, size_t M>
    Vector<double, N> operator*(const Matrix<N, M>& _m, const Vector<double, M>& _v) {
      Vector<double, N> v;
      for(size_t i = 0; i<N; ++i)
        for(size_t j = 0; j<M; ++j)
          v[i] += _m[i][j]*_v[j];
      return v;
    }
  template<size_t N, size_t M>
    Vector<double, M> operator*(const Vector<double, N>& _v, const Matrix<N, M>& _m) {
      Vector<double, M> v;
      for(size_t i = 0; i<N; ++i)
        for(size_t j = 0; j<M; ++j)
          v[j] += _m[i][j]*_v[i];
      return v;
    }

  //matrix multiplication
  template<size_t N, size_t K, size_t M>
    Matrix<N, M> operator*(const Matrix<N, K>& _m1, const Matrix<K, M>& _m2){
      Matrix<N, M> m;
      for(size_t i = 0; i<N; ++i)
        for(size_t j = 0; j<M; ++j)
          for(size_t k = 0; k<K; ++k)
            m[i][j] += _m1[i][k] * _m2[k][j];
      return m;
    }

  //vector outer product
  template<size_t N, size_t M>
    Matrix<N, M> operator&(const Vector<double, N>& _v1, const Vector<double, M>& _v2){
      Matrix<N, M> m;
      for(size_t i = 0; i<N; ++i)
        for(size_t j = 0; j<M; ++j)
          m[i][j] = _v1[i]*_v2[j];
      return m;
    }

  template<size_t N>
    double trace(const Matrix<N,N>& _m){
      double t = 0;
      for(size_t i = 0; i<N; ++i)
        t += _m[i][i];
      return t;
    }

  //output a NxM matrix
  template<size_t N, size_t M>
    std::ostream& operator<<(std::ostream& _os, const Matrix<N, M>& _m){
      _os << std::endl << "[ " << std::endl;
      for(size_t i = 0; i<N; ++i) {
        _os << "[ ";
        for(size_t j = 0; j<M; ++j) {
          _os << _m[i][j] << ", ";
        }
        _os << "];" << std::endl;
      }
      return _os << "]" << std::endl;
    }

  //////////////////////////////////////////////////////////////////////////////
  // Common Typedefs of matrices
  //////////////////////////////////////////////////////////////////////////////
  typedef Matrix<2> Matrix2x2;
  typedef Matrix<3> Matrix3x3;
  typedef Matrix<4> Matrix4x4;

  //matrix inverse for 3x3
  inline Matrix3x3 inverse(const Matrix3x3& _m) {
    Matrix3x3 m;
    const double det = _m[0][0]*(_m[1][1]*_m[2][2] - _m[2][1]*_m[1][2])
      -_m[0][1]*(_m[1][0]*_m[2][2]-_m[1][2]*_m[2][0])
      +_m[0][2]*(_m[1][0]*_m[2][1]-_m[1][1]*_m[2][0]);
    const double invdet = 1.0/det;
    m[0][0] =  (_m[1][1]*_m[2][2] - _m[2][1]*_m[1][2]) * invdet;
    m[1][0] = -(_m[0][1]*_m[2][2] - _m[0][2]*_m[2][1]) * invdet;
    m[2][0] =  (_m[0][1]*_m[1][2] - _m[0][2]*_m[1][1]) * invdet;
    m[0][1] = -(_m[1][0]*_m[2][2] - _m[1][2]*_m[2][0]) * invdet;
    m[1][1] =  (_m[0][0]*_m[2][2] - _m[0][2]*_m[2][0]) * invdet;
    m[2][1] = -(_m[0][0]*_m[1][2] - _m[1][0]*_m[0][2]) * invdet;
    m[0][2] =  (_m[1][0]*_m[2][1] - _m[2][0]*_m[1][1]) * invdet;
    m[1][2] = -(_m[0][0]*_m[2][1] - _m[2][0]*_m[0][1]) * invdet;
    m[2][2] =  (_m[0][0]*_m[1][1] - _m[1][0]*_m[0][1]) * invdet;
    return m;
  };
}

#endif
