// -*- c++ -*-
//
/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef MTL_ALGO_TEST_H
#define MTL_ALGO_TEST_H

#include "mtl/mtl.h"
#include "mtl/dense1D.h"
#include "fill_matrix.h"

using mtl::dense1D;

template <class Matrix>
bool
mat_algo_test(Matrix& A, std::string test_name)
{
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  typename Matrix::iterator i;
  typename Matrix::OneD::iterator j;
  T s;
  T sum, norm;
  bool success = true;

  mtl::insert_zero_matrix(A);

  typename mtl::matrix<T>::type C(A.nrows(), A.ncols());
  mtl::zero_matrix(C);

  // set
  mtl::set_value(A, T(5));

  for (i = A.begin(); i != A.end(); ++i)
    for (j = (*i).begin(); j != (*i).end(); ++j)
      if (*j != T(5)) {
        std::cerr << "**** FAILED: (matrix set) " << test_name.c_str() << " ****" << std::endl;
        success = false;
        break;
      }

  // scale
  mtl::scale(A, T(5));

  for (i = A.begin(); i != A.end(); ++i)
    for (j = (*i).begin(); j != (*i).end(); ++j)
      if (*j != T(25)) {
        std::cerr << "**** FAILED: (matrix set) " << test_name.c_str() << " ****" << std::endl;
        success = false;      
        break;
      }

  matrix_fill(A);

  mtl::copy(A, C);

  // one_norm
  {
    Int i=0, j;
    s = mtl::one_norm(A);

    if (C.ncols() > 0) {
      i = 0; sum = T();
      for (j = 0; j < Int(C.nrows()); ++j)
        sum += MTL_ABS(C(j,i));
      norm = sum;
      ++i;
    }
    
    for (; i < Int(C.ncols()); ++i) {
      sum = T();
      for (j = 0; j < Int(C.nrows()); ++j)
        sum += MTL_ABS(C(j,i));
      norm = MTL_MAX(MTL_ABS(norm), MTL_ABS(sum));
    }
    
    if (s != norm) {
      std::cout << test_name.c_str() << " failed one_norm(A) ****" << std::endl;
      success = false;      
    }
  }

  {
    // infinity_norm
    Int i=0, j;
    s = mtl::infinity_norm(A);
    
    if (C.nrows() > 0) {
      i = 0; sum = T();
      for (j = 0; j < Int(C.ncols()); ++j)
        sum += MTL_ABS(C(i,j));
      norm = sum;
      ++i;
    }
    
    for (; i < Int(C.nrows()); ++i) {
      sum = T();
      for (j = 0; j < Int(C.ncols()); ++j)
        sum += MTL_ABS(C(i,j));
      norm = MTL_MAX(MTL_ABS(norm), MTL_ABS(sum));
    }
    
    if (s != norm) {
      std::cout << test_name.c_str() << " failed infinity_norm(A) ****" << std::endl;
      success = false;      
    }
  }

  set_diagonal_test(test_name, A, success);

  if (success)
    std::cout << test_name.c_str() << " passed single matrix algorithms test" << std::endl;

  return success;
}

template <class Matrix>
void
set_diagonal_test(std::string test_name, Matrix& A, bool& success)
{
  if (A.is_unit()) {
    std::cout << test_name.c_str() << " skipping set_diagonal" << std::endl;
    return;
  }
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  // set_diagnal
  Int i;
  mtl::set_diagonal(A, T(1));
  
  for (i = 0; i < A.nrows() && i < A.ncols(); ++i)
    if (A(i,i) != T(1)) {
      std::cout << test_name.c_str() << " failed set_diagnal(A,a) *****" << std::endl;
      success = false;
      break;
    }
}

template <class Mat>
inline void
fill_matrix(Mat& A, int /*sub*/, int /*super*/, mtl::rectangle_tag)
{
  typedef typename mtl::matrix_traits<Mat>::size_type Int;
  typedef typename mtl::matrix_traits<Mat>::value_type T;
  Int i, j;
  for (i = 0; i < A.nrows(); ++i)
    for (j = 0; j < A.ncols(); ++j)
      A(i,j) = T(i * A.ncols() + j);
}

template <class Mat>
inline void
fill_matrix(Mat& A, int sub, int super, mtl::banded_tag)
{
  typedef typename mtl::matrix_traits<Mat>::size_type Int;
  typedef typename mtl::matrix_traits<Mat>::value_type T;
  Int i, j;
  for (i = 0; i < A.nrows(); ++i) {
    Int first = MTL_MAX(0, int(i) - sub);
    Int last = MTL_MIN(int(A.ncols()), int(i) + super + 1);
    for (j = 0; j < A.ncols(); ++j)
      if (j >= first && j < last)
        A(i,j) = T(i * A.ncols() + j);
  }
}

template <class Mat>
inline void
fill_matrix(Mat& A, int sub, int super, mtl::symmetric_tag)
{
  typedef typename mtl::matrix_traits<Mat>::size_type Int;
  typedef typename mtl::matrix_traits<Mat>::value_type T;
  Int i, j;
  for (i = 0; i < A.nrows(); ++i) {
    Int first = MTL_MAX(0, int(i) - sub);
    Int last = MTL_MIN(int(A.ncols()), int(i) + super + 1);
    for (j = 0; j < A.ncols(); ++j)
      if (j >= first && j < last)
        A(i,j) = T(i + j);
  }
}



template <class Matrix>
void test_matvec_mult(std::string test, Matrix& A)
{

  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  typedef typename mtl::matrix_traits<Matrix>::shape Shape;
  Int M = A.nrows();
  Int N = A.ncols();
  Int i, j;

  dense1D<T> x(N), y(M);

  typename mtl::matrix<T>::type AA(M, N);
  mtl::dense1D<T> yy(M);
  mtl::dense1D<T> z(M);

  bool pass;

  //
  // mult test    y = A x + y
  //

  mtl::set_value(AA, T());
  mtl::set_value(A, T());

  fill_matrix(A, A.sub(), A.super(), Shape());
  mtl::copy(A, AA);
  //  fill_matrix(AA, A.sub(), A.super(), Shape());

  mtl::set_value(x, T(1));
  mtl::set_value(y, T(1));
  mtl::set_value(yy, T());

  mtl::mult(A, x, y);

  for (i = 0; i < M; ++i)
    for (j = 0; j < N; ++j)
      yy[i] += AA(i,j) * x[j];

  // compare y's
  pass = std::equal(y.begin(), y.end(), yy.begin());

  if (pass)
    std::cout << test.c_str() << " passed matvec mult1" << std::endl;
  else {
    std::cout << "*** matvec mult1 " << test.c_str() << " failed" << std::endl;
#if !defined(_MSVCPP_)
    std::cout << "result vector ";
    mtl::print_vector(y.begin(), y.end());
    std::cout << "correct vector ";
    mtl::print_vector(yy.begin(), yy.end());
#endif
  }

  //
  //  mult2
  //  
  mtl::set_value(y, T(1));
  mtl::set_value(yy, T(1));
  mtl::set_value(z, T(9));
  
  mtl::mult(A, x, y, z);

  for (i = 0; i < M; ++i) {
    T tmp = y[i];
    for (j = 0; j < N; ++j)
      tmp += AA(i,j) * x[j];
    yy[i] = tmp;
  }
  pass = std::equal(z.begin(), z.end(), yy.begin());

  if (pass)
    std::cout << test.c_str() << " passed matvec mult2 " << std::endl;
  else {
    std::cout << "*** matvec mult2 " << test.c_str() << " failed" << std::endl;
#if !defined(_MSVCPP_)
    std::cout << "result vector ";
    mtl::print_vector(z.begin(), z.end());
    std::cout << "correct vector ";
    mtl::print_vector(yy.begin(), yy.end());
#endif
  }
}

template <class Matrix>
void test_matvec_rankone(std::string test, Matrix&, banded_tag)
{
  std::cout << test.c_str() << " skipping rank one update" << std::endl;
}


template <class Matrix>
void test_matvec_rankone(std::string test, Matrix& A, rectangle_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  typedef typename mtl::matrix_traits<Matrix>::shape Shape;
  Int M = A.nrows();
  Int N = A.ncols();

  typename mtl::matrix<T>::type AA(M, N);

  dense1D<T> x(M), y(N);
  
  //  mtl::set_value(x, T(1));
  bool pass;

  mtl::set_value(A, T());
  mtl::set_value(AA, T());

  fill_matrix(A, A.sub(), A.super(), Shape());
  mtl::copy(A, AA);
  //  fill_matrix(AA, A.sub(), A.super(), Shape());

  Int i, j;

  //
  // rank one update test    A += x * y'
  //
  pass = true;

  for (i = 0; i < M; ++i)
    x[i] = T(i);

  for (i = 0; i < N; ++i)
    y[i] = T(i);

  // mtl version
  mtl::rank_one_update(A, x, y);

  for (i = 0; i < M; ++i)
    for (j = 0; j < N; ++j)
      AA(i,j) += x[i] * MTL_CONJ(y[j]);

  // compare A's

  pass = mtl::matrix_equal(A, AA);

  if (pass)
    std::cout << test.c_str() << " passed matvec rank one" << std::endl;
  else {
    std::cout << "*** matvec rank one " << test.c_str() << " failed" << std::endl;
#if !defined(_MSVCPP_)
    std::cout << "result" << std::endl;
    mtl::print_all_matrix(A);
    std::cout << "correct" << std::endl;
    mtl::print_all_matrix(AA);
#endif
  }
}


template <class Matrix>
void test_matvec_rankone(std::string test, Matrix& A, symmetric_tag)
{
  if (A.super() == int(A.nrows()) - 1)
    test_matvec_rankone(test, A, rectangle_tag());
  else
    std::cout << test.c_str() << " skipping rank one (banded symm)" << std::endl;
}


template <class Matrix>
void test_matvec_rankone(std::string test, Matrix& A) {
  typedef typename mtl::matrix_traits<Matrix>::shape Shape;
  test_matvec_rankone(test, A, Shape());
}


template <class Matrix>
void test_ranktwo(std::string test, Matrix& A, rectangle_tag)
{
  if (A.nrows() != A.ncols()) {
    std::cout << test.c_str() << " skipping rank two (A must be N x N)" << std::endl;
    return;
  }

  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;

  Int M = A.nrows();
  Int N = A.ncols();

  typename mtl::matrix<T>::type AA(M, N); // column oriented

  dense1D<T> x(M), y(M);

  bool pass;

  mtl::set_value(A, T());
  mtl::set_value(AA, T());

  // start with symmetric matrices
  Int i, j;
  for (i = 0; i < A.nrows(); ++++i) {
    Int first = MTL_MAX(0, int(i) - int(A.sub()));
    Int last = MTL_MIN(int(A.ncols()), int(i) + int(A.super()) + 1);
    for (j = 0; j < A.ncols(); ++++j)
      if (j >= first && j < last) {
        A(i,j) = T(i + j);
        AA(i, j) = T(i + j);
      }
  }

  //
  // rank two update test    A += x * y' + x' * y
  //
  // doing symmetric version where y == x
  // perhaps need to do two versions of the test,
  // one of them not symmetric
  //
  pass = true;

  for (i = 0; i < M; ++i)
    x[i] = T(i);

  for (i = 0; i < N; ++i)
    y[i] = T(i);

  mtl::rank_two_update(A, x, y);

  // a correct version
  for (i = 0; i < M; ++i) {
    for (j = 0; j < i; ++j) {
      T tmp = x[i] * MTL_CONJ(y[j]) + MTL_CONJ(x[j]) * y[i];
      AA(i, j) += tmp;
      AA(j, i) += tmp;
    }
    T tmp = x[i] * y[j] + x[j] * y[i];
    AA(i, j) += tmp;
  }

  // compare A's
  pass = mtl::matrix_equal(A, AA);
      
  if (pass)
    std::cout << test.c_str() << " passed rank two " << std::endl;
  else {
    std::cout << "*** rank two " << test.c_str() << " failed" << std::endl;
#if !defined(_MSVCPP_)
    std::cout << "result" << std::endl;
    mtl::print_all_matrix(A);
    std::cout << "correct" << std::endl;
    mtl::print_all_matrix(AA);
#endif
  }

}

template <class Matrix>
void test_ranktwo(std::string test, Matrix&, banded_tag)
{
  std::cout << test.c_str() << " skipping rank two update" << std::endl;
}


template <class Matrix>
void test_ranktwo(std::string test, Matrix& A, symmetric_tag)
{
  if (A.super() == int(A.nrows()) - 1)
    test_ranktwo(test, A, rectangle_tag());
  else
    std::cout << test.c_str() << " skipping rank two update (symm banded)" << std::endl;
}

template <class Matrix>
void test_ranktwo(std::string test, Matrix& A)
{
  typedef typename mtl::matrix_traits<Matrix>::shape Shape;
  test_ranktwo(test, A, Shape());
}

template <class Matrix, class Vector>
void simple_tri_solve(const Matrix& A, Vector& X, bool is_unit, bool is_upper)
{
  // this algorithm cooresponds to netlib dtrsv

  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;

  char DIAG, UPLO;
  T TEMP;
  int I, J;
  Int N = A.ncols();
  
  if (is_unit)
    DIAG = 'U';
  else
    DIAG = 'N';

  if (is_upper)
    UPLO = 'U';
  else
    UPLO = 'L';
  
  bool NOUNIT = DIAG == 'N';

  if (UPLO == 'U') {
    for (J = N - 1; J >= 0; --J) {
      if (X[J] != T(0)) {
        if (NOUNIT)
          X[J] = X[J]/A(J,J);
        TEMP = X[J];
        for (I = J - 1; I >= 0; --I)
          X[I] = X[I] - TEMP * A(I,J);
      }
    }
  } else {
    for (J = 0; J < int(N); ++J) {
      if (X[J] != T(0) ) {
        if (NOUNIT)
          X[J] = X[J]/A(J, J);
        TEMP = X[J];
        for (I = J + 1; I < int(N); ++I)
          X[I] = X[I] - TEMP*A(I, J);
      }
    }
  }
}



template <class MatrixA>
void test_tri_solve(std::string test, MatrixA& A, mtl::triangle_tag)
{
  if (A.nrows() != A.ncols()) {
    std::cout << test.c_str() << " skipping tri solve (A must be N x N)" << std::endl;
    return;
  }

  bool success = true;
  typedef typename mtl::matrix_traits<MatrixA>::value_type T;
  typedef typename mtl::matrix_traits<MatrixA>::size_type Int;

  dense1D<T> x(A.ncols()), y(A.ncols());

  typename mtl::matrix<T>::type C(A.nrows(), A.ncols());

  mtl::set_value(A, T());
  mtl::set_value(C, T());

  Int i,j;
  if (A.is_upper()) {
    for (i = 0; i < A.nrows(); ++i) {
      for (j = i; j < A.ncols(); ++j) {
        if ((!A.is_unit() && i == j) || j - i == 1) {
          A(i,j) = T(1.5);
        } else if (j > i) {
          T t = T(i + j + 1);
          A(i,j) = t;
        }
      }
    }
  } else { // lower
    for (i = 0; i < A.nrows(); ++i) {
      for (j = 0; j <= i; ++j) {
        if ((!A.is_unit() && i == j) || i - j == 1) {
          A(i,j) = T(1.5);
        } else if (j < i) {
          T t = T(i + j + 1);
          A(i,j) = t;
        }
      }
    }
  }
  mtl::copy(A, C);

  mtl::set_value(x, T(1));
  mtl::set_value(y, T(1));

  // MTL
  mtl::tri_solve(A, x);

  simple_tri_solve(C, y, A.is_unit(), A.is_upper());

  // COMPARE
  bool isequal = true;

  for (i = 0; i < A.ncols(); ++i)
    if (MTL_ABS(x[i] - y[i]) > MTL_MIN(MTL_ABS(x[i]),MTL_ABS(y[i])) / 100) {
#if !defined(_MSVCPP_)
      std::cout << x[i] << " != " << y[i] << std::endl;
#endif
      isequal = false;
      break;
    }
  if (! isequal) {
    std::cout << "*** " << test.c_str() << " failed tri_solve(A,x,y)" << std::endl;
#if !defined(_MSVCPP_)
    std::cout << "result vector  ";
    mtl::print_vector(x);
    std::cout << "correct vector ";
    mtl::print_vector(y);
#endif
    success = false;
  }

  if (success)
    std::cout << test.c_str() << " passed matvec tri_solve" << std::endl; 
}

template <class MatrixA>
void test_tri_solve(std::string test, MatrixA&, mtl::rectangle_tag)
{
  std::cout << test.c_str() << " skipping tri solve" << std::endl;
}

template <class MatrixA>
void test_tri_solve(std::string test, MatrixA&, mtl::banded_tag)
{
  std::cout << test.c_str() << " skipping tri solve" << std::endl;
}

template <class MatrixA>
void test_tri_solve(std::string test, MatrixA&, mtl::symmetric_tag)
{
  std::cout << test.c_str() << " skipping tri solve" << std::endl;
}

template <class MatrixA>
void test_tri_solve(std::string test, MatrixA& A)
{
  typedef typename mtl::matrix_traits<MatrixA>::shape Shape;
  test_tri_solve(test, A, Shape());
}


template <class MatA, class MatC, class MatC2>
bool test_add(std::string test, MatA& A, MatC& C, MatC2& C2)
{
  typedef typename mtl::matrix_traits<MatA>::size_type Int;
  typedef typename mtl::matrix_traits<MatA>::value_type T;
  Int i, j;
  bool passed = true;

  mtl::add(A, C);

  for (i = 0; i < A.nrows(); ++i) {
    Int first = MTL_MAX(0, int(i) - int(A.sub()));
    Int last = MTL_MIN(int(A.ncols()), int(i) + int(A.super()) + 1);
    for (j = 0; j < A.ncols(); ++j)
      if (j >= first && j < last)
        C2(i,j) = C2(i,j) + A(i,j);
  }
  if (A.is_unit())
    for (Int i = 0; i < MTL_MIN(A.nrows(), A.ncols()); ++i)
      C2(i,i) = C2(i,i) + T(1);

  if (! mtl::matrix_equal(C, C2)) {
    passed = false;
    std::cout << test.c_str() << " failed add" << std::endl;
#ifndef _MSVCPP_
    std::cout << "result" << std::endl;
    mtl::print_all_matrix(C);
    std::cout << "correct" << std::endl;
    mtl::print_all_matrix(C2);
#endif
  }

  std::cout << test.c_str() << " passed mat-mat add" << std::endl;

  return passed;
}


/*
 *  matmat algorithms
 */

template <class MatA, class MatB, class MatC, class MatC2>
void test_mult(std::string test, MatA& A, MatB& B, MatC& C, MatC2& C2)
{
  typedef typename mtl::matrix_traits<MatA>::size_type Int;

  mtl::mult(A, B, C);
  
  Int M = C.nrows();
  Int N = C.ncols();
  Int K = A.ncols();

  for (Int i = 0; i != M; ++i)
    for (Int j = 0; j != N; ++j)
      for (Int k = 0; k != K; ++k) {
        Int first = MTL_MAX(0, int(i) - int(A.sub()));
        Int last = MTL_MIN(int(A.ncols()), int(i) + int(A.super()) + 1);
        if (k >= first && k < last)
          C2(i,j) += A(i,k) * B(k,j);
      }

  if (A.is_unit()) {
    Int M = MTL_MIN(A.nrows(), A.ncols());
    Int N = B.ncols();
    for (Int i = 0; i < M; ++i)
      for (Int j = 0; j < N; ++j)
        C2(i,j) += B(i,j);
  }

  if (! mtl::matrix_equal(C, C2)) {
    std::cout << test.c_str() << " failed mult" << std::endl;
#if !defined(_MSVCPP_)
    std::cout << "result" << std::endl;
    mtl::print_all_matrix(C);
    std::cout << "correct" << std::endl;
    mtl::print_all_matrix(C2);
#endif
  } else
    std::cout << test.c_str() << " passed mat-mat multiply test" << std::endl;
}



template <class MatA, class MatC, class MatC2>
void test_copy(std::string test, MatA& A, MatC& C, MatC2& C2)
{
  typedef typename mtl::matrix_traits<MatA>::size_type Int;

  mtl::copy(A, C);

  for (Int i = 0; i < A.nrows(); ++i)
    for (Int j = 0; j < A.ncols(); ++j) {
      Int first = MTL_MAX(0, int(i) - int(A.sub()));
      Int last = MTL_MIN(int(A.ncols()), int(i) + int(A.super()) + 1);
      if (j >= first && j < last)
        C2(i,j) = A(i,j);
    }

  if (A.is_unit())
    mtl::set_diagonal(C2,1);

  if (! mtl::matrix_equal(C, C2)) {
    std::cout << test.c_str() << " failed copy" << std::endl;
#if !defined(_MSVCPP_)
    std::cout << "result" << std::endl;
    mtl::print_all_matrix(C);
    std::cout << "correct" << std::endl;
    mtl::print_all_matrix(C2);
#endif
  } else {
    std::cout << test.c_str() << " passed mat-mat copy test" << std::endl;
  }
}

#if 0

template <class MatA, class MatB, class MatC, class MatC2>
bool test_swap(std::string test, MatA& A, MatB& B, MatC& C, MatC2& C2)
{
  bool passed = true;
  fill_mats(A, B, C);

  mtl::swap(A, C);

  for (int i = 0; i < A.nrows(); ++i)
    for (int j = 0; j < A.ncols(); ++j) {
      Int first = MTL_MAX(0, int(i) - int(A.sub()));
      Int last = MTL_MIN(int(A.ncols()), int(i) + int(A.super()) + 1);
      if (j >= first && j < last)
        C2(i,j) = A(i,j);
    }

  if (! mtl::matrix_equal(C, C2)) {
    passed = false;
    std::cout << test.c_str() << " failed swap" << std::endl;
    std::cout << "result" << std::endl;
    mtl::print_row(C);
    std::cout << "correct" << std::endl;
    mtl::print_all_matrix(C2);
  }
  return passed;
}


template <class MatA, class MatB, class MatC, class MatC2>
bool test_ele_mult(std::string test, MatA& A, MatB& B, MatC& C, MatC2& C2)
{
  bool passed = true;
  int i, j;
  fill_mats(A, B, C);

  mtl::ele_mult(A, C);

  mtl::set_value(C2, 0);
  for (i = 0; i < C2.nrows(); ++++i)
    for (j = 0; j < C2.ncols(); ++++j)
      C2(i,j) = 1;

  for (i = 0; i < A.nrows(); ++i)
    for (j = 0; j < A.ncols(); ++j)
      C2(i,j) *= A(i,j);

  if (! mtl::matrix_equal(C, C2)) {
    passed = false;
    std::cout << test.c_str() << " failed" << std::endl;
    std::cout << "result" << std::endl;
    mtl::print_all_matrix(C);
    std::cout << "correct" << std::endl;
    mtl::print_all_matrix(C2);
  }
  return passed;
}


template <class MatA, class MatB, class MatC, class MatC2>
bool test_transpose(std::string test, MatA& A, MatB& B, MatC& C, MatC2& C2)
{
  bool passed = true;

  // only transpose square matrices
  if (A.nrows() == A.ncols() && C.nrows() == C.ncols() 
      && A.nrows() == A.ncols()) {
    fill_mats(A, B, C);

    mtl::transpose(A, C);

    for (int i = 0; i < A.nrows(); ++i)
      for (int j = 0; j < A.ncols(); ++j)
        C2(j,i) = A(i,j);
    
    if (! mtl::matrix_equal(C, C2)) {
      passed = false;
      std::cout << test.c_str() << " failed" << std::endl;
      std::cout << "result" << std::endl;
      mtl::print_all_matrix(C);
      std::cout << "correct" << std::endl;
      mtl::print_all_matrix(C2);
    }
  }
  return passed;
}

#endif /* matmat algos */

#endif /* MTL_ALGO_TEST_H */
