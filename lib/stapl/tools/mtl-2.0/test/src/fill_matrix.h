/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef MTL_FILL_MATRIX_H
#define MTL_FILL_MATRIX_H

template <class Matrix, class Shape>
void iterator_fill_sparse(Matrix& A, row_tag, Shape) {
  // Can't iterate through an empty sparse matrix
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  Int i, j;
  T c = T(0);
  for (i = 0; i < A.nrows(); ++i)
    for (j = 0; j < A.ncols(); ++j) {
      c = c + T(1);
      A(i, j) = c;
    }
}

template <class Matrix, class Shape>
void iterator_fill_sparse(Matrix& A, column_tag, Shape) {
  // Can't iterate through an empty sparse matrix
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  Int i, j;
  T c = T(0);
  for (j = 0; j < A.ncols(); ++j)
    for (i = 0; i < A.nrows(); ++i) {
      c = c + T(1);
      A(i, j) = c;
    }
}


template <class Matrix>
void iterator_fill_sparse(Matrix& A, row_tag, symmetric_tag) {
  // Can't iterate through an empty sparse matrix
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  Int i, j;
  T c = T(0);
  for (i = 0; i < A.nrows(); ++i)
    for (j = 0; j <= i; ++j) {
      c = c + T(1);
      A(i, j) = c;
    }
}

template <class Matrix>
void iterator_fill_sparse(Matrix& A, column_tag, symmetric_tag) {
  // Can't iterate through an empty sparse matrix
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  Int i, j;
  T c = T(0);
  for (j = 0; j < A.ncols(); ++j)
    for (i = 0; i <= j; ++i) {
      c = c + T(1);
      A(i, j) = c;
    }
}

template <class Matrix>
void iterator_fill(Matrix& A, sparse_tag) {
  // Can't iterate through an empty sparse matrix
  typedef typename mtl::matrix_traits<Matrix>::orientation Orien;
  typedef typename mtl::matrix_traits<Matrix>::shape Shape;
  iterator_fill_sparse(A, Orien(), Shape());
}



template <class Matrix>
void iterator_fill(Matrix& A, dense_tag) {
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  T c = T(0);
  for (typename Matrix::iterator i = A.begin();
       i != A.end(); ++i)
    for (typename Matrix::OneD::iterator j = (*i).begin();
	 j != (*i).end(); ++j) {
      c = c + T(1);
      *j = c;
    }
}

template <class Matrix>
void iterator_fill(Matrix& A) {
  typedef typename mtl::matrix_traits<Matrix>::sparsity Sparsity;  
  iterator_fill(A, Sparsity());
}

template <class Matrix>
void matrix_fill(Matrix& A, symmetric_tag) {
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  for (typename Matrix::iterator i = A.begin();
       i != A.end(); ++i)
    for (typename Matrix::OneD::iterator j = (*i).begin();
	 j != (*i).end(); ++j)
      *j = j.row() + j.column();
}


template <class Matrix>
void matrix_fill(Matrix& A, rectangle_tag) {
  iterator_fill(A);
}

template <class Matrix>
void matrix_fill(Matrix& A, banded_tag) {
  iterator_fill(A);
}

template <class Matrix>
void matrix_fill(Matrix& A) {
  typedef typename mtl::matrix_traits<Matrix>::shape Shape;
  matrix_fill(A, Shape());
}

#endif /* MTL_FILL_MATRIX_H */
