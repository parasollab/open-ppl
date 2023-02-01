/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef ITER_IJ_TEST_H
#define ITER_IJ_TEST_H

#include "mtl/mtl_config.h"

template <class Matrix>
bool iterator_operator_ij_test(const Matrix& A, std::string test_name, 
			       rectangle_tag, row_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  T c = T(0);
  for (Int i = 0; i < A.nrows(); ++i)
    for (Int j = 0; j < A.ncols(); ++j) {
      c = c + T(1);
      if (A(i,j) != c || A[i][j] != c) {
	std::cerr << "**** FAILED: (iterator operator_ij) "
	     << test_name.c_str() << " ****" << std::endl;
#if !defined(_MSVCPP_)
	std::cerr << "A(" << i << "," << j << ") = " << A(i,j) << std::endl;
	std::cerr << "A[" << i << "][" << j << "] = " << A[i][j] << std::endl;
	std::cerr << "c = " << c << std::endl;
#endif
	return false;
      }
    }
  std::cout << test_name.c_str() << " passed iterator operator_ij" << std::endl;
  return true;
}


template <class Matrix>
bool iterator_operator_ij_test(const Matrix& A, std::string test_name, 
			       rectangle_tag, column_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  T c = T(0);
  for (Int j = 0; j < A.ncols(); ++j)
    for (Int i = 0; i < A.nrows(); ++i) {
      c = c + T(1);
      if (A(i,j) != c || A[j][i] != c) {
	std::cerr << "**** FAILED: (iterator operator_ij rect column) "
	     << test_name.c_str() << " ****" << std::endl;
#if !defined(_MSVCPP_)
	std::cerr << "A(" << i << "," << j << ") = " << A(i,j) << std::endl;
	std::cerr << "A[" << j << "][" << i << "] = " << A[j][i] << std::endl;
	std::cerr << "c = " << c << std::endl;
#endif
	return false;
      }
    }
 std::cout << test_name.c_str() << " passed iterator operator_ij (rect col)" << std::endl;
  return true;
}

template <class Matrix>
bool iterator_operator_ij_test(const Matrix& A, std::string test_name, 
			   banded_tag, row_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  Int i, j;
  int lo = A.sub();
  int up = A.super();
  T c = T(0);
  for (i = 0; i < A.nrows(); ++i) {
    Int first = MTL_MAX(0, int(i) - lo);
    Int last = MTL_MIN(int(A.ncols()), int(i) + up + 1);
    for (j = 0; j < A.ncols(); ++j) {
      if (j >= first && j < last) {
	c = c + T(1);
	if (A(i,j) != c || A[i][j] != c) {
		std::cerr << "**** FAILED: ( iterator_operator(i,j) banded row) "
	       << test_name.c_str() << " ****" << std::endl;
#if !defined(_MSVCPP_)
	  std::cerr << "A(" << i << "," << j << ") = " << A(i,j) << std::endl;
	  std::cerr << "A[" << i << "][" << j << "] = " << A[i][j] << std::endl;
	  std::cerr << "c = " << c << std::endl;
#endif
	  return false;
	}
      }
    }
  }
  std::cout << test_name.c_str() << " passed iterator operator_ij banded row" << std::endl;
  return true;
}


template <class Matrix>
bool iterator_operator_ij_test(const Matrix& A, std::string test_name, 
			   banded_tag, column_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  Int i, j;
  int lo = A.sub();
  int up = A.super();
  T c = T(0);
  for (j = 0; j < A.ncols(); ++j) {
    Int first = MTL_MAX(0, int(j) - up);
    Int last = MTL_MIN(int(A.nrows()), int(j) + lo + 1);
    for (i = 0; i < A.nrows(); ++i) {
      if (i >= first && i < last) {
	c = c + T(1);
	if (A(i,j) != c || A[j][i] != c) {
	  std::cerr << "**** FAILED: ( iterator_operator(i,j) banded column) "
	       << test_name.c_str() << " ****" << std::endl;
#if !defined(_MSVCPP_)
	  std::cerr << "A(" << i << "," << j << ") = " << A(i,j) << std::endl;
	  std::cerr << "A[" << j << "][" << i << "] = " << A[j][i] << std::endl;
	  std::cerr << "c = " << c << std::endl;
#endif
	  return false;
	}
      }
    }
  }
 std::cout << test_name.c_str() << " passed iterator operator_ij banded column" << std::endl;
  return true;
}


template <class Matrix>
bool iterator_operator_ij_test(const Matrix& A, std::string test_name, 
			   symmetric_tag, row_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  Int i, j;
  int lo = A.sub();
  int up = A.super();
  for (i = 0; i < A.nrows(); ++i) {
    Int first = MTL_MAX(0, int(i) - lo);
    Int last = MTL_MIN(int(A.ncols()), int(i) + up + 1);
    for (j = 0; j < A.ncols(); ++j) {
      if (j >= first && j < last) {
	if (A(i,j) != T(i + j)) {
	  // JGS, can't do this test because the indices
	  // can not be swapped using the [][] operators
	  // so they realy should not be used with symmetric
	  // matrices:
	  //	  A[j][i] != (i + j)
	  std::cerr << "**** FAILED: ( iterator_operator(i,j) symmetric row) "
	       << test_name.c_str() << " ****" << std::endl;
#if !defined(_MSVCPP_)
	  std::cerr << "A(" << i << "," << j << ") = " << A(i,j) << std::endl;
	  std::cerr << "correct = " << i + j << std::endl;
	  mtl::print_all_banded(A, lo, up);
#endif
	  return false;
	}
      }
    }
  }
 std::cout << test_name.c_str() << " passed iterator operator_ij symmetric row" << std::endl;
  return true;
}


template <class Matrix>
bool iterator_operator_ij_test(const Matrix& A, std::string test_name, 
			   symmetric_tag, column_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  Int i, j;
  int lo = A.sub();
  int up = A.super();
  for (j = 0; j < A.ncols(); ++j) {
    Int first = MTL_MAX(0, int(j) - up);
    Int last = MTL_MIN(int(A.nrows()), int(j) + lo + 1);
    for (i = 0; i < A.nrows(); ++i) {
      if (i >= first && i < last)
	if (A(i,j) != T(i + j)) {
	  // JGS, can't do this test because the indices
	  // can not be swapped using the [][] operators
	  // so they realy should not be used with symmetric
	  // matrices:
	  //	  A[j][i] != (i + j)
	  std::cerr << "**** FAILED: ( iterator_operator(i,j) symmetric column) "
	       << test_name.c_str() << " ****" << std::endl;
#if !defined(_MSVCPP_)
	  std::cerr << "A(" << i << "," << j << ") = " << A(i,j) << std::endl;
	  std::cerr << "correct = " << i + j << std::endl;
#endif
	  return false;
	}
    }
  }
 std::cout << test_name.c_str() << " passed iterator operator_ij symmetric column" << std::endl;
  return true;
}


template <class Matrix>
bool iterator_operator_ij_test(const Matrix& , std::string test_name, 
			   diagonal_tag, row_tag)
{
 std::cout << test_name.c_str() << " skipping iterator operator_ij" << std::endl;
  return true;
}

template <class Matrix>
bool iterator_operator_ij_test(const Matrix& , std::string test_name, 
			   diagonal_tag, column_tag)
{
 std::cout << test_name.c_str() << " skipping iterator operator_ij" << std::endl;
  return true;
}


#if 0
template <class Matrix>
bool iterator_operator_ij_test(const Matrix& A, std::string test_name,
			       sparse_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::orientation Orien;
  //  iterator_fill(A);
  return iterator_operator_ij_test_sparse(A, test_name, Orien());
}
#endif

template <class Matrix>
bool iterator_operator_ij_test(const Matrix& A, std::string test_name)
{
  typedef typename mtl::matrix_traits<Matrix>::shape Shape;
  typedef typename mtl::matrix_traits<Matrix>::orientation Orien;
  return iterator_operator_ij_test(A, test_name, Shape(), Orien());
}

#if 0
template <class Matrix>
bool iterator_operator_ij_test(const Matrix& A, std::string test_name)
{
  typedef typename mtl::matrix_traits<Matrix>::sparsity Sparsity;
  return iterator_operator_ij_test(A, test_name, Sparsity());
}
#endif
 

template <class Matrix>
bool strided_iterator_operator_ij_test(const Matrix& A, std::string test_name, 
			       rectangle_tag, row_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  T c = T();
  for (Int j = 0; j < A.ncols(); ++j)
    for (Int i = 0; i < A.nrows(); ++i) {
      c = c + T(1);
      if (A(i,j) != c || A[i][j] != c) {
	std::cerr << "**** FAILED: (strided iterator operator_ij rect row) "
	     << test_name.c_str() << " ****" << std::endl;
#if !defined(_MSVCPP_)
	std::cerr << "A(" << i << "," << j << ") = " << A(i,j) << std::endl;
	std::cerr << "A[" << i << "," << j << "] = " << A[i][j] << std::endl;
	std::cerr << "c = " << c << std::endl;
#endif
	return false;
      }
    }
 std::cout << test_name.c_str() << " passed strided iterator operator_ij rect row" << std::endl;
  return true;
}

template <class Matrix>
bool strided_iterator_operator_ij_test_sparse(const Matrix& A, 
					      std::string test_name, 
					      row_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  T c = T();
  for (Int j = 0; j < A.ncols(); ++j)
    for (Int i = 0; i < A.nrows(); ++i) {
      c = c + T(1);
      if (A(i,j) != c || A[i][j] != c) {
	std::cerr << "**** FAILED: (strided iterator operator_ij sparse row) "
	     << test_name.c_str() << " ****" << std::endl;
#if !defined(_MSVCPP_)
	std::cerr << "A(" << i << "," << j << ") = " << A(i,j) << std::endl;
	std::cerr << "A[" << i << "," << j << "] = " << A[i][j] << std::endl;
	std::cerr << "c = " << c << std::endl;
#endif
	return false;
      }
    }
 std::cout << test_name.c_str() << " passed strided iterator operator_ij sparse column" << std::endl;
  return true;
}

template <class Matrix>
bool strided_iterator_operator_ij_test_sparse(const Matrix& A,
					      std::string test_name, 
					      column_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  T c = T();
  for (Int i = 0; i < A.nrows(); ++i)
    for (Int j = 0; j < A.ncols(); ++j) {
      c = c + T(1);
      if (A(i,j) != c || A[j][i] != c) {
	std::cerr << "**** FAILED: (strided iterator operator_ij sparse column) "
	     << test_name.c_str() << " ****" << std::endl;
#if !defined(_MSVCPP_)
	std::cerr << "A(" << i << "," << j << ") = " << A(i,j) << std::endl;
	std::cerr << "A[" << j << "," << i << "] = " << A[j][i] << std::endl;
	std::cerr << "c = " << c << std::endl;
#endif
	return false;
      }
    }
 std::cout << test_name.c_str() << " passed strided iterator operator_ij sparse column" << std::endl;
  return true;
}


template <class Matrix>
bool strided_iterator_operator_ij_test(const Matrix& A, std::string test_name, 
				       rectangle_tag, column_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::size_type Int;
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  T c = T();
  for (Int i = 0; i < A.nrows(); ++i)
    for (Int j = 0; j < A.ncols(); ++j) {
      c = c + T(1);
      if (A(i,j) != c || A[j][i] != c) {
	std::cerr << "**** FAILED: (strided iterator operator_ij rect column) "
	     << test_name.c_str() << " ****" << std::endl;
#if !defined(_MSVCPP_)
	std::cerr << "A(" << i << "," << j << ") = " << A(i,j) << std::endl;
	std::cerr << "A[" << j << "," << i << "] = " << A[j][i] << std::endl;
	std::cerr << "c = " << c << std::endl;
#endif
	return false;
      }
    }
 std::cout << test_name.c_str() << " passed strided iterator operator_ij (rect col)" << std::endl;
  return true;
}

template <class Matrix>
bool strided_iterator_operator_ij_test(const Matrix& A, std::string test_name,
				       sparse_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::orientation Orien;
  return strided_iterator_operator_ij_test_sparse(A, test_name, Orien());
}

template <class Matrix>
bool strided_iterator_operator_ij_test(const Matrix& A, std::string test_name,
				       dense_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::shape Shape;
  typedef typename mtl::matrix_traits<Matrix>::orientation Orien;
  return strided_iterator_operator_ij_test(A, test_name, Shape(), Orien());
}

template <class Matrix>
bool strided_iterator_operator_ij_test(const Matrix& A, std::string test_name)
{
  typedef typename mtl::matrix_traits<Matrix>::sparsity Sparsity;
  return strided_iterator_operator_ij_test(A, test_name, Sparsity());
}


#endif
