/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef _MTL_MATRIX_TEST_
#define _MTL_MATRIX_TEST_

#include <string>
#include "mtl/matrix_traits.h"
#include <utility>
#include <iostream>

/*
  A collection of test routines for MTL matrices.

  Functionality tested:

  twod/oned iterators  read, write, traversal, const, non-const, indices

  operator(i,j) read, write, const, non-const
     this also tests nrows() and ncols()

  operator[], noneds()

 * oned's operator(s,f)

  rows(A), columns(A)
  trans(A)
  scaled(A,s)

  Configurations tested:  

  row/column orientations
  various shapes (rect, banded, diagonal)
  dense, sparse

  still need to test:
    index() functions
    external sparse and others
    small cases
    reverse iterators

  */

using mtl::rectangle_tag;
using mtl::banded_tag;
using mtl::diagonal_tag;
using mtl::symmetric_tag;
using mtl::row_tag;
using mtl::column_tag;
using mtl::dense_tag;
using mtl::sparse_tag;
using mtl::matrix_traits;
using mtl::strideable;
using mtl::not_strideable;

#include "fill_matrix.h"


template <class Matrix>
void
create_and_run_rect(int M, int N, std::string test_name, Matrix*, 
		    mtl::internal_tag)
{
  Matrix A(M, N);
  do_test(A, test_name);
}

template <class Matrix>
void
create_and_run_rect(int M, int N, std::string test_name, Matrix*, 
		    mtl::external_tag)
{
  // do the static thing as to completely ignore M, N
  // no need to allocate off of heap
  typedef typename mtl::matrix_traits<Matrix>::value_type T;
  T static_a[ Matrix::M * Matrix::N + 1]; // + 1 so not get zero compiler error
  T* a;
  bool del_a = false;

  if (Matrix::M != 0) {
    a = static_a;
  } else {
    del_a = true;
    a = new T[M * N];
  }

  Matrix A(a, M, N);

  do_test(A, test_name);

  if (del_a)
    delete a;
}

template <class Matrix>
void
create_and_run(int M, int N, int , int, 
               std::string test_name, Matrix* a, mtl::rectangle_tag)
{
  typedef typename mtl::matrix_traits<Matrix>::storage_loc StoreLoc;
  create_and_run_rect(M, N, test_name, a, StoreLoc());
}


template <class Matrix>
void
create_and_run(int M, int N, int SUB, int SUP,
	       std::string test_name, Matrix*, mtl::banded_tag)
{
  Matrix A(M, N, SUB, SUP);
  do_test(A, test_name);
}

template <class Matrix>
void
create_and_run(int M, int N, int SUB, int ,
	       std::string test_name, Matrix*, mtl::symmetric_tag)
{
  Matrix A(M, SUB);
  do_test(A, test_name);
}

template <class Matrix>
void
create_and_run(int M, int N, int , int ,
	       std::string test_name, Matrix*, mtl::triangle_tag)
{
  Matrix A(M, N);
  do_test(A, test_name);
}


#endif /* _MTL_MATRIX_TEST_ */
