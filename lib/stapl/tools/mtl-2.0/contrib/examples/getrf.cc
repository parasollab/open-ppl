// -*- c++ -*-
//
// Copyright 1997, 1998, 1999 University of Notre Dame.
// Authors: Andrew Lumsdaine, Jeremy G. Siek, Lie-Quan Lee
//
// This file is part of the Matrix Template Library
//
// You should have received a copy of the License Agreement for the
// Matrix Template Library along with the software;  see the
// file LICENSE.  If not, contact Office of Research, University of Notre
// Dame, Notre Dame, IN  46556.
//
// Permission to modify the code and to distribute modified code is
// granted, provided the text of this NOTICE is retained, a notice that
// the code was modified is included with the above COPYRIGHT NOTICE and
// with the COPYRIGHT NOTICE in the LICENSE file, and that the LICENSE
// file is distributed with the modified code.
//
// LICENSOR MAKES NO REPRESENTATIONS OR WARRANTIES, EXPRESS OR IMPLIED.
// By way of example, but not limitation, Licensor MAKES NO
// REPRESENTATIONS OR WARRANTIES OF MERCHANTABILITY OR FITNESS FOR ANY
// PARTICULAR PURPOSE OR THAT THE USE OF THE LICENSED SOFTWARE COMPONENTS
// OR DOCUMENTATION WILL NOT INFRINGE ANY PATENTS, COPYRIGHTS, TRADEMARKS
// OR OTHER RIGHTS.
//
// $Id: getrf.cc 1749 2004-01-27 00:01:18Z gabrielt $
//


#include <complex>
#include "mtl/mtl2lapack.h"
#include "mtl/dense1D.h"
#include "mtl/utils.h"


/*
  Sample Output

  A:
  3x3
  [
  [1,2,2],
  [2,1,2],
  [2,2,1]
  ]
  b:
  3x1
  [
  [15],
  [15],
  [15]
  ]
  x:
  3x1
  [
  [3],
  [3],
  [3]
  ]

  */


int
main()
{
  using namespace mtl2lapack;
  using namespace mtl;

  const int M = 3;
  const int N = 3;
  const int NRHS = 1;
  //begin
  double da [] = { 1, 2, 2, 2, 1, 2, 2, 2, 1 };
  lapack_matrix<double,external>::type A(da, M, N);
  lapack_matrix<double>::type B(M*NRHS, NRHS);
  mtl::set_value(B, 15.0);
  dense1D<int> pivot(N, 0);
  //end
  std::cout << "A:" << std::endl;
  print_all_matrix(A);

  std::cout << "b:" << std::endl;
  print_all_matrix(B);

  // Factor A, solve the system, and print the solution.
  //begin
  int info = getrf(A, pivot);
  //end
  print_all_matrix(A);

  std::cout << "pivots" << std::endl;
  print_vector(pivot);
  //begin
  if (info == 0) {
    info = getrs('N', A, pivot, B);

    if (info == 0) {
      std::cout << "x:" << std::endl;
      print_all_matrix(B);
    } else
      std::cout << "Factorization failed with INFO = " << info << std::endl;
  } else
    std::cout << "Solve failed with INFO = " << info << std::endl;
  //end
  return 0;
}
