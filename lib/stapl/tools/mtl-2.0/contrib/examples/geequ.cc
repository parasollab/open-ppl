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
// $Id: geequ.cc 1749 2004-01-27 00:01:18Z gabrielt $
//


#include "mtl/mtl2lapack.h"
#include "mtl/dense1D.h"
#include "mtl/utils.h"


/*
  Sample Output

  3x3
  [
  [2,4,8],
  [2,8,16],
  [2,16,32]
  ]
  Row scale factors:
  [0.125,0.0625,0.03125,]
  Column scale factors:
  [4,2,1,]
  Scaled A:
  3x3
  [
  [1,1,1],
  [0.5,1,1],
  [0.25,1,1]
  ]
  
  */

int
main()
{
  const int N = 3;
  using namespace mtl;
  //begin
  double da [] = { 2, 2, 2, 4, 8, 16, 8, 16, 32 };
  mtl2lapack::lapack_matrix<double,external>::type A(da, N, N);
  double rowcond, colcond;
  double amax;
  dense1D<double> rowsca(N), colsca(N);
  int info;
  //end
  print_all_matrix(A);

  // Compute and print the scale factors that will
  //   equilibrate A

  //begin
  info = mtl2lapack::geequ(A, rowsca, colsca, rowcond, colcond, amax); 

  if (info > N) {
    std::cout << "Column " << info - N << " of A is exactly zero." << std::endl;
    return 0;
  } else if (info > 0)
    std::cout << "Row " << info << " of A is exactly zero." << std::endl;
  //end
  std::cout << "Row scale factors:" << std::endl;
  print_vector(rowsca);

  std::cout << "Column scale factors:" << std::endl;
  print_vector(colsca);

  // Use the scale factors to equilibrate A and print
  //   the equilibrated A.

  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j)
      A(i,j) *= rowsca[i] * colsca[j];

  std::cout << "Scaled A:" << std::endl;
  print_all_matrix(A);

  return 0;
}
