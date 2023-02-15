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
// $Id: gecon.cc 1749 2004-01-27 00:01:18Z gabrielt $
//


#include "mtl/mtl2lapack.h"
#include "mtl/dense1D.h"
#include "mtl/utils.h"


/*
  Sample Output

  3x3
  [
  [1,2,2],
  [2,1,2],
  [2,2,1]
  ]
  Estimated condition number: 7

 */
int
main()
{
  const int M = 3;
  const int N = 3;
  using namespace mtl;
  //begin
  double da[] = { 1, 2, 2, 2, 1, 2, 2, 2, 1};
  mtl2lapack::lapack_matrix<double, external>::type A(da, M, N);
  dense1D<int> pivot(N, 0);
  double anorm = 0;
  double cond;
  //end
  print_all_matrix(A);
  //begin
  int info = mtl2lapack::getrf(A, pivot);

  if (info == 0) {
    anorm = 5;
    mtl2lapack::gecon('1', A, anorm, cond);
    
    std::cout << "Estimated condition number: ";
    if (info == 0)
      std::cout << 1.0 / cond << std::endl;
    else
      std::cerr << "error INFO = " << info << std::endl;;
  } else {
    std::cerr << "error INFO = " << info << std::endl;
  }
  //end  
  return 0;
}
