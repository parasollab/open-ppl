//
// Copyright 1997, University of Notre Dame.
// Authors: Andrew Lumsdaine, Jeremy G. Siek
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
// $Id: banded_matvec_mult.cc 1749 2004-01-27 00:01:18Z gabrielt $
//

#include <iostream>
#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"  
#include "mtl/linalg_vec.h"

/*

  Sample Output

  Array A in packed form
  [
  [1,1,1,],
  [2,2,2,],
  [3,3,3,],
  [4,4,],
  [5,],
  ]
  x
  [1,2,3,4,5,]
  y
  [1,1,1,1,1,]
  Ax + y
  [15,39,75,75,53,]

  */

using namespace mtl;

typedef matrix< double, banded<>, banded<>, row_major>::type Matrix; 
typedef dense1D<double> Vector;

int
main()
{
  const int M = 5;
  const int N = 5;
  
  Matrix A(M, N, 0, 2);

  Vector x(N);
  Vector y(M);

  //           1  1  1              1        1
  //
  //              2  2  2           2        1
  //
  //       A =       3  3  3    x = 3    y = 1
  //
  //                    4  4        4        1
  //
  //                       5        5        1

  int row = 1;

  Matrix::iterator ri = A.begin();
  while (ri != A.end()) {
    Matrix::Row::iterator i = (*ri).begin();
    while (i != (*ri).end()) {
      *i = row;
      ++i;
    }
    ++row;
    ++ri;
  }

  for (int i = 0; i < N; ++i)
    x[i] = i + 1;

  mtl::set_value(y, 1);

  std::cout << "Array A in packed form" << std::endl;
  print_row(A);

  std::cout << "x" << std::endl;
  print_vector(x);
  std::cout << "y" << std::endl;
  print_vector(y);

  mult(A, scaled(x, 2), scaled(y, 3), y);

  std::cout << "Ax + y" << std::endl;
  print_vector(y);

  return 0;
}
