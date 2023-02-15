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
// $Id: vec_scale.cc 1749 2004-01-27 00:01:18Z gabrielt $
//

#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"  

/*

  Sample Output

  Before scaling row 2:
  3x3
  [
  [5,5.5,6],
  [2.5,3,3.5],
  [1,1.5,2]
  ]
  After scaling row 2:
  3x3
  [
  [5,5.5,6],
  [5,6,7],
  [1,1.5,2]
  ]
  
  */

using namespace mtl;

typedef matrix< double, rectangle<>, dense<>, row_major>::type Matrix;  

int
main()
{
  const Matrix::size_type N = 3;

  Matrix A(N, N);

  A(0,0) = 5;   A(0,1) = 4;   A(0,2) = 3;
  A(1,0) = 2.5; A(1,1) = 3;   A(1,2) = 3.5;
  A(2,0) = 1;   A(2,1) = 1.5; A(2,2) = 2;

  std::cout << std::endl;
  std::cout << "Before scaling row 2:" << std::endl;
  print_all_matrix(A);

  double scal = A(0,0) / A(1,0);

  copy(scaled(A[1], scal), A[1]);

  std::cout << "After scaling row 2:" << std::endl;
  print_all_matrix(A);

  return 0;
}
