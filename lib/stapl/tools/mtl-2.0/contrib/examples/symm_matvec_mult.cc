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
// $Id: symm_matvec_mult.cc 1749 2004-01-27 00:01:18Z gabrielt $
//

#include <mtl/dense1D.h>
#include <mtl/mtl.h>
#include <mtl/matrix.h>


/*

  Sample Output

  A in full form:
  4x4
  [
  [1,2,3,4],
  [2,5,6,7],
  [3,6,8,9],
  [4,7,9,10]
  ]
  x:
  [2,2,2,2,]
  y:
  [4000,5000,6000,7000,]
  Ax + y:
  [20,40,52,60,]
  

 */

using namespace mtl;

//begin
typedef matrix<double, symmetric<lower>, array< dense<> >, row_major>::type Matrix;
typedef dense1D<double> Vector;
//end

int
main()
{
  typedef Matrix::size_type sizeT;
  sizeT i, j;
  const sizeT N = 4;
  //begin
  Matrix A(N);
  Vector x(N);
  Vector y(N);
  //end
  //         1  2  3  4       2       4000
  //
  //     A = 2  5  6  7   x = 2   y = 5000
  //
  //         3  6  8  9       2       6000
  //
  //         4  7  9 10       2       7000

  int c = 0;
  for (i = 0; i < N; ++i)
    for (j = i; j < N; ++j)
      A(i,j) = ++c;

  for (i = 0; i < N; ++i) {
    x[i] = 2;
    y[i] = (i + 4) * 1000;
  }

  std::cout << "A in full form:" << std::endl;
  print_all_matrix(A);

  std::cout << "x:" << std::endl;
  print_vector(x);

  std::cout << "y:" << std::endl;
  print_vector(y);
  //begin
  mult(A, x, y);

  std::cout << "Ax + y:" << std::endl;
  print_vector(y);
  //end
  return 0;
}
