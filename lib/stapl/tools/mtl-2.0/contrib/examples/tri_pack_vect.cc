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
// $Id: tri_pack_vect.cc 1749 2004-01-27 00:01:18Z gabrielt $
//


#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"  
#include "mtl/linalg_vec.h"

/*

  Sample Output

  Array A:
  3x3
  [
  [1,0,0],
  [2,4,0],
  [3,5,6]
  ]
  Vector x
  [3,2,1,]
  Ax
  [3,14,25,]

  */

using namespace mtl;

int
main()
{
  //begin
  typedef matrix< double, 
                  triangle<lower>, 
                  packed<external>, 
                  column_major >::type Matrix; 
  typedef dense1D<double> Vector;
  //         1               3
  //     A = 2  4       x =  2
  //         3  5  6         1
  const Matrix::size_type N = 3;
  double dA[] = { 1, 2, 3, 4, 5, 6 };
  Matrix A(dA, N, N);
  Vector x(N), Ax(N);
  for (unsigned int i = 0; i < N; ++i)
    x[i] = 3-i;
  //end
  
  std::cout << "Array A:" << std::endl;
  print_all_banded(A, A.sub(), A.super());

  std::cout << "Vector x" << std::endl;
  print_vector(x);

  //begin
  mult(A, x, Ax);
  //end

  std::cout << "Ax" << std::endl;
  print_vector(Ax);

  return 0;
}
