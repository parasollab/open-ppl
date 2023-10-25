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
// $Id: tri_solve.cc 1749 2004-01-27 00:01:18Z gabrielt $
//

#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"
#include "mtl/linalg_vec.h"

/*
  
  Sample Output
  
  A in packed form
  [
  [1,],
  [2,1,],
  [3,5,1,],
  [4,6,7,1,],
  ]
  b:
  [8,25,79,167,]
  A^-1 * b:
  [8,9,10,11,]
  
 */

using namespace mtl;


int
main()
{
  //begin

  typedef matrix< double, 
                  triangle<lower>,
                   packed<>, 
                   row_major >::type Matrix;
  typedef external_vec<double> Vector;
  const int N = 3;

  Matrix A(N, N);

  set_diagonal(A, 1);

  //end
  //C         1                7
  //C     A = 2  4       b =  46   b is stored in the
  //C         3  5  7        115   last column of A
  //begin
  //Fill the matrix...

  A(1,0) = 2;  A(1,1) = 4;
  A(2,0) = 3;  A(2,1) = 5; A(2,2) = 6;

  double db[] = { 7, 46, 115};
  Vector b(db, N);

  std::cout << "A in packed form" << std::endl;
  print_row(A);

  std::cout << "b:" << std::endl;
  print_vector(b);

  tri_solve(A, b);

  std::cout << "A^-1 * b:" << std::endl;

  print_vector(b);

  //end

  return 0;
}
  
