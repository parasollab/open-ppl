// -*- c++ -*-
//
// $Id $
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


#include "mtl/matrix.h"
#include "mtl/utils.h"

/*

  Sample Output

  matrix A
  3x3
  [
  [1,0,2],
  [0,3,0],
  [0,4,5]
  ]
  matrix B
  3x3
  [
  [1,0,2],
  [0,3,0],
  [0,4,5]
  ]
  
  */

int
main()
{
  using namespace mtl;
  //begin
  //  [1,0,2]
  //  [0,3,0]
  //  [0,4,5]
  const int m = 3, n = 3, nnz = 5;
  double values[] = { 1, 2, 3, 4, 5 };
  int indices[]   = { 1, 3, 2, 2, 3 };
  int row_ptr[]   = { 1, 3, 4, 6 };

  // Create from pre-existing arrays
  typedef matrix<double, 
                 rectangle<>, 
                 compressed<int, external,
                                 index_from_one>,
                 row_major>::type MatA;

  MatA A(m, n, nnz, values, row_ptr, indices);
  //end
  std::cout << "matrix A" << std::endl;
  print_all_matrix(A);
  //begin  
  // Create from scratch
  typedef matrix<double,
                 rectangle<>, 
                 compressed<>,
                 row_major >::type MatB;
  MatB B(m, n);
  B(0,0) = 1;  B(0,2) = 2;
  B(1,1) = 3;
  B(2,1) = 4;  B(2,2) = 5;
  //end  


  std::cout << "matrix B" << std::endl;
  print_all_matrix(B);

  MatB BB(2,2);
  BB = B;

  std::cout << "matrix BB "  << BB.nrows() << "," << BB.ncols() << std::endl;
  print_all_matrix(BB);

  return 0;
}
