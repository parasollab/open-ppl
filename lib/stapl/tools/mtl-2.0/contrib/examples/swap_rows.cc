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
// $Id: swap_rows.cc 1749 2004-01-27 00:01:18Z gabrielt $
//

#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h" 

/*
  Swaps the first row with the row that has
  the largest element in column 1.

  Sample Output

  3x3
  [
  [1,1.5,4.5],
  [3,2.5,9.5],
  [2,3.5,5.5]
  ]
  3x3
  [
  [3,2.5,9.5],
  [1,1.5,4.5],
  [2,3.5,5.5]
  ]

 */

using namespace mtl;

int
main()
{
  //begin
  typedef matrix< double,
                  rectangle<>, 
                  dense<external>,
                  column_major>::type Matrix; 
  const Matrix::size_type N = 3;
  Matrix::size_type large;
  double dA[] = { 1, 3, 2, 1.5, 2.5, 3.5, 4.5, 9.5, 5.5 };
  Matrix A(dA, N, N);
  //end
  print_all_matrix(A);
  //begin
  //  Find the largest element in column 1.
  large = max_index(A[0]);
  //end
  std::cout << "x" << std::endl;
  //begin
  // Swap the first row with the row containing the largest
  // element in column 1.
  swap( rows(A)[0] , rows(A)[large]);
  //end
  print_all_matrix(A);

  return 0;
}
