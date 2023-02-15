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

#include <iostream>
#include <mtl/matrix.h>

int
main()
{
#ifdef MTL_DISABLE_BLOCKING
  std::cout << "Static blocking unsupported for this compiler" << std::endl;
#else
  using namespace mtl;
  //begin
  const int M = 4;
  const int N = 4;
  typedef matrix<double,
                 rectangle<>, 
                 dense<>, 
                 column_major >::type Matrix;
  Matrix A(M,N);

  for (int i = 0; i < M; ++i)
    for (int j = 0; j < N; ++j)
      A(i, j) = i * N + j;
  print_all_matrix(A);

  block_view<Matrix,2,2>::type
                     bA = blocked(A, blk<2,2>());
  print_partitioned_matrix(bA);

  block_view<Matrix>::type cA = blocked(A, 2, 2);
  print_partitioned_by_column(cA);
  //end
  return 0;
#endif
}
