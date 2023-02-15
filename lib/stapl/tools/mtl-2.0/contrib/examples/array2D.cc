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
//===========================================================================

#include <iostream>
#include "mtl/mtl.h"
#include "mtl/matrix.h"

using namespace mtl;


int
main()
{
  const int M = 4, N = 3;

  //begin
  typedef matrix< double, 
                  rectangle<>, 
                  array< dense<> >, 
                  row_major>::type MatA;

  typedef matrix< double, 
                  rectangle<>, 
                  array< compressed<> >, 
                  row_major>::type MatB;

  typedef matrix< double, 
                  rectangle<>, 
                  array< sparse_pair >, 
                  row_major>::type MatC;
  std::cout << "start" << std::endl;
  MatA A(M,N);
  MatB B(M,N);
  MatC C(M, N);
  // Fill A ...
  //end
  for (int i = 0; i < M; ++i)
    for (int j = 0; j < N; ++j)
      A(i,j) = i * N + j;
  //begin
  mtl::copy(A, B); // JGS, this is broken!
#if 0
  MatB::Row tmp = B[2];
  B[2] = B[3];
  B[3] = tmp;

  mtl::copy(B, C);
  //end
#endif
  mtl::print_all_matrix(A);
  mtl::print_all_matrix(B);
  mtl::print_all_matrix(C);

  return 0;
}
