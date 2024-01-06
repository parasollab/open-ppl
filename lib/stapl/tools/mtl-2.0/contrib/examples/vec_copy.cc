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
// $Id: vec_copy.cc 1749 2004-01-27 00:01:18Z gabrielt $
//


#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"  
#include "mtl/linalg_vec.h"

/*
  Transpose matrix A using vec::copy.

  Sample Output

  3x3
  [
  [1,4,7],
  [2,5,8],
  [3,6,9]
  ]
  3x3
  [
  [1,2,3],
  [4,5,6],
  [7,8,9]
  ]
  
  */

using namespace mtl;

//typedef dense2D<double> TwoD;
//typedef column<TwoD> Matrix;
typedef dense1D<double> Vec;
typedef external_vec<double> EVec;
typedef matrix< double, rectangle<>, dense<>, column_major>::type Matrix;  

int
main()
{
  int i, j;
  const int N = 3;
  double da[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
  EVec a(da,N*N);
  Vec b(N*N);

  Matrix A(N, N);

  //temp
  int k=1;
  for (i=0;i<N;++i)
    for (j=0;j<N;++j)
      A(j,i) = k++;

  print_all_matrix(A);

  mtl::set_value(b, 0);

  for (i = 0; i < N; ++i) {
    EVec b_(&b[i], N*N);
    EVec a_(&a[i*N], N);
    copy(a_, strided(b_,N));
    //print_vector(a_);
  }

  //TwoD twodB(b.data(), N, N);
  Matrix B(N,N);
  //tmp
  Vec::iterator it = b.begin();    
  for (i=0;i<N;++i)
    for (j=0;j<N;++j)
      B(j,i) = *it++;
      
  print_all_matrix(B);
}

