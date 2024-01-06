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
// $Id: external_matrix.cc 1749 2004-01-27 00:01:18Z gabrielt $
//

#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"


int
main()
{
  using namespace mtl;
  typedef matrix< double, rectangle<>, dense<external> >::type Matrix;
  const Matrix::size_type m=4,n=4;
  double da[m*n];
  Matrix A(da, m,n);

  for (unsigned int i = 0; i < (m*n); ++i)
    da[i] = i + 1;

  print_all_matrix(A);
  
  return 0;
}
