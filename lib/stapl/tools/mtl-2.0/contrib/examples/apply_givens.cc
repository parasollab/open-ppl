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
// $Id: apply_givens.cc 1749 2004-01-27 00:01:18Z gabrielt $
//


#include "mtl/mtl.h"
#include "mtl/utils.h" 
#include "mtl/linalg_vec.h"


/*
  Eliminates the last element of vector y.

  Sample Output

  [1,2,3,4,5,]
  [2,4,8,16,32,]
  [2.1304,4.2608,8.36723,16.4257,32.3883,]
  [-0.679258,-1.35852,-1.72902,-1.48202,0,]

 */

using namespace mtl;

typedef external_vec<double> Vec;

int
main()
{
  //begin
  const int N = 5;
  double dx[] = { 1, 2, 3, 4, 5 };
  double dy[] = { 2, 4, 8, 16, 32};

  Vec x(dx, N), y(dy, N);
  //end
  print_vector(x);
  print_vector(y);
  //begin
  double a = x[N-1];
  double b = y[N-1];

  givens_rotation<double> rot(a, b);

  rot.apply(x, y);
  //end

  print_vector(x);
  print_vector(y);
  
  return 0;
}  


