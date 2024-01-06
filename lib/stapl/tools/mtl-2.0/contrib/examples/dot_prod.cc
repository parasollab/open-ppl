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
// $Id: dot_prod.cc 1749 2004-01-27 00:01:18Z gabrielt $
//

#include <iostream>
#include "mtl/mtl.h"
#include "mtl/utils.h" 
#include "mtl/linalg_vec.h"


/*

  Sample Output


  [1,2,3,]
  [3,0,-1,]
  Vectors x and y are orthogonal.

  */

using namespace mtl;


int
main()
{
  //begin
  const int N = 3;
  double dx[] = { 1, 2, 3};
  double dy[] = { 3, 0, -1};
  typedef external_vec<double> Vec;
  Vec x(dx,N), y(dy,N);

  print_vector(x);
  print_vector(y);

  double dotprd = dot(x, y);

  if (dotprd == 0)
    std::cout << "Vectors x and y are orthogonal." << std::endl;
  else
    std::cout << "dot(x,y) = " << dotprd << std::endl;
  //end
  return 0;
}

