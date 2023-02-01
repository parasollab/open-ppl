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
// $Id: max_index.cc 1749 2004-01-27 00:01:18Z gabrielt $
//

#include "mtl/mtl.h"
#include "mtl/utils.h" 
#include "mtl/linalg_vec.h"

using namespace mtl;
//begin
typedef complex<float> c;
typedef external_vec<c> Vec;
//end

/*
  
  Note: the Sun Perf Lib docs show the correct answer
  as item 1. This is incorrect. The correct answer
  is item 5.cvazac

  abs( (5,-4) ) = sqrt(5^2 + 4^2) = 6.40312
  abs( (0, 8) ) = 8

  Sample Output 
  [(5,-4),(-3,-2),(5,-4),(6,0),(0,8),]
  Largest element in the vector x is item 5

 */

int
main()
{
  //begin
  const int N = 5;
  c dx[] = { c(5,-4), c(-3,-2),
             c(5,-4), c(6,0), c(0,8) };
  Vec x(dx, N);
  
  int imax = max_index(x);
  //end

#if !defined(_MSVCPP_) && !defined(__sgi) || defined(__GNUC__)
  // VC++ and SGI CC do not have operator << defined for complex numbers
  print_vector(x);
#endif

  std::cout << "Largest element in the vector x is item " << imax + 1 << std::endl;
  return 0;
}

