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
// $Id: general_matvec_mult.cc 1749 2004-01-27 00:01:18Z gabrielt $
//


#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"  
#include "mtl/linalg_vec.h"
#include <math.h>

/*
  Rotates the vector.

  Sample Output

  Original vector: [3,4,]

  Rotated vector:  [-2,1.5,]
  
  */

using namespace mtl;

int
main()
{
  //begin
  typedef matrix< double, 
                  rectangle<2,2>, 
                  dense<external>, 
                  column_major>::type Matrix; 
  typedef dense1D<double> Vector;
  double pi, theta;
  double dA[4];
  Matrix rot(dA);
  Vector vec1(Matrix::N),vec2(Matrix::N);

  pi = 4.0 * atan (1.0);
  theta = pi / 2.0;

  rot(0,0) = cos(theta);
  rot(1,0) = sin(theta);
  rot(0,1) = -sin(theta);
  rot(1,1) = cos(theta);

  vec1[0] = 3.0;
  vec1[1] = 4.0;
  //end
  std::cout << std::endl;
  std::cout << "Original vector: ";
  print_vector(vec1);
  std::cout << std::endl;
  //begin
  mult(rot, scaled(vec1, 0.5), vec2);
  //end
  std::cout << "Rotated vector: ";
  print_vector(vec2);

  return 0;
}
