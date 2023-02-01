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
// $Id: geev.cc 1749 2004-01-27 00:01:18Z gabrielt $
//


#include <complex>
#include "mtl/mtl2lapack.h"
#include "mtl/dense1D.h"
#include "mtl/utils.h"


/*
  Sample Output

  2x2
  [
  [0,1],
  [-6,5]
  ]
  eigenvalues
  [(2,0),(3,0),]
  eigenvectors
  4x2
  [
  [0.447214,0.316228],
  [0.894427,0.948683],
  [0,0],
  [0,0]
  ]

  */

int
main()
{
  using namespace mtl2lapack;

  const int N = 2;
  //begin
  double da [] = { 0, -6, 1, 5 };
  lapack_matrix<double,external>::type A(da, N,N), vl((double*)0,N,N);
  lapack_matrix<double>::type vr(2*N,N);
  mtl::dense1D< std::complex<double> > wr(N);
  int info;
  //end
  mtl::print_all_matrix(A);

  // Compute the eigenvalues and right eigenvectors of A.
  //begin
  info = geev(GEEV_CALC_RIGHT, A, wr, vl, vr);
  //end
  if (info > 0) {
    std::cout << "QR failed to converge, INFO = " << info << std::endl;
    return 0;
  }

  // Print the eigenvalues and eigenvectors.

  std::cout << "eigenvalues" << std::endl;
  mtl::print_vector(wr);

  std::cout << "eigenvectors" << std::endl;

  mtl::print_all_matrix(vr);

  return 0;
}

