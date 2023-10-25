//-*-c++-*-----------------------------------------------------------------
//
//  A simple LU factorization algorithm written using MTL
//  The example matrix is the same as the getrf example, which
//  is the LAPACK version of LU factorization.
//
//-------------------------------------------------------------------------

#include "mtl/lu.h"
#include "mtl/matrix.h"
#include "mtl/dense1D.h"
#include "mtl/utils.h"

/*
  Sample output:
  
  3x3
  [
  [1,2,2],
  [2,1,2],
  [2,2,1]
  ]
  3x3
  [
  [2,1,2],
  [0.5,1.5,1],
  [1,0.666667,-1.66667]
  ]
  [2,2,3,]

  
  */

int
main()
{
  using namespace mtl;
  //begin
  typedef matrix<double, 
                 rectangle<>, 
                 dense<external>, 
                 column_major>::type Matrix;

  const Matrix::size_type N = 3;
  double da [] = { 1, 2, 2, 2, 1, 2, 2, 2, 1 };

  Matrix A(da, N, N);
  dense1D<int> pivots(N, 0);
  //end
  print_all_matrix(A);
  //begin
  lu_factor(A, pivots);
  //end
  print_all_matrix(A);

  print_vector(pivots);

  return 0;
}
