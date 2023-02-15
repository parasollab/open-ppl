#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"
#include "mtl/linalg_vec.h"

/*
  
  Sample Output
  A:
  [
  [1,],
  [2,3,],
  [0,4,5,],
  [0,0,6,7,],
  ]
  b:
  [8,43,86,137,]
  A^-1 * b:
  [8,9,10,11,]

 */

using namespace mtl;

typedef matrix< double, triangle<lower>, banded<>, row_major>::type Matrix;
//typedef dense1D<double> Vector;
typedef external_vec<double> Vector;

int
main()
{
  const int N = 4;

  Matrix A(N, N);

  set_diagonal(A, 1);

  //C         1                   8
  //C     A = 2  3          b =  43
  //C            4  5            86
  //C               6  7        137

  A(1,1) = 3; A(2,2) = 5; A(3,3) = 7;
  A(1,0) = 2; A(2,1) = 4; A(3,2) = 6;
  

  double db[] = { 8, 43, 86, 137 };
  Vector b(db, N);

  std::cout << "A:" << std::endl;
  print_row(A);

  std::cout << "b:" << std::endl;
  print_vector(b);

  tri_solve(A, b);

  std::cout << "A^-1 * b:" << std::endl;

  print_vector(b);

  return 0;
}
  
