#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"
#include "mtl/linalg_vec.h"

/*
  
  Sample Output
  
  A in packed form
  [
  [1,],
  [2,1,],
  [3,5,1,],
  [4,6,7,1,],
  ]
  b:
  [8,25,79,167,]
  A^-1 * b:
  [8,9,10,11,]
 */

using namespace mtl;

typedef matrix< double, triangle<lower>, packed<>, row_major>::type Matrix;
typedef external_vec<double> Vector;

int
main()
{
  //begin
  const int N = 4;
  Matrix A(N, N);

  set_diagonal(A, 1);

  //C         1.0                         8.0
  //C     A = 2.0  1.0              b =  25.0
  //C         3.0  5.0  1.0              79.0
  //C         4.0  6.0  7.0  1.0        167.0

  A(1,0) = 2; A(2,1) = 5; A(3,2) = 7;
  A(2,0) = 3; A(3,1) = 6;
  A(3,0) = 4;
  //end
  double db[] = { 8, 25, 79, 167 };
  Vector b(db, N);

  std::cout << "A in packed form" << std::endl;
  print_row(A);

  std::cout << "b:" << std::endl;
  print_vector(b);

  tri_solve(A, b);

  std::cout << "A^-1 * b:" << std::endl;

  print_vector(b);

  return 0;
}
  
