#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"  
#include "mtl/linalg_vec.h"

/*

  Sample Output

Array A:
4x4
[
[1,0,0,0],
[2,4,0,0],
[3,5,7,0],
[0,6,8,9]
]
Vector y
[4,3,2,1,]
Ay
[4,20,41,43,]

  */

using namespace mtl;

typedef matrix< double, triangle<lower>, banded<>, row_major>::type Matrix; 
typedef dense1D<double> Vector;

int
main()
{
  const int N = 4;
  
  Matrix A(N, N);

  Vector y(N),x(N);

  //C         1                 4
  //C     A = 2  4          y = 3
  //C         3  5  7           2
  //C            6  8  9        1

  A(0,0) = 1;
  A(1,0) = 2; A(1,1) = 4;
  A(2,0) = 3; A(2,1) = 5; A(2,2) = 7;
              A(3,1) = 6; A(3,2) = 8; A(3,3) = 9;


  for (int i=0;i<4;++i)
    y[i] = 4-i;

  std::cout << "Array A:" << std::endl;
  print_all_matrix(A);

  std::cout << "Vector y" << std::endl;
  print_vector(y);

  mult(A, y, x);

  std::cout << "Ay" << std::endl;
  print_vector(x);

  return 0;
}

