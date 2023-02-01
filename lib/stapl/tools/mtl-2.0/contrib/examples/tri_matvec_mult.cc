#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"  
#include "mtl/linalg_vec.h"

/*

  Sample Output

Array A:
3x3
[
[1,2,4],
[2,4,3],
[3,5,6]
]
Vector y
[7,8,9,]
Ay
[7,46,115,]

  */

using namespace mtl;


typedef matrix< double, triangle<lower>, packed<>, row_major>::type Matrix; 
typedef dense1D<double> Vector;

int
main()
{
  const int N = 3;
  
  Matrix A(N, N);

  Vector y(N), Ay(N);

  //C         1              7
  //C     A = 2  4       y = 8
  //C         3  5  6        9

  A(0,0) = 1;
  A(1,0) = 2; A(1,1) = 4;
  A(2,0) = 3; A(2,1) = 5; A(2,2) = 6;


  for (int i=0;i<N;++i)
    y[i] = 7+i;

  

  std::cout << "Array A:" << std::endl;
  print_all_matrix(A);

  std::cout << "Vector y" << std::endl;
  print_vector(y);

  mult(A, y, Ay);

  std::cout << "Ay" << std::endl;
  print_vector(Ay);

  return 0;
}


