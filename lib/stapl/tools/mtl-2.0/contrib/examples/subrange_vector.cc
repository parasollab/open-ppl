#include <mtl/mtl.h>
#include <mtl/utils.h>
#include <mtl/dense1D.h>

/*
  Sample output:

  Matrix A:
  5x5
  [
  [0,1,2,3,4],
  [5,6,7,8,9],
  [10,11,12,13,14],
  [15,16,17,18,19],
  [20,21,22,23,24]
  ]
  Vector x: [0,1,2,3,4,5,6,7,8,9,]
  Vector x(3,8): [3,4,5,6,7,]
  Vector y=A*x: [60,185,310,435,560,]

 */

int
main(int, char*[])
{
  using namespace mtl;

  dense1D<double> x(10), y(10);

  for (int i = 0; i < 10; ++i)
    x[i] = i;

  matrix<double>::type A(5,5);
  for (int j = 0; j < 5; ++j)
    for (int k = 0; k < 5; ++k)
      A(j,k) = j * 5 + k;

  std::cout << "Matrix A:" << std::endl;
  print_all_matrix(A);
  std::cout << "Vector x: ";
  print_vector(x);
  std::cout << "Vector x(3,8): ";
  print_vector(x(3,8));

  mult(A, x(3,8), y(3,8));

  std::cout << "Vector y=A*x: ";
  print_vector(y(3,8));
  
  return 0;
}
