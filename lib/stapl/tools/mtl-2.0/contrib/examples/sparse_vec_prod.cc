#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"  
#include "mtl/linalg_vec.h"

/*
  Sample output:
  A =
  3x3
  [
  [1,0,2],
  [0,3,0],
  [0,4,5]
  ]
  x =
  [1,2,3,]
  y =
  [1,1,1,]
  Ax + y =[17,15,49,]
  
 */

using namespace mtl;
typedef matrix< double, rectangle<>, compressed<>, row_major>::type Matrix; 
typedef dense1D<double> Vec;


int
main()
{
  const int matrix_size=3;
  Matrix A(matrix_size,matrix_size);
  Vec y(matrix_size,1),x(matrix_size);
  double alpha=2, beta=3;
  int i;

  //make A
  A(0,0) = 1;  A(0,2) = 2;
  A(1,1) = 3;
  A(2,1) = 4;  A(2,2) = 5;

  //make x
  for (i=0;i<matrix_size;++i)
    x[i]=i+1;

  std::cout << "A =" << std::endl;
  print_all_matrix(A);
  std::cout << "x =" << std::endl;
  print_vector(x);
  std::cout << "y =" << std::endl;
  print_vector(y);

  mult(A,scaled(x,alpha),scaled(y,beta),y);

  std::cout << "A*3x + 2y =";
  print_vector(y);
}

