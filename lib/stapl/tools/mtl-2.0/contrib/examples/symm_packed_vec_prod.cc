#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"  
#include "mtl/linalg_vec.h"

/*
  Sample output
  A =
  5x5
  [
  [1,2,3,4,5],
  [2,6,7,8,9],
  [3,7,10,11,12],
  [4,8,11,13,14],
  [5,9,12,14,15]
  ]
  x =
  [1,2,3,4,5,]
  y =
  [1000,2000,3000,4000,5000,]
  Ax + y =[55,112,151,175,190,]
  
  */

using namespace mtl;


int
main()
{
  unsigned int i;
  //begin
  double da[16];
  typedef matrix< double,
                  symmetric<lower>, 
                  packed<external>,
                  column_major >::type Matrix;
  const Matrix::size_type matrix_size = 5;
  Matrix A(da, matrix_size, matrix_size);
  typedef dense1D<double> Vec;
  Vec y(matrix_size,1),x(matrix_size), Ax(matrix_size);
  double alpha=1, beta=0;
  //          1   2   3   4   5        1        1000
  //          2   6   7   8   9        2        2000
  //     A =  3   7  10  11  12    x = 3    y = 3000
  //          4   8  11  13  14        4        4000
  //          5   9  12  14  15        5        5000

  //make A
  for (i = 0; i < 15; ++i)
    da[i] = i + 1;
  //make x y
  for ( i = 0; i < matrix_size; ++i){
    y[i]=1000*(i+1);
    x[i]=i+1;
  }
  //end

  std::cout << "A =" << std::endl;
  print_all_matrix(A);
  std::cout << "x =" << std::endl;
  print_vector(x);
  std::cout << "y =" << std::endl;
  print_vector(y);
  //begin
  mult(A, scaled(x,alpha), scaled(y,beta), y);
  //end
  std::cout << "Ax + y =";
  print_vector(y);
}


