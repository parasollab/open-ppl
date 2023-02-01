#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"  
#include "mtl/linalg_vec.h"

/*
Sample output
A =
5x5
[
[1,2,0,0,0],
[2,3,4,0,0],
[0,4,5,6,0],
[0,0,6,7,8],
[0,0,0,8,9]
]
x =
[1,2,3,4,5,]
y =
[1,1,1,1,1,]
Ax + y =[13,43,97,175,157,]   
 */

using namespace mtl;

int
main()
{
  //begin
  typedef matrix< double, symmetric<lower>, banded<>, row_major>::type Matrix; 
  typedef dense1D<double> Vec;
  typedef Matrix::size_type Int;
  const Int matrix_size = 5;
  const Int band_size = 1;
  Matrix A(matrix_size, band_size);
  Vec y(matrix_size,1),x(matrix_size);
  double alpha=2, beta=3;
  //            1  2                 1        1
  //            2  3  4              2        1
  //       A =     4  5  6       x = 3    y = 1
  //                  6  7  8        4        1
  //                     8  9        5        1

  //make A
  Int r = 0, i = 0;
  for (r = 0; r < matrix_size - 1; ++r){
    A(r,r) = ++i;
    A(r,r+1) = ++i;
  }
  A(r,r) = ++i;
  //make x
  for (i=0;i<matrix_size;++i)
    x[i]=i+1;
  //end
  std::cout << "A =" << std::endl;
  //print_all_matrix(A);
  print_all_banded(A, band_size, band_size);
  std::cout << "x =" << std::endl;
  print_vector(x);
  std::cout << "y =" << std::endl;
  print_vector(y);
  //begin
  mult(A,scaled(x,alpha),scaled(y,beta),y);
  //end
  std::cout << "Ax + y =";
  print_vector(y);
}

