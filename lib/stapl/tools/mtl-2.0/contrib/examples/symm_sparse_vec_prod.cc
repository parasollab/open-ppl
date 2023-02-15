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
typedef matrix< double, symmetric<lower>, 
                compressed<>, row_major>::type Matrix; 
typedef dense1D<double> Vec;



int
main()
{
  int i;
  //begin
  const Matrix::size_type matrix_size=5;
  Matrix A(matrix_size,matrix_size);
  Vec y(matrix_size,1),x(matrix_size);
  double alpha=2, beta=3;
  //fill arrays...
  //end
  //            1  2                 1        1
  //            2  3  4              2        1
  //       A =     4  5  6       x = 3    y = 1
  //                  6  7  8        4        1
  //                     8  9        5        1

  //mtl::set(A, 0);

  //make A
  int r=0;
  i=0;
  for (r=0;r<matrix_size;++r){
    A(r,(r)) = ++i;    
    if ( r+1 < matrix_size) 
      A(r,(r+1)) = ++i;
  }

  //make x
  for (i=0;i<matrix_size;++i)
    x[i]=i+1;

  std::cout << "A =" << std::endl;
  print_all_matrix(A);
  std::cout << "x =" << std::endl;
  print_vector(x);
  std::cout << "y =" << std::endl;
  print_vector(y);
  //begin
  mult(A,scaled(x,alpha),scaled(y,beta),y);

  std::cout << "Ax + y =";
  print_vector(y);
  //end
}

