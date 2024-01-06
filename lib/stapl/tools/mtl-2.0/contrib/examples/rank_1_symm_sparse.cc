#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"
#include "mtl/linalg_vec.h" 

/*
  Expected output:
  
Matrix A:
3x3
[
[100,200,300],
[200,400,500],
[300,500,600]
]
Vector X:
[1,2,3,]
a * x * x^T + A:3x3
[
[101,202,303],
[202,404,506],
[303,506,609]
]

*/


using namespace mtl;
typedef matrix< double , symmetric<lower>, 
                compressed<int,external, index_from_one>, column_major>::type Matrix;
typedef dense1D<double> Vec;
                                     

int
main()
{
  const int N = 3;
  const double alpha = 1;
  //        100  200  300        1
  //    A = 200  400  500    x = 2
  //        300  500  600        3

  double values[] = { 100, 200, 300, 200, 400, 500, 300, 300, 600 };
  int ptrs[] = { 1, 4, 7, 10};
  int indices[] = { 1, 2, 3, 1, 2, 3, 1, 2, 3};
  
  Matrix A(N, N, N*N, values, ptrs, indices);
  Vec y(N),x(N);

  //initialize X
  for (int i=0;i<N;++i)
    x[i]=i+1;

  //print a x y
  std::cout << "Matrix A:" << std::endl;
  print_all_matrix(A);
  std::cout << "Vector X:" << std::endl;
  print_vector(x);

  //do the update
  rank_one_update(A, scaled(x, alpha), x);
 
  //print result
  std::cout << "a * x * x^T + A:";
  print_all_matrix(A);
}

