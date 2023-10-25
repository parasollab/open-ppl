#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"
#include "mtl/linalg_vec.h" 

/*
Sample Outut:
Matrix A:4x3
[
[100,100,100],
[100,100,100],
[100,100,100],
[100,100,100]
]
Vector X:[1,2,3,4,]
Vector Y:[1,2,3,]
A + xy':4x3
[
[101,102,103],
[102,104,106],
[103,106,109],
[100,100,100]
]

 */

using namespace mtl;  

using namespace mtl;
typedef matrix< double, rectangle<>, dense<>, column_major>::type Matrix;
typedef dense1D<double> Vec;
                                     

int
main()
{
  const int r = 4;
  const int c = 3;
  const double alpha = 1;
  Matrix A(r,c);
  Vec y(c),x(r);

  //initialize the arrays A and Y
  for (int j = 0; j < c; ++j){
    y[j] = j+1;
    for (int i = 0; i < r; ++i)
      A(i,j) = 100;
  }
  //initialize X
  for (int i=0;i<r;++i)
    x[i] = i+1;

  //print a x y
  std::cout << "Matrix A:";
  print_all_matrix(A);
  std::cout << "Vector X:";
  print_vector(x);
  std::cout << "Vector Y:";
  print_vector(y);

  //do the update
  rank_one_update(A, scaled(x, alpha), y);
 
  //print result
  std::cout << "A + xy':";
  print_all_matrix(A);
}

