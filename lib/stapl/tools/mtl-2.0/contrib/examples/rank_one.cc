#include "mtl/matrix.h"
#include "mtl/dense1D.h"
#include "mtl/mtl.h"

/*
  Sample Output

  A:
  4x3
  [
  [100,100,100],
  [100,100,100],
  [100,100,100],
  [100,100,100]
  ]
  x:
  [1,2,3,4,]
  y:
  [1,2,3,]
  A + xy':
  4x3
  [
  [101,102,103],
  [102,104,106],
  [103,106,109],
  [104,108,112]
  ]
  */

int
main()
{
  using namespace mtl;
  //begin
  typedef matrix< double,
                  rectangle<>,
                  dense<>,
                  column_major >::type Matrix;
  typedef Matrix::size_type sizeT;
  const sizeT M = 4, N = 3;
  Matrix A(M, N);
  dense1D<double> x(M), y(N);
  double alpha = 1.0;
  // fill A, x, and y ...
  //end
  mtl::set_value(A, 100);
  sizeT i;
  for (i = 0; i < x.size(); ++i)
    x[i] = i+1;
  for (i = 0; i < y.size(); ++i)
    y[i] = i+1;

  std::cout << "A:" << std::endl;
  print_all_matrix(A);
  std::cout << "x:" << std::endl;
  print_vector(x);
  std::cout << "y:" << std::endl;
  print_vector(y);
  //begin
  rank_one_update(A, scaled(x, alpha), y);
  //end
  std::cout << "A + xy':" << std::endl;
  print_all_matrix(A);

  return 0;
}

