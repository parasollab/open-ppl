#include "mtl/matrix.h"
#include "mtl/utils.h"

int
main()
{
  using namespace mtl;
  typedef matrix<double, rectangle<>, 
                 dense<>, row_major>::type Matrix;
  typedef Matrix::size_type sizeT;

  const sizeT M = 4, N = 4;
  Matrix A(M, N);
  for (sizeT i = 0; i < M; ++i)
    for (sizeT j = 0; j < N; ++j)
      A(i,j) = i * N + j;

  std::cout << "Matrix A" << std::endl;
  print_all_matrix(A);

  std::cout << "columns(A)[1](1,3)" << std::endl;
  print_vector(columns(A)[1](1,3));

  std::cout << "rows(A)[1](1,3)" << std::endl;
  print_vector(rows(A)[1](1,3));

  return 0;
}
