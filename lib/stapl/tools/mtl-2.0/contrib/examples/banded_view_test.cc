#include <iostream>
#include "mtl/matrix.h"
#include "mtl/mtl.h"

//begin
template <class Matrix>
void print_banded_views(Matrix& A)
{
  using namespace mtl;
//end
  const int M = A.nrows();
  const int N = A.nrows();
  
  std::cout << "full matrix" << std::endl;
  print_all_matrix(A);
  std::cout << std::endl;
  
  typedef typename rows_type<Matrix>::type RowMatrix;
  
  std::cout << "rows banded" << std::endl;
//begin
  typename band_view<RowMatrix>::type B(2, 1, A);
//end
  print_all_banded(B, 2, 1);
  print_row(B);
  std::cout << std::endl;

  std::cout << "columns banded" << std::endl;
  typedef typename columns_type<Matrix>::type ColMatrix;
  typename band_view<ColMatrix>::type C(2, 1, columns(A));
  print_all_banded(C, 2, 1);
  print_column(C);
  std::cout << std::endl;

  std::cout << "rows lower triangle" << std::endl;
  typename triangle_view<RowMatrix, lower>::type L(A);
  print_all_banded(L, M-1, 0);
  print_row(L);
  std::cout << std::endl;

  std::cout << "rows unit upper triangle" << std::endl;
  typename triangle_view<RowMatrix, unit_upper>::type U(A);
  print_all_banded(U, -1, N-1);
  print_row(U);


  std::cout << "columns lower triangle" << std::endl;
  typename triangle_view<ColMatrix, lower>::type CL(columns(A));
  print_all_banded(CL, M-1, 0);
  print_column(CL);
  std::cout << std::endl;

  std::cout << "columns unit upper triangle" << std::endl;
  typename triangle_view<ColMatrix, unit_upper>::type CU(columns(A));
  print_all_banded(CU, -1, N-1);
  print_column(CU);
//begin
}
//end

//begin
int main(int argc, char* argv[])
{
  using namespace mtl;
  int M, N;
  if (argc < 2) {
    M = 10;
    N = 10;
  } else {
    M = atoi(argv[1]);
    N = atoi(argv[2]);
  }
//end

//begin
  typedef matrix<double>::type Matrix;
//end
  typedef matrix<double, 
                 rectangle<>,
                 dense<>, 
                 column_major>::type ColMatrix;
//begin
  Matrix A(M, N);
//end
  ColMatrix B(M, N);

  for (int j = 0; j < N; ++j)
    for (int i = 0; i < M; ++i)
      A(i,j) = double(i * N + j);

  mtl::copy(A, B);

  std::cout << "Row Matrix ***********" << std::endl;
//begin
  print_banded_views(A);
//end
  std::cout << "Column Matrix ***********" << std::endl;
  print_banded_views(B);

//begin
  return 0;
}
//end
