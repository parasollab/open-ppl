#include "mtl/matrix.h"
#include "mtl/mtl.h"
//#include "mtl/sparse_mult.h"
#include "mtl/utils.h"

int random(int range) {
  return rand() % (range -1) + 1;
}

int
main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "sparse_mult_col <N>" << std::endl;
    return -1;
  }
  using namespace mtl;

  typedef double T;

  typedef matrix<T, rectangle<>, 
                 compressed<>, 
                 column_major>::type  SparseMatrix;

  typedef matrix<T, rectangle<>, dense<>, 
                 column_major>::type  DenseMatrix;

  int N = atoi(argv[1]);
  SparseMatrix A(N, N);
  SparseMatrix B(N, N);
  SparseMatrix C(N, N, N*N); // N*N is a hint for how many nnz there will be

  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N/10; ++j) {
      A(i, random(N)) = T(random(5));
      B(i, random(N)) = T(random(5));
    }
  }

  DenseMatrix D(N, N);
  DenseMatrix E(N, N);
  DenseMatrix F(N, N);

  mtl::set_value(D, 0);
  mtl::set_value(E, 0);
  mtl::set_value(F, 0);

  copy(A, D);
  copy(B, E);

  mult(D, E, F);
  
  mult(A, B, C);

  if (matrix_equal(C, F))
    std::cout << "success" << std::endl;
  else {
    std::cout << "failure" << std::endl;
    print_column(C);
    print_column(F);
  }

  std::cout << "C.nnz(): " << C.nnz() << std::endl;
  return 0;
}
