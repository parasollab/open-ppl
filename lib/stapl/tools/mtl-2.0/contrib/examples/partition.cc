#include "mtl/matrix.h"
#include "mtl/linalg_vec.h"
#include "mtl/utils.h"
#include <list>

/*
  Sample output:

 Partitioning Vectors
 
[0,1,2,3,4,5,6,7,8,9,]
split in half
[0,1,2,3,4,]
[5,6,7,8,9,]
split into thirds
[0,1,2,]
[3,4,5,6,]
[7,8,9,]
 
Partitioning Matrices
 
4x4
[
[0,1,2,3],
[4,5,6,7],
[8,9,10,11],
[12,13,14,15]
]
split into four submatrices
Top mat: 2x2
2x2
[
[0,1],
[4,5]
]
2x2
[
[2,3],
[6,7]
]
 
2x2
[
[8,9],
[12,13]
]
2x2
[
[10,11],
[14,15]
]
 
split into six submatrics
Top mat: 3x2
1x2
[
[0,1]
]
1x2
[
[2,3]
]
 
2x2
[
[4,5],
[8,9]
]
2x2
[
[6,7],
[10,11]
]
 
1x2
[
[12,13]
]
1x2
[
[14,15]
]

  
  */



int
main()
{
  using namespace mtl;

  /* Partitioning Vectors */
  std::cout << "Partitioning Vectors" << std::endl << std::endl;
  const int N = 10;
  int dx[N];
  int i, j;

  for (i = 0; i < N; ++i)
    dx[i] = i;

  typedef external_vec<int> Vec;
  typedef dense1D<Vec::subrange_type> PVec;

  Vec x(dx, N);
  print_vector(x);

  std::cout << "split in half" << std::endl;
  PVec xp1(2);
  subdivide(5, x, xp1);
  print_partitioned_vector(xp1);

  std::cout << "split into thirds" << std::endl;
  PVec xp2(3);
  int splits[] = { 3, 7 };
  partition(array_to_vec(splits), x, xp2);
  print_partitioned_vector(xp2);

  /* Partitioning Matrices */
  std::cout << std::endl;
  std::cout << "Partitioning Matrices" << std::endl << std::endl;

  typedef matrix<int, rectangle<>, dense<>, row_major>::type Matrix;

  typedef matrix<Matrix::submatrix_type, 
                 rectangle<>, dense<>, row_major>::type PMatrix;

  const int M = 4;
  Matrix A(M, M);
  for (i = 0; i < M; ++i)
    for (j = 0; j < M; ++j)
      A(i,j) = i * M + j;

  print_all_matrix(A);

  std::cout << "split into four submatrices" << std::endl;
  
  PMatrix Ap1(2,2);
  subdivide(2, 2, A, Ap1);
  print_partitioned_matrix(Ap1);

  std::cout << "split into six submatrics" << std::endl;

  int row_splits[] = { 1, 3 };
  int col_splits[] = { 2 };
  PMatrix Ap2(3,2);
  partition(array_to_vec(row_splits), 
	    array_to_vec(col_splits),
	    A, Ap2);
  print_partitioned_matrix(Ap2);

  return 0;
}
