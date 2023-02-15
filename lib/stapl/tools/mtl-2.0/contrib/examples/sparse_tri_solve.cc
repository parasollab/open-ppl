#include "mtl/matrix.h"
#include "mtl/mtl.h"
#include "mtl/utils.h"
#include "mtl/linalg_vec.h"

/*
  A:
  5x5
  [
  [1,1,3,0,0],
  [0,1,5,0,0],
  [0,0,1,0,0],
  [0,0,0,1,0],
  [0,0,0,0,1]
  ]
  b:
  [1,10,7,46,115,]
  A^-1 * b:
  [5,-25,7,46,115,]
 */

using namespace mtl;


int
main()
{
  //begin

  typedef matrix< double, 
                  triangle<unit_upper>,
                  array<compressed<> >,
                  column_major >::type Matrix;
  typedef external_vec<double> Vector;
  const int N = 5;

  Matrix A(N, N);

  A(0,1) = 1; 
  A(0,2) = 3; A(1,2) = 5; 


  double db[] = {1, 10, 7, 46, 115};
  Vector b(db, N);

  std::cout << "A:" << std::endl;
  print_all_matrix(A);

  std::cout << "b:" << std::endl;
  print_vector(b);

  tri_solve(A, b);


  std::cout << "A^-1 * b:" << std::endl;

  print_vector(b);

  //end

  return 0;
}
  
