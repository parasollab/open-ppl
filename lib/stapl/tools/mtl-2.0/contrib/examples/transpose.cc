// -*- C++ -*-

#include <mtl/matrix.h>
#include <mtl/mtl.h>
#include <mtl/utils.h> 

using namespace mtl;

typedef matrix< double, rectangle<>, dense<>, column_major>::type MATRIX; 


int
main (int , char *[])
{
  MATRIX A(3, 5), B(5, 3);
  for (int j = 0; j < 5; ++j)
    for (int i = 0; i < 3; ++i)
      A(i,j) = i * 5 + j;

  transpose(A, B);
  print_all_matrix(A);
  print_all_matrix(B);
  exit (0);
}
