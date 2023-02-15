#include <mtl/matrix.h>
#include <mtl/mtl.h>

using namespace mtl;

int main()
{
  //  matrix<double, rectangle<>, array< dense<> >, row_major>::type A(10,10);
  matrix<double, rectangle<>, dense<>, row_major>::type A(10,10);
  mtl::set_value(A, 2);

  A.resize(5,4);
  print_all_matrix(A);

  A.resize(10, 10);
  print_all_matrix(A);

  return 0;
}
