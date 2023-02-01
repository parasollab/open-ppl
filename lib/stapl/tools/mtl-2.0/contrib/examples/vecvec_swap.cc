#include <mtl/dense1D.h>
#include <mtl/mtl.h>
#include <mtl/utils.h>

/*
  example output:

  [2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1,2,1]
  [1,1,1,1,1,1,1,1,1,1]

  */

int
main()
{
  using namespace mtl;
  //begin
  dense1D<double> x(20,1), y(10,2);
  swap(strided(x,2), y);
  //end
  print_vector(x);
  print_vector(y);
  return 0;
}
