#include <mtl/dense1D.h>
#include <mtl/mtl.h>
#include <mtl/utils.h>

/*
  example output:

  [6,6,6,6,6,6,6,6,6,6]

  */

int
main()
{
  using namespace mtl;
  //begin
  dense1D<double> x(10,4), y(10,2), z(10);
  ele_div(scaled(x,3), y, z);
  //end
  print_vector(z);
  return 0;
}
