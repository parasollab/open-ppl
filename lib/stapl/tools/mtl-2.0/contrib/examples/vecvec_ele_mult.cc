#include <mtl/dense1D.h>
#include <mtl/mtl.h>
#include <mtl/utils.h>


/*
  example output:

  [12,12,12,12,12,12,12,12,12,12]

  */

int
main()
{
  using namespace mtl;
  //begin
  dense1D<double> x(20,1), y(10,4), z(10);
  ele_mult(strided(scaled(x,3),2), y, z);
  //end
  print_vector(z);
  return 0;
}

