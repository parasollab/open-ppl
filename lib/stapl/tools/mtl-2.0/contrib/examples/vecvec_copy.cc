#include <mtl/dense1D.h>
#include <mtl/mtl.h>
#include <mtl/utils.h>

/*
  example output:

  [4,4,4,4,4,4,4,4,4,4]

  */

int
main()
{
  using namespace mtl;
  //begin
  dense1D<double> x(10,1);
  dense1D<double> y(10);
  double alpha = 4.0;
  mtl::copy(scaled(x, alpha), y);
  //end
  print_vector(y);
  return 0;
}
