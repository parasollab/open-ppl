#include <iostream>
#include <mtl/dense1D.h>
#include <mtl/mtl.h>

/*
  example output:

  60

  */

int
main()
{
  using namespace mtl;
  //begin
  dense1D<double> x(10, 2), y(10, 3);
  double s = dot(x, y);
  //end
  std::cout << s << std::endl;
  return 0;
}
