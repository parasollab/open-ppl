#include <iostream>
#include <mtl/dense1D.h>
#include <mtl/mtl.h>

/*

  example output:

  1

  */

int
main()
{
  //begin
  mtl::dense1D<float> x(10, 0.0);
  x[5] = 1.0;
  float s = mtl::max(x);
  //end
  std::cout << s << std::endl;
  return 0;
}

