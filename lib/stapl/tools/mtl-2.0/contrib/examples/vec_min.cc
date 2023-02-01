#include <iostream>
#include <mtl/dense1D.h>
#include <mtl/mtl.h>


/*
  example output:

  2

  */

int
main()
{
  //begin
  mtl::dense1D< float > x(10, 3.0);
  x[5] = 2.0;
  float s = mtl::min(x);
  std::cout << s << std::endl;
  // end
  return 0;
}

