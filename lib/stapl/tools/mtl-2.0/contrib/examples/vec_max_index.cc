#include <iostream>
#include <mtl/mtl.h>
#include <mtl/dense1D.h>

using namespace mtl;

/*
  example output:

  5

  */

int
main()
{
  //begin
  mtl::dense1D<float> x(10, 0.0);
  x[5] = 1.0;
  int i = max_index(x);
  std::cout << i << std::endl;
  //end
  return 0;
}

