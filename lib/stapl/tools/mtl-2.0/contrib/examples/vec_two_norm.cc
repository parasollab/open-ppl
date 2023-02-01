#include <iostream>
#include "dense1D.h"
#include "mtl.h"

/*
  example output:

  6.32456

  */

int
main()
{
  using namespace mtl;
  //begin
  dense1D<double> x(10, 2.0);
  double s = two_norm(x);
  std::cout << s << std::endl;
  //end
  return 0;
}
