#include <mtl/dense1D.h>
#include <mtl/mtl.h>
#include <mtl/utils.h>


/*
  example output:

  [7,7,7,7,7,7,7,7,7,7]

  */

int
main()
{
  using namespace mtl;
  //begin
  dense1D< double > x(10,2);
  dense1D< double > y(10,3);
  add(scaled(x,2), y, y);
  print_vector(y);
  //end
  return 0;
}
