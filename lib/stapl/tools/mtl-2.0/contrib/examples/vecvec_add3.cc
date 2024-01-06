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
  dense1D<double> x(20,2), y(20,3), z(10,1), w(10);
  add(strided(x,2), strided(y,2), z, w);
  //end
  print_vector(w);
  return 0;
}
