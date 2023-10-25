#include <iostream>
#include "mtl/mtl.h"
#include "mtl/utils.h"
#include "mtl/linalg_vec.h"

/*
  Sample output;

X: [1,5,3,1,]
The L-2 norm of X is 6   
 */

using namespace mtl;

typedef external_vec<double> Vec;

int
main()
{
  double data[] = {1,5,3,1};
  Vec x(data, 4);
 
  double s = two_norm(x);

  std::cout << "X: ";
  print_vector(x);

  std::cout << "The L-2 norm of X is " << s << std::endl;
}

