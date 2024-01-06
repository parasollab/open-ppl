#include <iostream>
#include "mtl/utils.h"
#include "mtl/mtl.h"
#include "mtl/linalg_vec.h"

/*
  Sample output;

X: [1,2,3,4,]
The L-1 norm of X is 10   
 */

using namespace mtl;

int
main()
{
  double x_[4] = { 1, 2, 3, 4 };  

  double s = one_norm(array_to_vec(x_));

  std::cout << "X: ";
  print_vector(array_to_vec(x_));

  std::cout << "The L-1 norm of X is " << s << std::endl;
}

