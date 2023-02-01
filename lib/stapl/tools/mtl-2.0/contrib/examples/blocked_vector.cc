#include "mtl/mtl_config.h"

#ifdef MTL_DISABLE_BLOCKING
#include <iostream>
#else
#include "mtl/block1D.h"
#include "mtl/linalg_vec.h"
#include "mtl/utils.h"
#endif

int
main()
{
#ifdef MTL_DISABLE_BLOCKING
  std::cout << "Blocking unsupported for this compiler" << std::endl;
#else
  using namespace mtl;
  using namespace fast;

  const int N = 9;
  typedef dense1D<int> Vec;
  Vec x(N);

  for (int i = 0; i < N; ++i)
    x[i] = i;

  block1D<Vec,3> bx(x);

  print_partitioned_vector(bx);
#endif
}
