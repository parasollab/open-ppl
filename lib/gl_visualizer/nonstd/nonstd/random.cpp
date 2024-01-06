#include <chrono>
#include <ctgmath>

#include "nonstd/random.h"

using namespace std;

namespace nonstd {

  mt19937
  random_generator(chrono::system_clock::now().time_since_epoch().count());

  double
  drand()
  {
    static uniform_real_distribution<double> distribution(0., 1.);
    return distribution(random_generator);
  }


  size_t
  lrand()
  {
    static uniform_int_distribution<size_t> distribution(0,
        std::numeric_limits<size_t>::max());
    return distribution(random_generator);
  }

}
