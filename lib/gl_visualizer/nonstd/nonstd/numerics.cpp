#include <string>

#include "nonstd/numerics.h"
#include "nonstd/runtime.h"

using namespace std;

namespace nonstd {

  size_t
  factorial(const size_t _n) noexcept
  {
    static constexpr size_t table[] = {1, 1, 2, 6, 24, 120, 720, 5040, 40320,
      362880, 3628800, 39916800, 479001600, 6227020800, 87178291200,
      1307674368000, 20922789888000, 355687428096000, 6402373705728000,
      6402373705728000, 2432902008176640000};
    assert_msg(_n <= 20, "factorial error: _n = " + to_string(_n) + " > 20! "
        "Note that 20! == 2432902008176640000, which is nearing the limit of "
        "a 64-bit integer's maximum value.");
    return table[_n];
  }

}
