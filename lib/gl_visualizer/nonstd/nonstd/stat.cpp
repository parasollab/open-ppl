#include "nonstd/stat.h"

using namespace std;

namespace nonstd {

  /*------------------------ Binomial Distribution ---------------------------*/

  binomial_distribution::
  binomial_distribution(size_t _n, double _p)
    : n(_n), p(_p)
  {
    assert_msg(p >= 0, "nonstd::binomial_distribution error: specified "
        "probability " + to_string(p) + " < 0!");
    assert_msg(p <= 1, "nonstd::binomial_distribution error: specified "
        "probability " + to_string(p) + " > 1!");
  }


  size_t
  binomial_distribution::
  num_trials() const noexcept
  {
    return n;
  }


  double
  binomial_distribution::
  probability() const noexcept
  {
    return p;
  }


  double
  binomial_distribution::
  probability(size_t _r) const noexcept
  {
    return combinations(n, _r) * pow(p, _r) * pow(1 - p, n - _r);
  }


  double
  binomial_distribution::
  mean() const noexcept
  {
    return n * p;
  }


  double
  binomial_distribution::
  variance() const noexcept
  {
    return n * p * (1 - p);
  }


  double
  binomial_distribution::
  std_dev() const noexcept
  {
    return sqrt(n * p * (1 - p));
  }

  /*--------------------------------------------------------------------------*/

}
