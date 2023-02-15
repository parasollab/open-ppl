#ifndef NONSTD_RANDOM_H_
#define NONSTD_RANDOM_H_

#include <random>

namespace nonstd {

  ///@name Random
  ///@{

  extern std::mt19937 random_generator; ///< Global generator for all randoms.


  double drand(); ///< Produce a random double in the range [0,1).


  size_t lrand(); ///< Produce a random size_t.

  ///@}

}

#endif
