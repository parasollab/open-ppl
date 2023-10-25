/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cstdlib>
#include <string>
#include <iostream>
#include <random>
#include <algorithm>

#include <stapl/array.hpp>
#include <stapl/views/overlap_view.hpp>
#include <stapl/skeletons/map_reduce.hpp>
#include <stapl/runtime.hpp>

using namespace std;

// Get the digits of the number given at the Project Euler website.
std::vector<int> parse_validation_input()
{
  const string validation_input_string {
    "73167176531330624919225119674426574742355349194934"
    "96983520312774506326239578318016984801869478851843"
    "85861560789112949495459501737958331952853208805511"
    "12540698747158523863050715693290963295227443043557"
    "66896648950445244523161731856403098711121722383113"
    "62229893423380308135336276614282806444486645238749"
    "30358907296290491560440772390713810515859307960866"
    "70172427121883998797908792274921901699720888093776"
    "65727333001053367881220235421809751254540594752243"
    "52584907711670556013604839586446706324415722155397"
    "53697817977846174064955149290862569321978468622482"
    "83972241375657056057490261407972968652414535100474"
    "82166370484403199890008895243450658541227588666881"
    "16427171479924442928230863465674813919123162824586"
    "17866458359124566529476545682848912883142607690042"
    "24219022671055626321111109370544217506941658960408"
    "07198403850962455444362981230987879927244284909188"
    "84580156166097919133875499200524063689912560717606"
    "05886116467109405077541002256983155200055935729725"
    "71636269561882670428252483600823257530420752963450"
  };

  std::vector<int> validation_input_digits(validation_input_string.size());
  transform(validation_input_string.begin(), validation_input_string.end(),
    validation_input_digits.begin(), [](unsigned char c) { return c - '0'; });

  return validation_input_digits;
}

struct rand_gen
{
  mt19937 m_rng;
  typedef uniform_int_distribution<size_t> rng_dist_t;

  rand_gen(unsigned int seed = 42)
    : m_rng(seed + stapl::get_location_id())
  { }

  size_t rand(size_t min, size_t max)
  { return rng_dist_t(min, max)(m_rng); }
};

struct fill_arr
{
  typedef void result_type;

  fill_arr(vector<int> validation_digits = vector<int>())
    : m_validation_digits(std::move(validation_digits))
  { }

  template<typename Ref>
  void operator()(Ref&& elem)
  {
    random_device gen;
    rand_gen r(gen());
    elem = r.rand(0,9);
  }

  template<typename Ref>
  void operator()(Ref&& elem, size_t idx)
  {
    assert(!m_validation_digits.empty());
    elem = m_validation_digits[idx];
  }

  void define_type(stapl::typer& t)
  { t.member(m_validation_digits); }

private:
  vector<int> m_validation_digits;
};

struct comp_prod
{
  typedef long long result_type;

  template<typename Ref>
  result_type operator()(Ref&& inview)
  {
    long long prod = 1;
    for (auto x : inview)
      prod *= x;
    return prod;
  }
};

stapl::exit_code stapl_main(int argc, char *argv[])
{
  bool validate = false;

  if (argc == 1)
    validate = true;
  else if (argc != 3)
  {
    cout << "Usage: " << argv[0] << " n m";
    return EXIT_FAILURE;
  }

  long n, m;
  std::vector<int> validation_digits;
  if (validate)
  {
    validation_digits = parse_validation_input();
    n = validation_digits.size();
    assert(n == 1000);
    m = 13;
  }
  else
  {
    n = atol(argv[1]);
    m = atol(argv[2]);
  }

  stapl::counter<stapl::default_timer> ctr_tot;
  ctr_tot.start();

  stapl::array<int> arr(n);
  auto arr_view = stapl::make_array_view(arr);

  if (validate)
  {
    // Use the input specified at the Project Euler website.
    stapl::map_func(fill_arr(std::move(validation_digits)), arr_view,
      stapl::counting_view(n, 0ul));
  }
  else
  {
    // Generate n random digits of the input number.
    stapl::map_func(fill_arr(), arr_view);

    // Make sure the generated number does not start with zero digit.
    if (arr_view.get_location_id() == 0 && arr_view[0] == 0)
       arr_view[0] = rand_gen{random_device{}()}.rand(1,9);
  }

  auto ov_arr_view = stapl::make_overlap_view(arr_view, 1, 0, m - 1);

  long long max = stapl::map_reduce(comp_prod(), stapl::max<long long>(),
    ov_arr_view);

  float exec_time = ctr_tot.stop();

  bool passed = validate ? max == 23514624000ll : true;

  stapl::do_once([n, m, max, exec_time, validate, passed] {
    std::cout << "N Value:         " << n << '\n';
    std::cout << "M Value:         " << m << '\n';
    std::cout << "Maximum Product: " << max;
    if (validate) {
      if (passed)
        std::cout << " (PASSED)";
      else
        std::cout << " (FAILED)";
    }
    std::cout << "\nExecution time:  " << exec_time << std::endl;
  });

  return EXIT_SUCCESS;
}
