/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <stapl/skeletons/serial.hpp>
#include <stapl/array.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/numeric.hpp>

namespace  {

struct init_sets
{
private:
  unsigned int m_nlocs;

public:
  typedef void result_type;

  init_sets(unsigned int nlocs)
    : m_nlocs(nlocs)
  { }

  template <typename Index, typename Element, typename Container>
  result_type operator()(Index i, Element e, Container& c)
  {
    if (i != m_nlocs-1)
      c.set_element(i+1, e+1);
  }

  void define_type(stapl::typer& t)
  { t.member(m_nlocs); }
};

} // namespace

stapl::exit_code stapl_main(int, char**)
{
  unsigned int nlocs = stapl::get_num_locations();

  stapl::array<int> values(nlocs, 0);
  stapl::array_view<stapl::array<int>> vv(values);

  stapl::serial(init_sets(nlocs), stapl::counting_view<int>(nlocs), vv,
                stapl::make_repeat_view(vv));

  int serial_sum = stapl::accumulate(vv, 0);

  // reset container to zeros.
  stapl::generate(vv, stapl::sequence<int>(0,0));


  // test serial_sets, nsets = 1
  int num_sets = 1;


  stapl::serial_sets(init_sets(nlocs), num_sets,
                     stapl::counting_view<int>(nlocs), vv,
                     stapl::make_repeat_view(vv));

  int serial_sets_sum_1 = stapl::accumulate(vv, 0);

  // reset container to zeros.
  stapl::generate(vv, stapl::sequence<int>(0,0));


  // test serial_sets, nsets = 2
  num_sets = 2;


  stapl::serial_sets(init_sets(nlocs), num_sets,
                     stapl::counting_view<int>(nlocs), vv,
                     stapl::make_repeat_view(vv));

  int serial_sets_sum_2 = stapl::accumulate(vv, 0);

  // reset container to zeros.
  stapl::generate(vv, stapl::sequence<int>(0,0));


  // test serial_sets, nsets = 3
  num_sets = 3;


  stapl::serial_sets(init_sets(nlocs), num_sets,
                     stapl::counting_view<int>(nlocs), vv,
                     stapl::make_repeat_view(vv));

  int serial_sets_sum_3 = stapl::accumulate(vv, 0);

  // reset container to zeros.
  stapl::generate(vv, stapl::sequence<int>(0,0));


  // test serial_sets, nsets = 4
  num_sets = 4;


  stapl::serial_sets(init_sets(nlocs), num_sets,
                     stapl::counting_view<int>(nlocs), vv,
                     stapl::make_repeat_view(vv));

  int serial_sets_sum_4 = stapl::accumulate(vv, 0);

  // The sums of the sets lie between the value produced when all sets begin
  // executing concurrently and the value produced by a serial scheduling of
  // the tasks.
  stapl::do_once([&](void) {
    if ((unsigned int)serial_sum == (nlocs-1)*nlocs/2)
      std::cerr << "serial test passed\n";
    else
      std::cerr << "serial test FAILED with sum " << serial_sum
                << " expected " << (nlocs-1)*nlocs/2 << "\n";

    // Results for 1 set should be same as serial.
    if ((unsigned int)serial_sets_sum_1 == (nlocs-1)*nlocs/2)
      std::cerr << "serial_set nsets=1 test passed\n";
    else
      std::cerr << "serial_set nsets=1 test FAILED with sum "
                << serial_sets_sum_1 << " expected "
                << (nlocs-1)*nlocs/2 << "\n";

    // For two sets, first sequence is [0, nlocs/2], second is [1, nlocs/2-1]
    if ((unsigned int)serial_sets_sum_2 >=
        (nlocs/2)*(nlocs/2+1)/2 + (nlocs/2-1)*(nlocs/2)/2 &&
        (unsigned int)serial_sets_sum_2 <= (nlocs-1)*nlocs/2)
      std::cerr << "serial_set nsets=2 test passed\n";
    else
      std::cerr << "serial_set nsets=2 test FAILED with sum "
                << serial_sets_sum_2 << " expected between "
                << (nlocs/2)*(nlocs/2+1)/2 + (nlocs/2-1)*(nlocs/2)/2 << " and "
                << (nlocs-1)*nlocs/2 << "\n";

    if (nlocs < 3)
    {
      // all elements in first block
      if ((unsigned int)serial_sets_sum_3 == (nlocs-1)*nlocs/2)
        std::cerr << "serial_set nsets=3 test passed\n";
      else
        std::cerr << "serial_set nsets=3 test FAILED with sum "
                  << serial_sets_sum_3 << " expected "
                  << (nlocs-1)*nlocs/2 << "\n";
    }
    else if (nlocs == 3)
    {
      // For three sets, each element is 1 except the first.
      if ((unsigned int)serial_sets_sum_3 >= nlocs-1 &&
          (unsigned int)serial_sets_sum_3 <= (nlocs-1)*nlocs/2)
        std::cerr << "serial_set nsets=3 test passed\n";
      else
        std::cerr << "serial_set nsets=3 test FAILED with sum "
                  << serial_sets_sum_3 << " expected between " << nlocs-1
                  << " and " << (nlocs-1)*nlocs/2 << "\n";
    }
    else
    {
      // For three sets, first sequence is [0,nlocs/3], others are [1,nlocs/3-1]
      if ((unsigned int)serial_sets_sum_3 >=
          (nlocs/3+1)*(nlocs/3+2)/2 + (nlocs/3)*(nlocs/3+1)/2 &&
          (unsigned int)serial_sets_sum_3 <= (nlocs-1)*nlocs/2)
        std::cerr << "serial_set nsets=3 test passed\n";
      else
        std::cerr << "serial_set nsets=3 test FAILED with sum "
                  << serial_sets_sum_3 << " expected between "
                  << (nlocs/3+1)*(nlocs/3+2)/2 + (nlocs/3)*(nlocs/3+1)/2
                  << " and " << (nlocs-1)*nlocs/2 << "\n";
    }

    // For four sets, each element is 1 except the first.
    if ((unsigned int)serial_sets_sum_4 >= nlocs-1 &&
        (unsigned int)serial_sets_sum_4 <= (nlocs-1)*nlocs/2)
      std::cerr << "serial_set nsets=4 test passed\n";
    else
      std::cerr << "serial_set nsets=4 test FAILED with sum "
                << serial_sets_sum_4 << " expected " << nlocs-1 << "\n";
  });

  return EXIT_SUCCESS;
}
