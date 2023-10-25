/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/unordered_set/unordered_set.hpp>
#include <stapl/views/set_view.hpp>
#include <boost/lexical_cast.hpp>

#include "../../test_report.hpp"

using namespace stapl;

template<typename T>
struct shift_hasher
{
  std::size_t m_shift;

  shift_hasher() = default;
  shift_hasher(std::size_t shift) : m_shift(shift) { }

  std::size_t operator()(T const& x) const
  {
    return x << m_shift;
  }
};

template<typename Set>
void test_set(Set& s, size_t n)
{
  typedef set_view<Set> view_type;
  size_t locs = get_num_locations();
  size_t this_loc = get_location_id();

  // tests the insert function by inserting n/p elements and then
  // comparing the actual size to the expected size
  for (size_t i = this_loc; i < n; i += locs) {
    s.insert(i);
    s.insert(i);
  }
  rmi_fence();
  STAPL_TEST_REPORT(s.size() == n, "Testing insert     ");
  rmi_fence();

  // tests the clear function by calling clear on the container, reinserting
  // the n/p elements twice, and then comparing the actual size to the expected
  // size
  s.clear();
  rmi_fence();

  bool empty1 = s.empty();
  size_t cleared_size = s.size();
  rmi_fence();

  for (size_t i = this_loc; i < n; i += locs)
    s.insert(i);
  rmi_fence();
  STAPL_TEST_REPORT(cleared_size == 0 && s.size() == n,"Testing clear      ");
  rmi_fence();

  bool empty2 = !s.empty();
  STAPL_TEST_REPORT(empty1 && empty2, "Testing empty      ");
  rmi_fence();

  // tests global iteration by iterating from begin to end and compares the sum
  // of the keys to the expected value.
  size_t amount = 0;
  size_t count = 0;
  typename Set::iterator it = s.begin();
  while (it != s.end()) {
    amount += *it;
    count += s.count(*it);
    ++it;
  }
  STAPL_TEST_REPORT(amount == n*(n-1)/2, "Testing global iter");
  STAPL_TEST_REPORT(count == n, "Testing count      ");
  rmi_fence();

 // tests the find function by calling find on all n elements and setting
 // the test bool equal to false if end() is returned (an element is not
 // found). Also looks for an element beyond end, which should return end()
  bool passed1 = true;
  bool passed2 = true;
  for (size_t i = 0; i < n; ++i)
    if (s.find(i) == s.end())
      passed1 = false;

  if (s.find(3*n) != s.end())
    passed2 = false;
  STAPL_TEST_REPORT(passed1 && passed2, "Testing find       ");
  rmi_fence();

  // tests the equal_range function by summing the size of the constructed
  // views and comparing the result to the expected size
  size_t total_sz = 0;
  size_t element_count = 0;
  for (size_t i = this_loc; i < n; i += locs) {
    total_sz += s.equal_range(i).size();
    ++element_count;
  }
  STAPL_TEST_REPORT(total_sz == element_count, "Testing equal_range");

  // tests the erase function by calling erase on n elements and then, after
  // each location finishes erasing elements, checks if the actual size is 0
  size_t erase_count = 0;
  element_count = 0;
  for (size_t i = this_loc; i < n; i += locs) {
    erase_count += s.erase_sync(i);
    ++element_count;
  }
  rmi_fence();
  STAPL_TEST_REPORT(s.size() == 0 && erase_count == element_count,
                                                  "Testing erase      ");
  rmi_fence();
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
     std::cerr << "usage: exe n" << std::endl;
     exit(1);
  }

  size_t n = boost::lexical_cast<size_t>(argv[1]);

  typedef size_t key_type;
  typedef stapl::unordered_set<key_type> set_type;
  using hash_type = stapl::hash<key_type>;
  using partition_type = stapl::unordered_map_partition<
    hash_type, continuous_domain<key_type>
  >;

  partition_type mypart(hash_type{}, get_num_locations());
  stapl::mapper<size_t> mymapper(mypart.domain());

  set_type s0;
  STAPL_TEST_REPORT(true, "Testing with default constructor----------------");
  test_set(s0, n);
  rmi_fence();

  set_type s1;
  set_type s2(s1);
  STAPL_TEST_REPORT(true, "Testing with copy constructor-------------------");
  test_set(s2, n);
  rmi_fence();

  set_type s3(mypart);
  STAPL_TEST_REPORT(true, "Testing with partition constructor--------------");
  test_set(s3, n);
  rmi_fence();

  set_type s4(mypart, mymapper);
  STAPL_TEST_REPORT(true, "Testing with partition, mapper constructor------");
  test_set(s4, n);
  rmi_fence();

  using custom_hash_set_type = stapl::unordered_set<
    key_type,
    shift_hasher<key_type>,
    stapl::equal_to<key_type>,
    unordered_map_partition<shift_hasher<key_type>, continuous_domain<key_type>>
   >;

  const std::size_t shift = 5;
  custom_hash_set_type s5(shift_hasher<key_type>{shift});
  STAPL_TEST_REPORT(true, "Testing with custom hash");
  test_set(s5, n);
  rmi_fence();
  STAPL_TEST_REPORT(
    s5.distribution().container_manager().begin()->hash_function().m_shift ==
      shift,
    "Testing custom hash instance is propagated to base container");

  STAPL_TEST_REPORT(
    s5.distribution().partition().hash_function().m_shift == shift,
    "Testing custom hash instance is propagated to partition");

  return EXIT_SUCCESS;
}
