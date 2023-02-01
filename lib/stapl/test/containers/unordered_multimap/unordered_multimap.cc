/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/unordered_multimap/unordered_multimap.hpp>
#include <stapl/views/map_view.hpp>
#include <boost/lexical_cast.hpp>

#include "../../test_report.hpp"

using namespace stapl;

template<typename Map>
void test_map(Map& m, size_t n)
{
  typedef map_view<Map> view_type;
  size_t locs = get_num_locations();
  size_t this_loc = get_location_id();

  // tests the insert function by inserting n/p elements and then
  // comparing the actual size to the expected size
  for (size_t i = this_loc; i < n; i += locs) {
    m.insert(i, i);
    m.insert(i, i*2);
  }
  rmi_fence();
  STAPL_TEST_REPORT(m.size() == 2*n, "Testing insert     ");
  rmi_fence();

  // tests the clear function by calling clear on the container, reinserting
  // the n/p elements twice, and then comparing the actual size to the expected
  // size
  m.clear();
  rmi_fence();

  bool empty1 = m.empty();
  size_t cleared_size = m.size();
  rmi_fence();

  for (size_t i = this_loc; i < n; i += locs) {
    m.insert(i, i);
    m.insert(i, i*2);
  }
  rmi_fence();
  STAPL_TEST_REPORT(cleared_size == 0 && m.size() == 2*n,"Testing clear      ");
  rmi_fence();

  bool empty2 = !m.empty();
  STAPL_TEST_REPORT(empty1 && empty2, "Testing empty      ");
  rmi_fence();

  // tests global iteration by iterating from begin to end and compares the sum
  // of the keys to the expected value.
  size_t key_sum = 0;
  size_t map_sum = 0;
  size_t count = 0;
  for (typename Map::iterator it = m.begin(); it != m.end(); ++it)
  {
    key_sum += (*it).first;
    map_sum += (*it).second;
    count += m.count((*it).first);
    count += m.count((*it).first + n);
  }
  rmi_fence();
  bool passed = (key_sum == ((n-1)*n)) && (map_sum == (3*n*(n-1)/2));
  STAPL_TEST_REPORT(passed, "Testing global iter");
  STAPL_TEST_REPORT(count == 4*n, "Testing count      ");

 // tests the find function by calling find on all n elements and setting
 // the test bool equal to false if end() is returned (an element is not
 // found). Also looks for an element beyond end, which should return end()
  bool passed1 = true;
  bool passed2 = true;
  for (size_t i = 0; i < n; ++i)
    if (m.find(i) == m.end())
      passed1 = false;

  if (m.find(3*n) != m.end())
    passed2 = false;
  STAPL_TEST_REPORT(passed1 && passed2, "Testing find       ");
  rmi_fence();

  // tests the equal_range function by summing the size of the constructed
  // views and comparing the result to the expected size
  size_t total_sz = 0;
  size_t element_count = 0;
  for (size_t i = this_loc; i < n; i += locs) {
    total_sz += m.equal_range(i).size();
    ++element_count;
  }
  STAPL_TEST_REPORT(total_sz == element_count*2, "Testing equal_range");

  // tests the erase function by calling erase on n elements and then, after
  // each location finishes erasing elements, checks if the actual size is 0
  size_t erase_count = 0;
  element_count = 0;
  for (size_t i = this_loc; i < n; i += locs) {
    erase_count += m.erase_sync(i);
    ++element_count;
  }
  rmi_fence();
  STAPL_TEST_REPORT(m.size() == 0 && erase_count == 2*element_count,
                                                  "Testing erase      ");
  rmi_fence();
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
     std::cerr << "usage: exe n " << std::endl;
     exit(1);
  }

  typedef size_t key_type;
  typedef stapl::unordered_multimap<key_type, key_type> map_type;

  typedef pair_hash<key_type, stapl::hash<key_type>> Hasher;
  typedef continuous_domain<multi_key<key_type>> Dom;

  size_t n = boost::lexical_cast<size_t>(argv[1]);

  stapl::unordered_map_partition<Hasher, Dom>
    partd = stapl::unordered_map_partition<Hasher, Dom>();

  stapl::mapper<size_t>                            mapperd(partd.domain());

  map_type s0;
  STAPL_TEST_REPORT(true, "Testing with default constructor----------------");
  test_map(s0, n);
  rmi_fence();

  map_type s1;
  map_type s2(s1);
  STAPL_TEST_REPORT(true, "Testing with copy constructor-------------------");
  test_map(s2, n);
  rmi_fence();

  map_type s3(partd);
  STAPL_TEST_REPORT(true, "Testing with partition constructor--------------");
  test_map(s3, n);
  rmi_fence();

  map_type s4(partd, mapperd);
  STAPL_TEST_REPORT(true, "Testing with partition, mapper constructor------");
  test_map(s4, n);
  rmi_fence();


  return EXIT_SUCCESS;
}
