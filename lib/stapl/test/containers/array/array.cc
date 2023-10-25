/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/array/array.hpp>
#include <stapl/containers/partitions/blocked_partition.hpp>
#include <stapl/containers/mapping/mapper.hpp>

#include <stapl/views/array_view.hpp>

#include <stapl/algorithms/algorithm.hpp>

#include <boost/mpl/bool.hpp>

#include "../../test_report.hpp"

using namespace stapl;

template<typename T>
struct assign_val
{
  typedef T                  first_argument_type;
  typedef void               result_type;

private:
  first_argument_type m_val;

public:
  assign_val(const T& val) : m_val(val) { }

  template<typename Reference>
  void operator()(Reference& y) const
  {
    y = m_val;
  }

  void define_type(typer& t)
  {
    t.member(m_val);
  }
};

struct locality_wf
{
  typedef std::pair<int, size_t> result_type;

  template<typename T>
  result_type operator()(T t) const
  {
    return std::make_pair(t, get_location_id());
  }

};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  typedef size_t                       value_t;
  typedef array_view<array<value_t> >  view_type;

  size_t n = atol(argv[1]);

  if (n < 2) {
    stapl::do_once(
      [&]{std::cerr << "The size should be greater than 1." << std::endl;}
    );
    exit(1);
  }

  array<value_t> c(n);

  for (size_t i = 0; i < n; ++i)
    c.set_element(i, i);

  STAPL_TEST_REPORT(true, "Testing set_element ")

  bool passed = true;

  for (size_t i = 0; i < n; ++i)
    if (c.get_element(i) != i)
      passed = false;

  STAPL_TEST_REPORT(passed, "Testing get_element")

  passed = true;

  for (size_t i = 0; i < n; ++i)
    if (c[i] != i)
      passed = false;

  STAPL_TEST_REPORT(passed, "Testing [] ")

  passed = true;

  if (c.front() != 0)
    passed = false;

  STAPL_TEST_REPORT(passed, "Testing front")

  passed = true;

  if (c.back() != (n - 1))
    passed = false;

  STAPL_TEST_REPORT(passed, "Testing back")

  passed = true;

  array<value_t> o(c);
  for (size_t i = 0; i < n; ++i)
    if (o.get_element(i) != i)
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing copy constructor")

  passed = true;

  size_t count = 0;
  for (array<value_t>::iterator it = c.begin(); it != c.end(); ++it)
    if (*it != count++)
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing global iteration");

  passed = true;

  count = 0;
  for (array<value_t>::const_iterator it = c.begin(); it != c.end(); ++it)
    if (*it != count++)
      passed = false;

  count = 0;
  for (array<value_t>::const_iterator it = c.cbegin(); it != c.cend(); ++it)
    if (*it != count++)
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing global const iteration");

  passed = true;

  c.distribution().apply_set(0, assign_val<size_t>(n*2));
  passed = (c[0] == n*2);

  STAPL_TEST_REPORT(passed,"Testing apply_set");

  location_type last_loc =
    n < get_num_locations() ? n-1 : get_num_locations() - 1;

  std::pair<value_t, size_t> last_value =
    c.distribution().apply_get((n-1), locality_wf());

  passed = (last_value.first == (n-1)) &&
           (last_value.second == last_loc);

  STAPL_TEST_REPORT(passed,"Testing apply_get");

  passed = true;

  array<value_t> rc(c);

  rc.resize(n*2);
  passed = (rc.size() == n*2);

  array_view<array<value_t> > rcv(rc);
  value_t sum = accumulate(rcv, 0);
  if (sum != (n*2) + (n-1) * n/2) {
    passed = false;
  }

  STAPL_TEST_REPORT(passed,"Testing increasing resize");

  //
  //BEGIN TESTS ON INCREASING RESIZE CONTAINER
  //

  n *= 2; //rc is twice as big as c

  for (size_t i = 0; i < n; ++i)
    rc.set_element(i, i);

  STAPL_TEST_REPORT(true, "Testing increasing resize - set_element")

  passed = true;

  for (size_t i = 0; i < n; ++i)
    if (rc.get_element(i) != i)
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing increasing resize - get_element");

  passed = true;

  for (size_t i = 0; i < n; ++i)
    if (rc[i] != i)
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing increasing resize - []");

  passed = true;

  if (rc.front() != 0)
    passed = false;

  STAPL_TEST_REPORT(passed, "Testing increasing resize - front")

  passed = true;

  if (rc.back() != (n - 1))
    passed = false;

  STAPL_TEST_REPORT(passed, "Testing increasing resize - back")

  passed = true;

  array<value_t> rco(rc);
  for (size_t i = 0; i < n; ++i)
    if (rco.get_element(i) != i)
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing increasing resize - copy constructor");


  passed = true;

  count = 0;
  for (array<value_t>::iterator it = rc.begin(); it != rc.end(); ++it)
    if (*it != count++)
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing increasing resize - global iteration");


  passed = true;

  count = 0;
  for (array<value_t>::const_iterator it = rc.begin(); it != rc.end(); ++it)
    if (*it != count++) {
      passed = false;
    }

  count = 0;
  for (array<value_t>::const_iterator it = rc.cbegin(); it != rc.cend(); ++it)
    if (*it != count++) {
      passed = false;
    }

  STAPL_TEST_REPORT(passed,
    "Testing increasing resize - global const iteration");


  passed = true;

  rc.distribution().apply_set(0, assign_val<size_t>(n*2));
  passed = (rc[0] == n*2);

  STAPL_TEST_REPORT(passed,"Testing increasing resize - apply_set");

  last_loc = n < get_num_locations() ? n-1 : get_num_locations() - 1;

  last_value =
    rc.distribution().apply_get((n-1), locality_wf());

  passed = (last_value.first == (n-1)) &&
           (last_value.second == last_loc);

  STAPL_TEST_REPORT(passed,"Testing increasing resize - apply_get");


  n /= 2; //set back to non-resized n value

  //
  //END TESTS ON INCREASING RESIZE CONTAINER
  //

  array<value_t> sc(c);

  sc.resize(n/2);
  passed = (sc.size() == n/2);

  array_view<array<value_t> > scv(sc);
  sum = accumulate(scv, 0);
  if (sum != (n*2) + ((n/2)-1) * (n/2)/2) {
    passed = false;
  }

  STAPL_TEST_REPORT(passed,"Testing decreasing resize");

  //
  //BEGIN TESTS ON DECREASING RESIZE CONTAINER
  //

  size_t ntemp = n;
  n /= 2; //sc is half c

  for (size_t i = 0; i < n; ++i)
    sc.set_element(i, i);

  STAPL_TEST_REPORT(true, "Testing decreasing resize - set_element")

  passed = true;

  for (size_t i = 0; i < n; ++i)
    if (sc.get_element(i) != i)
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing decreasing resize - get_element");

  passed = true;

  for (size_t i = 0; i < n; ++i)
    if (sc[i] != i)
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing decreasing resize - []")

  passed = true;

  if (sc.front() != 0)
    passed = false;

  STAPL_TEST_REPORT(passed, "Testing decreasing resize - front")

  passed = true;

  if (sc.back() != (n - 1))
    passed = false;

  STAPL_TEST_REPORT(passed, "Testing decreasing resize - back")

  passed = true;

  array<value_t> sco(sc);
  for (size_t i = 0; i < n; ++i)
    if (sco.get_element(i) != i)
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing decreasing resize - copy constructor");


  passed = true;

  count = 0;
  for (array<value_t>::iterator it = sc.begin(); it != sc.end(); ++it)
    if (*it != count++)
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing decreasing resize - global iteration");


  passed = true;

  count = 0;
  for (array<value_t>::const_iterator it = sc.begin(); it != sc.end(); ++it)
    if (*it != count++) {
      passed = false;
    }

  count = 0;
  for (array<value_t>::const_iterator it = sc.cbegin(); it != sc.cend(); ++it)
    if (*it != count++) {
      passed = false;
    }

  STAPL_TEST_REPORT(passed,
    "Testing decreasing resize - global const iteration");


  passed = true;

  sc.distribution().apply_set(0, assign_val<size_t>(n*2));
  passed = (sc[0] == n*2);
  sc.distribution().apply_set(0, assign_val<size_t>(0));

  STAPL_TEST_REPORT(passed,"Testing decreasing resize - apply_set");

  last_loc = n < get_num_locations() ? n-1 : get_num_locations() - 1;

  last_value =
    sc.distribution().apply_get((n-1), locality_wf());

  passed = (last_value.first == (n-1)) &&
           (last_value.second == last_loc);

  STAPL_TEST_REPORT(passed,"Testing decreasing resize - apply_get");


  n = ntemp; //set back to non-resized n value

  //
  //END TESTS ON DECREASING RESIZE CONTAINER
  //


  passed = true;

  array<value_t> d(n*2);
  view_type vd(d);

  stapl::generate(vd, sequence<value_t>(n*2+5, -1));

  c = d;

  view_type vc(c);

  passed = stapl::equal(vc, vd);

  if (passed)
    passed = (c.size() == d.size() && vc.size() == vd.size());

  STAPL_TEST_REPORT(passed, "Testing assignment operator");


  typedef indexed_domain<size_t>                domain_type;
  typedef stapl::block_partitioner<domain_type> blocked_type;
  domain_type array_dom(0, n-1);

  array<value_t, blocked_type> blocked_array(blocked_type(array_dom, 5));

  for (size_t i = 0; i < n; ++i)
    blocked_array.set_element(i, i);

  passed = true;

  for (size_t i = 0; i < n; ++i)
    if (blocked_array.get_element(i) != i)
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing block_partition");

  blocked_type partition1(array_dom, get_num_locations());
  stapl::mapper<size_t>  map(partition1.domain());
  array<value_t, blocked_type> e(partition1, map);

  passed = true;
  passed = e.size() == n;
  STAPL_TEST_REPORT(passed, "Testing Mapper/Partition Constructor");

  array<value_t> empty_array;

  passed = empty_array.size() == 0;

  STAPL_TEST_REPORT(passed, "Testing Default Constructor");

  return EXIT_SUCCESS;
}
