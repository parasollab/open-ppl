/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/list/list.hpp>
#include <stapl/views/list_view.hpp>
#include <stapl/algorithms/numeric.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/partitions/balanced.hpp>

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
  size_t num_locs = get_num_locations();

  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  size_t n = atol(argv[1]);
  typedef list<size_t>       list_t;
  typedef list_view<list_t>  view_t;
  list_t c(n);

  STAPL_TEST_MESSAGE("-- Testing stapl::list --")

  list_t c2(n, 2);
  size_t sum = stapl::accumulate(list_view<list_t>(c2), 0);
  STAPL_TEST_REPORT(sum == 2*n,"Testing construction with initial value");

  size_t i = 1;
  for (list_t::iterator it=c.begin();it!=c.end();++it) {
    list_t::gid_type gid = gid_of(it);
    c.set_element(gid, i++);
  }
  size_t sum2 = stapl::accumulate(list_view<list_t>(c), 0);
  STAPL_TEST_REPORT(sum2 == n*(n+1)/2,"Testing set_element");


  bool passed = true;
  i = 1;
  for (list_t::iterator it=c.begin();it!=c.end();++it) {
    list_t::gid_type gid = gid_of(it);
    if (c.get_element(gid) != i)
      passed = false;
    ++i;
  }
  STAPL_TEST_REPORT(passed,"Testing get_element");


  passed = true;
  list_t o(c);
  i = 1;
  for (list_t::iterator it=o.begin();it!=o.end();++it) {
    list_t::gid_type gid = gid_of(it);
    if (o.get_element(gid) != i)
      passed = false;
    ++i;
  }
  STAPL_TEST_REPORT(passed,"Testing copy constructor");


  passed = true;
  size_t count = 1;
  for (list<size_t>::iterator it = c.begin(); it != c.end(); ++it)
    if (*it != count++)
      passed = false;
  STAPL_TEST_REPORT(passed,"Testing global iteration");

  passed = true;
  count = 1;
  for (list<size_t>::const_iterator cit = c.cbegin(); cit != c.cend(); ++cit)
    if (*cit != count++)
      passed = false;
  STAPL_TEST_REPORT(passed,"Testing global const_iteration");

  c.front() = n;
  passed = (c.front()==n);
  STAPL_TEST_REPORT(passed,"Testing front");

  c.back() = -n;
  passed = (c.back()==-n);
  STAPL_TEST_REPORT(passed,"Testing back");

  // passed = true;

  // c.distribution().apply_set(0, assign_val<int>(n*2));
  // passed = *(c.begin()) == n*2;

  // STAPL_TEST_REPORT(passed,"Testing apply_set");


  // std::pair<int, size_t> last_value =
  //   c.distribution().apply_get(n-1, locality_wf());

  // passed = last_value.first == int(n-1) &&
  //          last_value.second == get_num_locations()-1;

  // STAPL_TEST_REPORT(passed,"Testing apply_get");


  if (get_location_id()==0)
    c.erase(++(c.begin()));
  rmi_fence();
  passed = c.front() == n;
  if (get_location_id()==num_locs-1)
    c.erase(c.begin());
  rmi_fence();

  passed &= c.front() == size_t(3);
  STAPL_TEST_REPORT(passed,"Testing erase");

  if (get_location_id()==0)
    c.push_front(10000);
  stapl::rmi_fence();
  passed = *(c.begin()) == size_t(10000);
  STAPL_TEST_REPORT(passed,"Testing push_front");

  if (get_location_id()==0)
    c.pop_front();
  stapl::rmi_fence();
  passed = *(c.begin()) != size_t(10000);
  STAPL_TEST_REPORT(passed,"Testing pop_front");

  if (get_location_id()==0)
    c.push_back(10000);
  stapl::rmi_fence();
  passed = *(c.end()-1) == size_t(10000);
  STAPL_TEST_REPORT(passed,"Testing push_back");

  if (get_location_id()==0)
    c.pop_back();
  stapl::rmi_fence();
  passed = *(c.end()-1) != size_t(10000);
  STAPL_TEST_REPORT(passed,"Testing pop_back");

  list<size_t> c3;
  size_t loc = get_location_id();
  for (size_t i=0;i<=loc;++i)
    c3.add(loc+1);
  // c3.flush();
  passed = c3.size() == (num_locs*(num_locs+1)/2);
  STAPL_TEST_REPORT(passed,"Testing add");


  list_t d(n*2);
  view_t vd(d);

  stapl::generate(vd, sequence<size_t>(n*2+5, -1));

  c = d;

  view_t vc(c);

  passed = stapl::equal(vc, vd);

  if (passed)
    passed = (c.size() == d.size() && vc.size() == vd.size());

  STAPL_TEST_REPORT(passed, "Testing assignment operator")


  c3.clear();
  passed = c3.size() == 0;
  STAPL_TEST_REPORT(passed,"Testing clear")


  typedef size_t                                       value_t;
  typedef indexed_domain<size_t>                       domain_type;
  typedef stapl::balanced_partition<domain_type>       balanced_type;
  domain_type list_dom(0, n -1);

  balanced_type partition1(list_dom, get_num_locations());
  stapl::mapper<size_t>  map(partition1.domain());
  list<value_t, balanced_type> e(partition1, map);

  passed = e.size() == n;
  STAPL_TEST_REPORT(passed, "Testing List Mapper & Partition Constructor")

  n = 100;
  domain_type list_dom2(0, n -1);
  balanced_type partition2(list_dom2, get_num_locations());
  list<value_t, balanced_type> f(partition2);

  passed = f.size() == n;
  STAPL_TEST_REPORT(passed, "Testing List Partition Constructor")

  n = 10;
  domain_type list_dom3(0, get_num_locations()-1);
  stapl::mapper<size_t> map2(list_dom3);
  list<value_t> g(n, map2);

  passed = g.size() == n;
  STAPL_TEST_REPORT(passed, "Testing List Size_t & Mapper Constructor")

  return EXIT_SUCCESS;
}
