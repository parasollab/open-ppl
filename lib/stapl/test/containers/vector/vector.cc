/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/containers/partitions/blocked_partition.hpp>
#include <stapl/algorithms/numeric.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../../test_report.hpp"

using namespace stapl;

template<typename T>
struct assign_val
{
  typedef T      first_argument_type;
  typedef void   result_type;

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


struct test_wf
{
  typedef std::pair<int, size_t> result_type;

  template<typename T>
  result_type operator()(T t) const
  {
    return std::make_pair(t, t*2);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t num_locs = get_num_locations();

  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  typedef vector_view<vector<size_t> > view_type;

  size_t n = atol(argv[1]);
  vector<size_t> c(n);

  STAPL_TEST_MESSAGE("-- Testing stapl::vector --")

  for (size_t i = 0; i < n; ++i)
    c.set_element(i, i);

  STAPL_TEST_REPORT(true, "Testing size");

  bool passed = true;

  for (size_t i = 0; i < n; ++i)
    if (c.get_element(i) != i)
      passed = false;

  STAPL_TEST_REPORT(passed, "Testing get_element")

  passed = true;

  for (size_t i = 0; i < n; ++i)
    if (c[i] != i)
      passed = false;

  STAPL_TEST_REPORT(passed, "Testing []")


  passed = true;

  vector<size_t> o(c);
  for (size_t i = 0; i < n; ++i)
    if (o.get_element(i) != i)
      passed = false;

  STAPL_TEST_REPORT(passed, "Testing copy constructor")


  passed = true;

  size_t count = 0;
  for (vector<size_t>::iterator it = c.begin(); it != c.end(); ++it)
    if (*it != count++)
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing global iteration")


  passed = true;

  count = 0;
  for (vector<size_t>::const_iterator cit = c.cbegin(); cit != c.cend(); ++cit)
    if (*cit != count++)
      passed = false;

  STAPL_TEST_REPORT(passed, "Testing global const_iteration")


  std::pair<size_t, size_t> last_value =
    c.distribution().apply_get(n-1, test_wf());
  passed = last_value.first == n-1 && last_value.second == (n-1)*2;
  STAPL_TEST_REPORT(passed,"Testing apply_get")


  c.distribution().apply_set(0, assign_val<int>(n*2));
  passed = c[0] == n*2;
  STAPL_TEST_REPORT(passed,"Testing apply_set")

  c.front() = n;
  passed = (c.front()==n);
  STAPL_TEST_REPORT(passed,"Testing front")

  c.back() = -n;
  passed = (c.back()==-n);
  STAPL_TEST_REPORT(passed,"Testing back")

  if (get_location_id()==0)
    c.push_back(10000);
  stapl::rmi_fence();
  passed = c[c.size()-1] == size_t(10000);
  STAPL_TEST_REPORT(passed,"Testing push_back")

  if (get_location_id()==0)
    c.pop_back();
  stapl::rmi_fence();
  passed = c[c.size()-1] != size_t(10000);
  STAPL_TEST_REPORT(passed,"Testing pop_back")

  c.push_back(get_location_id());
  stapl::rmi_fence();
  passed = c.size() == n + get_num_locations() &&
    c[c.size()-1-get_location_id()] >= 0 &&
    c[c.size()-1-get_location_id()] < get_num_locations();
  STAPL_TEST_REPORT(passed, "Testing concurrent push_back")

  c.pop_back();
  stapl::rmi_fence();
  passed = c.size() == n;
  STAPL_TEST_REPORT(passed, "Testing concurrent pop_back")

  view_type cvw(c);
  cvw.push_back(get_location_id());
  stapl::rmi_fence();
  passed = cvw.size() == n + get_num_locations() &&
    cvw[cvw.size()-1-get_location_id()] >= 0 &&
    cvw[cvw.size()-1-get_location_id()] < get_num_locations();
  STAPL_TEST_REPORT(passed, "Testing concurrent push_back using a vector_view")

  cvw.pop_back();
  stapl::rmi_fence();
  passed = cvw.size() == n;
  STAPL_TEST_REPORT(passed, "Testing concurrent pop_back using a vector_view")

  size_t loc_n = n/get_num_locations();
  size_t rem = n % get_num_locations();
  vector<size_t> v1(n);
  for (size_t i = 0; i < loc_n; ++i)
    v1.pop_back();
  stapl::rmi_fence();
  passed = v1.size() == rem;
  STAPL_TEST_REPORT(passed, "Testing concurrent pop_back over the whole vector")

  vector<size_t> v2(n);
  view_type vv2(v2);
  for (size_t i = 0; i < loc_n; ++i)
    vv2.pop_back();
  stapl::rmi_fence();
  passed = vv2.size() == rem;
  STAPL_TEST_REPORT(passed,
    "Testing concurrent pop_back over the whole vector using a vector_view")

  vector<size_t> v3;
  view_type vv3(v3);
  for (size_t i = 0; i < loc_n; ++i)
    vv3.push_back(i);
  stapl::rmi_fence();
  passed = vv3.size() == loc_n * get_num_locations();
  STAPL_TEST_REPORT(passed,
    "Testing concurrent push_back into an empty vector using a vector_view");

  for (size_t i = 0; i < loc_n/2; ++i)
    vv3.pop_back();
  stapl::rmi_fence();
  passed = vv3.size() == (loc_n/2 + loc_n%2) * get_num_locations();
  STAPL_TEST_REPORT(passed,
    "Testing concurrent pop_back from half of a vector using a vector_view");

  for (size_t i = 0; i < loc_n/2 + loc_n%2; ++i)
    vv3.pop_back();
  stapl::rmi_fence();
  passed = vv3.size() == 0;
  STAPL_TEST_REPORT(passed, "Testing concurrent pop_back completely"
                            " clearing a vector using a vector_view");

  stapl::rmi_fence();
  size_t k = (n/num_locs)-1;
  size_t before_size = c.size();
  c.insert(get_location_id()*k+1,5000*(get_location_id()+1));
  c.insert(get_location_id()*k+1,5000*(get_location_id()+1)+1);
  c.insert(get_location_id()*k+1,5000*(get_location_id()+1)+2);
  c.insert(get_location_id()*k+1,5000*(get_location_id()+1)+3);
  c.insert(get_location_id()*k+1,5000*(get_location_id()+1)+4);
  stapl::rmi_fence();
  size_t after_size = c.size();
  passed = (c[0] == n) && (c[1]!=size_t(1)) &&
           (after_size == before_size + 5*num_locs);
  STAPL_TEST_REPORT(passed,"Testing insert")

  stapl::rmi_fence();
  before_size = c.size();
  c.erase(get_location_id()*k+1);
  c.erase(get_location_id()*k+1);
  c.erase(get_location_id()*k+1);
  c.erase(get_location_id()*k+1);
  c.erase(get_location_id()*k+1);
  stapl::rmi_fence();
  after_size = c.size();
  passed = (c[0] == n) && (c[after_size-1] == -n)
           && (after_size == before_size -  5 * num_locs);
  STAPL_TEST_REPORT(passed,"Testing erase")


  vector<size_t> d(n*2);
  view_type vd(d);

  stapl::generate(vd, sequence<size_t>(n*2+5, -1));

  c = d;

  view_type vc(c);

  passed = stapl::equal(vc, vd);

  if (passed)
    passed = (c.size() == d.size() && vc.size() == vd.size());

  STAPL_TEST_REPORT(passed, "Testing assignment operator")

  vector<int> c2;

  c2.clear();
  passed = c2.size() == 0;
  STAPL_TEST_REPORT(passed,"Testing clear on empty container");

  size_t loc = get_location_id();
  for (size_t i=0;i<=loc;++i)
    c2.add(loc+1);
  c2.flush();
  passed = c2.size() == (num_locs*(num_locs+1)/2);
  STAPL_TEST_REPORT(passed,"Testing add")

  c2.clear();
  passed = c2.size() == 0;
  STAPL_TEST_REPORT(passed,"Testing clear")

  vector<size_t> c3(n, 2);
  passed = size_t(2*n) ==
           accumulate(vector_view<vector<size_t> >(c3), size_t(0));
  STAPL_TEST_REPORT(passed,"Testing construct with initial value")

  c3.resize(n/2);
  passed = size_t(n) ==
           accumulate(vector_view<vector<size_t> >(c3), size_t(0));
  passed = passed && c3.size() == n/2;
  STAPL_TEST_REPORT(passed,"Testing resize decrease")

  c3.resize(2*n);
  passed = size_t(n) ==
           accumulate(
             vector_view<vector<size_t> >(c3, indexed_domain<size_t>(n/2)),
             size_t(0));
  passed = passed && c3.size() == 2*n;
  STAPL_TEST_REPORT(passed,"Testing resize increase")

  return EXIT_SUCCESS;
}
