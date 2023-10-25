/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/array/array.hpp>
#include <stapl/containers/heap/heap.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/heap_view.hpp>
#include "../../test_report.hpp"

using namespace stapl;
using namespace std;

template <typename C, typename It>
void print(C c) {
  cout << get_location_id() <<"] " ;
  for (It it = c.begin(); it != c.end(); ++it)
    cout << *it << ", " ;
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef heap_base_container<size_t,std::less<size_t>,
                              heap_base_container_traits
                              <size_t,std::less<size_t> > >      bc_type;
  typedef bc_type::value_type                      value_type;

  typedef balanced_partition< indexed_domain<size_t> >      partitioner_type;
  typedef mapper<size_t>                                    mapper_type;
  typedef heap_traits<size_t, std::less<size_t>,
          partitioner_type, mapper_type>                    heap_traits_type;
  typedef heap<size_t, std::less<size_t>, partitioner_type,
               mapper_type, heap_traits_type>                    heap_type;
  typedef heap_view<heap_type>                              heap_view_type;
  typedef heap_view<heap_type>::iterator           heap_view_iterator;

  if (argc < 2) {
    std::cerr << "usage: exe n (n > 2)" << std::endl;
    exit(1);
  }
  size_t n = atol(argv[1]);
  bool passed = true;

  stapl::array<size_t> parray(n);
  stapl::array_view<stapl::array<size_t> > v(parray);

  for (size_t i=0; i < n ; ++i)
    parray[i] = n-i;

  heap_type heap_empty;
  heap_type heap1(v);
  heap_type heap3(v);
  heap_view_type heap_view_full(heap1);
  heap_view_type heap_view_full_copy(heap1);
  heap_view_type heap_view_full2(heap3);
  heap_view_type heap_view_empty(heap_empty);
  heap_view_type heap_view_make(heap_empty);

  STAPL_TEST_REPORT(passed,"Testing constructors");

  passed = true;
  int count =0;
  for (heap_view_iterator it = heap_view_full.begin();
       it != heap_view_full.end(); ++it) {
    ++count;
  }
  n = n + count*0; //Fool compiler warning ... count useless
  STAPL_TEST_REPORT(passed,"Testing global iteration");

  passed = true;
  if (heap_view_full.size() != n)
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing size");

  passed = true;
  if (heap_view_full.top() !=
      stapl::max_value(v,std::less<size_t>()))
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing top");

  passed = true;
  if (heap_view_full.empty() || !heap_view_empty.empty())
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing empty");

  passed = true;
  heap_view_empty.push((rand() % 10) + 10);
  if (heap_view_empty.size() == 0)
    passed = false;
  heap_view_empty.push(5);
  heap_view_empty.push(50);
  if (heap_view_empty.top() != 50)
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing push");

  passed = true;
  value_type keep = heap_view_full.top();
  if (heap_view_full.pop() != keep)
    passed = false;
  if (heap_view_full.pop() == keep)
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing pop");

  passed = true;
  heap_view_make.make(v);
  if (!heap_view_make.is_heap() ||
      heap_view_make.size() != (v.size() + get_num_locations()*3)
      || heap_view_make.top() !=
         stapl::max_value(v,std::less<size_t>()))
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing make(view)");

  return EXIT_SUCCESS;
}
