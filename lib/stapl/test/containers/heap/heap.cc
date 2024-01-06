/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

#include <stapl/containers/array/array.hpp>
#include <stapl/containers/heap/heap.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/array_view.hpp>

#include "../../test_report.hpp"

using namespace stapl;
using namespace std;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef heap_base_container<size_t,std::less<size_t>,
                              heap_base_container_traits
                             <size_t,std::less<size_t> > >   bc_type;
  typedef bc_type::gid_type                         gid_type;
  typedef bc_type::value_type                       value_type;

  typedef balanced_partition< indexed_domain<size_t> >       partitioner_type;
  typedef mapper<size_t>                                     mapper_type;
  typedef heap_traits<size_t, std::less<size_t>,
                      partitioner_type, mapper_type>         heap_traits_type;
  typedef heap<size_t, std::less<size_t>,
               partitioner_type, mapper_type,
               heap_traits_type>                             heap_type;
  typedef heap_type::iterator                       heap_iterator;

  boost::random::mt19937                            gen;
  boost::random::uniform_int_distribution<int>      dist;

  if (argc < 2) {
    std::cerr << "usage: exe n (n > 2)" << std::endl;
    exit(1);
  }
  size_t n = atol(argv[1]);
  bool passed = true;
  int my_f = 0;
  int nb_part = 0;
  if (n > 10) {
    my_f = dist(gen) % ((int) (n/10));
  }
  if (my_f == 0) {
    nb_part = 1;
  } else {
    nb_part = my_f;
  }

  indexed_domain <size_t> dom(0,n-1);
  partitioner_type  Partitionner(dom,nb_part) ;
  mapper_type Mapper(indexed_domain<size_t> (0,nb_part - 1));
  stapl::array<size_t> parray(n);
  stapl::array_view<stapl::array<size_t> > v(parray);

  for (size_t i=0; i < n ; ++i)
    parray[i] = n-i;

  heap_type heap_empty;
  heap_type heap1(v);
  heap_type heap2(Partitionner, Mapper);
  heap_type heap3(v);
  heap_type heap_make;
  heap_type heap4;

  STAPL_TEST_REPORT(passed,"Testing constructors");

  int i = dist(gen) % 10 + 5;
  gid_type gid = heap1.domain().first();
  size_t value_setted = 0;
  for (heap_iterator it=heap1.begin();it!=heap1.end();++it) {
    if (gid.valid()) {
      heap1.set_element(gid, i);
      value_setted = i;
    }
    ++i;
  }
  STAPL_TEST_REPORT(passed,"Testing set_element");

  passed = true;
  for (heap_iterator it=heap1.begin();it!=heap1.end();++it) {
    if (gid.valid())
      if (heap1.get_element(gid) != value_setted)
        passed = false;
  }
  STAPL_TEST_REPORT(passed,"Testing get_element and global iteration");

  //Copy constructor will copy a heap that
  //doesn't follow the heap property anymore
  passed = true;
  heap_type heap_copy(heap1);
  gid_type gid_copy = heap_copy.domain().first();
  if (heap_copy.get_element(gid_copy) != value_setted)
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing copy constructor");

  passed = true;
  int count = 1;
  for (heap_iterator it = heap1.begin(); it != heap1.end(); ++it)
    ++count;
  STAPL_TEST_REPORT(passed,"Testing global iteration");

  passed = true;
  if (heap3.size() != n)
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing size");

  passed = true;
  if (heap3.top() != stapl::max_value(v,std::less<size_t>()))
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing top");

  passed = true;
  if (heap1.empty() || !heap_empty.empty())
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing empty");

  passed = true;
  heap4.push((dist(gen) % 10) + 10);
  if (heap4.size() == 0)
    passed = false;
  heap4.push(5);
  heap4.push(50);
  if (heap4.top() != 50)
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing push");

  passed = true;
  value_type keep = heap3.top();
  if (heap3.pop() != keep)
    passed = false;
  if (heap3.pop() == keep)
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing pop");

  passed = true;
  if (!heap3.is_heap())
    passed = false;
  if (get_location_id() == 0)
    heap3.set_element(gid_of(heap3.begin()),0);

  if (heap3.is_heap())
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing is_heap");

  passed = true;
  heap_make.make(v);
  if (!heap_make.is_heap() || heap_make.size() != v.size()
      || heap_make.top() !=
      stapl::max_value(v,std::less<size_t>()))
    passed = false;
  STAPL_TEST_REPORT(passed,"Testing make(view)");

  passed = true;
  if (get_location_id() == 0)
    if (heap3.make_iterator(gid_of(heap3.begin())) != heap3.begin())
      passed = false;
  STAPL_TEST_REPORT(passed,"Testing make_iterator");

  return EXIT_SUCCESS;
}
