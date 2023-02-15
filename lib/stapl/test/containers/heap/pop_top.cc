/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/heap/heap.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/heap_view.hpp>
#include <stapl/views/array_view.hpp>
#include "../../test_report.hpp"

template <typename OutputView, typename HeapView>
bool top_fail(const OutputView& data, HeapView& v)
{
  bool heap_broke = false;
  bool heap_exist = true;
  typename HeapView::value_type max = v.top();
  for (typename OutputView::iterator it = data.begin() ;
       it != data.end() ; ++it)
  {
    heap_exist = true;
    //Check heap property
    if (it+1 != data.end() && *it < *(it+1))
      heap_broke = true;
    if (*it > max)
      heap_broke = true;
    //Check if value is still in v
    for (typename HeapView::iterator iter = v.begin() ;
         iter != v.end() ; ++iter)
      if (*iter == *it)
        heap_exist = false;
    if (heap_broke || heap_exist)
      return true;
  }
  return false;
}

template <typename OutputView, typename HeapView>
bool pop_fail(const OutputView& data, HeapView& v)
{
  bool heap_broke = false;
  bool heap_exist = false;
  typename HeapView::value_type max = v.top();

  for (typename OutputView::iterator it = data.begin() ;
       it != data.end() ; ++it)
  {
    if (it+1 != data.end() && *it < *(it+1))
      heap_broke = true;
    if (*it < max)
      heap_broke = true;
    //Check if value has been removed from v
    for (typename HeapView::iterator iter = v.begin() ;
         iter != v.end() ; ++iter)
      if (*iter == *it)
        heap_exist = true;
    stapl::rmi_fence();
    if (heap_broke || heap_exist)
    {
      return true;
    }
  }
  return false;
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cerr << "2 params required : heap_size, k"
    << std::endl;
    exit(1);
  }

  typedef stapl::heap<int, stapl::less<int> >    heap_type;
  typedef stapl::heap_view<heap_type>            view_type;
  typedef stapl::array<int>                      array_type;
  typedef stapl::array_view<array_type>          arr_view_type;
  typedef stapl::array_view<stapl::array<int> >  output_view_type;

  size_t input_size = atoi(argv[1]);
  size_t ci = 3;
  size_t k = atoi(argv[2]);
  bool passed_1 = true;
  bool passed_2 = true;
  size_t keep_size = 0;

  //Craeate arrays and views
  array_type arr2(input_size);
  array_type arr3(input_size);
  arr_view_type v2(arr2);
  arr_view_type v3(arr3);

  //Fill them
  arr2[0]=13;
  arr2[1]=0;
  arr2[2]=4;
  arr2[3]=11;
  arr2[4]=3;
  arr2[5]=10;
  arr2[6]=2;
  arr2[7]=12;
  arr2[8]=9;
  arr2[9]=6;
  arr2[10]=8;
  arr2[11]=7;
  arr2[12]=5;
  stapl::iota(v3,0);

  //Create 2 heaps
  heap_type heap;
  heap_type heap_bis;
  heap.make(v2);
  heap_bis.make(v3);
  view_type view(heap);
  view_type view_bis(heap_bis);

  //Heap iota
  for (size_t i = 0 ; i < ci; ++i)
  {
    keep_size = view.size();
    output_view_type keep_top = view.top_k(k);
    if (view.size() != keep_size
        || top_fail(keep_top,view))
      passed_1 = false;

    output_view_type keep_pop = view.pop_k(k);
    if (view.size() != keep_size - k
        || pop_fail(keep_pop,view))
      passed_1 = false;
  }

  STAPL_TEST_REPORT(passed_2,"Testing heap iota filled");

  //Heap filled
  for (size_t i = 0 ; i < ci; ++i)
  {
    keep_size = view_bis.size();
    output_view_type keep_top = view_bis.top_k(k);

    if (view_bis.size() != keep_size
        || top_fail(keep_top, view_bis))
      passed_2 = false;

    output_view_type keep_pop = view_bis.pop_k(k);

    if (view_bis.size() != keep_size - k
        || pop_fail(keep_pop,view_bis))
      passed_2 = false;
  }

  STAPL_TEST_REPORT(passed_1,"Testing heap top distributed");

  return EXIT_SUCCESS;
}
