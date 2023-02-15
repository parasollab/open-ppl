/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/containers/heap/seq_heap.hpp>
#include <vector>

#include "../../test_report.hpp"

using namespace stapl;

typedef  seq_heap<int,std::less<int> >        heap_type;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  size_t n = atol(argv[1]);
  if (n == 0)
  {
    std::cerr << "The number of elements specified must be greater than 0\n";
    return EXIT_FAILURE;
  }

  //Testing storage
  std::vector<int> v;
  std::less<int> c_less;
  heap_type  heap1(c_less);
  heap_type heap2(n,c_less);

  for (size_t i = 0 ; i < n ; i++) {
    int my_r = rand() % 50;
    heap1.push(my_r);
    heap2.push(my_r);
    v.push_back(my_r);
  }
  heap_type heap3(v,c_less);

  bool passed = true ;
  heap_type::iterator it1;
  heap_type::iterator it2 = heap2.begin();
  heap_type::iterator it3 = heap3.begin();

  //check if they all got the same data
  std::vector<int> tmp1(heap1.size());
  std::vector<int> tmp2(heap2.size());
  std::vector<int> tmp3(heap3.size());
  std::copy(heap1.begin(), heap1.end(), tmp1.begin());
  std::sort(tmp1.begin(), tmp1.end());
  std::copy(heap2.begin(), heap2.end(), tmp2.begin());
  std::sort(tmp2.begin(), tmp2.end());
  std::copy(heap3.begin(), heap3.end(), tmp3.begin());
  std::sort(tmp3.begin(), tmp3.end());
  std::sort(v.begin(), v.end());

  if (tmp1 != tmp2 || tmp2 != tmp3 || tmp3 != v)
    {
      std::cout<<"DATA DIFFERENT"<<std::endl;
      passed = false;
    }

  //Check if they all are heap
  int cpt=0; //jump to children during checking
  for (it1 = heap1.begin(); it1 != heap1.end() ; it1++) {
    it2 = it1+1+cpt;
    it3 = it2+1+cpt;
    if (it2 != heap1.end()) {
      cpt++;
      if (c_less(*it1,*it2)) { //*it2 > *it1
        passed = false;
        std::cout<<*it2<<">"<<*it1<<std::endl;
      }
      if (it3 <= heap1.end()) {
        if (c_less(*it1,*it3)) { //*it3 > *it1
          passed = false;
          std::cout<<*it3<<">"<<*it1<<std::endl;
        } else {
          break;
        }
      }
    } else {
      break;
    }
  }

  STAPL_TEST_REPORT(passed,"Testing constructors");
  passed = true;

  heap_type heap_copy(heap1);
  it1 = heap1.begin();
  for (it2 = heap_copy.begin(); it2 != heap_copy.end() ; it2++) {
    if (*it1 != *it2)
      passed = false;
    it1++;
  }

  STAPL_TEST_REPORT(passed,"Testing copy constructors");
  passed = true;

  heap2.push(100);
  heap2.push(101);
  heap2.push(103);
  heap2.push(102);
  if (heap2.front() != 103)
    passed = false;
  heap2.pop();
  if (heap2.front() != 102)
    passed = false;
  heap2.pop();
  if (heap2.front() != 101)
    passed = false;
  heap2.pop();
  if (heap2.front() != 100)
    passed = false;
  heap2.pop();

  STAPL_TEST_REPORT(passed,"Testing pop()");
  passed = true;

  for (size_t i=0; i < n ; i++) {
    heap1.pop();
  }

  if (!heap1.empty())
    passed = false;

  STAPL_TEST_REPORT(passed,"Testing empty()");
  passed = true;

  it3 = heap3.begin();
  size_t rand_mess = rand() % n ;
  for (size_t i = 0 ; i < rand_mess ; i++)
    it3++;

  (*it3) = 999;
  if (it3 != heap3.begin())
    if (heap3.is_heap())
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing is_heap()");
  passed = true;

  heap3.clear();
  if (heap3.size() !=0)
    passed = false;

  STAPL_TEST_REPORT(passed,"Testing size() and clear()");
  passed = true;

  stapl::array<size_t> parray(n);
  stapl::array_view<stapl::array<size_t> > view(parray);

  for (size_t i=0; i < n ; i++)
    parray[i] = n-i;
  heap1.make(view);
  if (!heap1.is_heap())
    passed = false;

  STAPL_TEST_REPORT(passed,"Testing make_heap(view)");

  return EXIT_SUCCESS;
}
