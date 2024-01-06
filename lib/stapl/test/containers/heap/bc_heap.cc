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
#include <stapl/containers/heap/base_container.hpp>
#include <stapl/containers/distribution/container_manager/cm_heap.hpp>
#include <vector>

#include "../../test_report.hpp"

using namespace stapl;

typedef  heap_base_container<int,std::less<int>,
         heap_base_container_traits<int,std::less<int> > >   bc_type;

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

  //Testing constructors
  std::vector<int> v;
  std::less<int> less;
  std::vector<int> tmp1(n);
  std::vector<int> tmp2(n);
  bc_type bc1(less);
  bc_type bc2(n,less);
  bc_type::iterator i1;
  bc_type::iterator i2;
  bc_type::iterator i3;
  bool passed = true;
  int cpt = 0;

  //Fill base containers and vector
  for (size_t i = 0 ; i < n ; ++i) {
    int my_r = rand() % 50;
    bc1.push(my_r);
    bc2.push(my_r);
    v.push_back(my_r);
  }

  //Copy and sort base containers
  //Sort is needed, because heap order is not unique
  tmp1.resize(bc1.size());
  tmp2.resize(bc2.size());
  std::copy(bc1.begin(), bc1.end(), tmp1.begin());
  std::sort(tmp1.begin(), tmp1.end());
  std::copy(bc2.begin(), bc2.end(), tmp2.begin());
  std::sort(tmp2.begin(), tmp2.end());
  std::sort(v.begin(), v.end());

  //if different after sorting, constructors failed
  if (tmp1 != tmp2 || tmp2 != v) {
    std::cout<<"DATA DIFFERENT"<<std::endl;
    passed = false;
  }

  //Check if they all are heap
  cpt=0;
  for (i1 = bc1.begin(); i1 != bc1.end() ; ++i1) {
    i2 = i1+1+cpt;
    i3 = i2+1+cpt;
    if (i2 != bc1.end()) {
      ++cpt;
      if (less(*i1,*i2)) {
        passed = false;
        std::cout<<*i2<<">"<<*i1<<std::endl;
      }
      if (i3 <= bc1.end()) {
        if (less(*i1,*i3)) {
          passed = false;
          std::cout<<*i3<<">"<<*i1<<std::endl;
        } else {
          break;
        }
      }
    } else {
      break;
    }
  }

  //At this point same data + correct heap ordering => true, false otherwise
  STAPL_TEST_REPORT(passed,"Testing constructors and push()");
  passed = true;

  bc2.clear();
  if (!bc2.empty())
    passed = false;

  STAPL_TEST_REPORT(passed,"Testing clear() and empty()");
  passed = true;

  i1 = bc1.begin();
  for (size_t i = 0; i < n; ++i) {
    bc1.set_element(bc1.gid_of(i1),int(i));
    i1++;
  }

  i1 = bc1.begin();
  for (size_t i = 0; i < n; ++i) {
    if (bc1.get_element(bc1.gid_of(i1)) != int(i))
      passed = false;
    i1++;
  }

  STAPL_TEST_REPORT(passed,"Testing set_element() and get_element()");
  passed = true;

  bc_type o(bc1);
  passed = std::equal(bc1.begin(),bc1.end(),o.begin());

  STAPL_TEST_REPORT(passed,"Testing copy constructor");
  passed = true;

  int count = 0;
  for (bc_type::iterator it = o.begin(); it != o.end(); ++it)
    if (*it != count++)
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing global iteration");
  passed = true;

  i1 = bc1.begin();
  bc_type::iterator iter = bc1.make_iterator(bc1.gid_of(i1));
  if (*iter != *(bc1.begin())) {
    std::cout <<"iter : "<<*iter<<" bc1 begin : "<<*(bc1.begin());
    passed = false;
  }

  bc_type::iterator next_it = iter;
  next_it++;
  if (*next_it < *iter) {
    std::cout <<"iter : "<<*iter<<" next_it : "<<*(next_it);
    passed = false;
  }

  STAPL_TEST_REPORT(passed,"Testing make_iterator(gid)");
  passed = true;

  i1 = bc1.begin();
  size_t rand_mess = rand() % n ;
  for (size_t i = 0 ; i < rand_mess ; ++i)
    i1++;

  (*i1) = 999;
  if (i1 != bc1.begin())
    if (bc1.is_heap())
      passed = false;

  STAPL_TEST_REPORT(passed,"Testing is_heap()");

  return EXIT_SUCCESS;
}
