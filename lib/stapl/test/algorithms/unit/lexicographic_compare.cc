/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.
// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/sorting.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/containers/generators/functor.hpp>

#include <time.h>
#include "test_util.hpp"

using namespace stapl;

const char alfa[] =
  {'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S',
   'T','U','V','W','X','Y','Z'};

struct rand_functor
{
  typedef char    index_type;
  typedef char    result_type;

  char operator()(index_type) const
  {
    return alfa[rand()%26];
  }
};

void correct_test()
{
  //Creation of pArrays
  typedef array<char>   p_array_char;
  typedef array_view<p_array_char>     view_array_char;
  p_array_char p_array1(5);
  p_array_char p_array2(5);
  p_array_char p_array3(5);
  p_array_char p_array4(5);
  p_array_char p_arraya(4);
  p_array_char p_arrayb(4);
  p_array_char p_arrayc(4);
  p_array_char p_arrayd(4);
  view_array_char pview1(p_array1);
  view_array_char pview2(p_array2);
  view_array_char pview3(p_array3);
  view_array_char pview4(p_array4);
  view_array_char pviewa(p_arraya);
  view_array_char pviewb(p_arrayb);
  view_array_char pviewc(p_arrayc);
  view_array_char pviewd(p_arrayd);

  //Fill them
  pview1[0]='b'; pview1[1]='b'; pview1[2]='c'; pview1[3]='d'; pview1[4]='e';
  pview2[0]='b'; pview2[1]='b'; pview2[2]='c'; pview2[3]='d'; pview2[4]='e';
  pview3[0]='b'; pview3[1]='a'; pview3[2]='b'; pview3[3]='d'; pview3[4]='e';
  pview4[0]='b'; pview4[1]='c'; pview4[2]='b'; pview4[3]='d'; pview4[4]='e';
  pviewa[0]='b'; pviewa[1]='b'; pviewa[2]='c'; pviewa[3]='d';
  pviewb[0]='b'; pviewb[1]='a'; pviewb[2]='b'; pviewb[3]='d';
  pviewc[0]='b'; pviewc[1]='c'; pviewc[2]='b'; pviewc[3]='d';
  pviewd[0]='a'; pviewd[1]='c'; pviewd[2]='b'; pviewd[3]='d';
  //Ensure the explicit initialization from all locations is finished.
  rmi_fence();

  //Test of function
  bool result1 = lexicographical_compare(pview1,pview2);
  bool result2 = lexicographical_compare(pview1,pview3);
  bool result3 = lexicographical_compare(pview1,pview4);
  bool result4 = lexicographical_compare(pview1,pviewa);
  bool result5 = lexicographical_compare(pview1,pviewb);
  bool result6 = lexicographical_compare(pview1,pviewc);
  bool result7 = lexicographical_compare(pview1,pviewd);

  //Result
  if (get_location_id()==0){
    if (!result1 && !result2 && result3 && !result4 && !result5 && result6 &&
        !result7) {
      std::cout << "[PASSED]" << std::endl;
    } else {
      std::cout << "[FAILED]" << std::endl;
    }
  }
}

void performance_test(long minelem2, long minelembis2)
{
  //Performance test
  bool result =false;
  unsigned long minelem = minelem2*get_num_locations();
  unsigned long minelembis = minelembis2*get_num_locations();

  typedef array<char>                       p_array_char;
  typedef array_view<p_array_char>          view_array_char;
  typedef functor_container<rand_functor>   rand_container_type;
  typedef array_view<rand_container_type>   rand_gen_view;

  rand_container_type rgc1(minelem, rand_functor());
  rand_container_type rgc2(minelembis, rand_functor());

  //Copy from a generator view into the parray
  p_array_char p_array1(minelem);
  p_array_char p_array2(minelembis);
  view_array_char pview1(p_array1);
  view_array_char pview2(p_array2);
  rand_gen_view rgen1(rgc1);
  rand_gen_view rgen2(rgc2);
  copy(rgen1,pview1);
  copy(rgen2,pview2);

  //Timer
  std::vector <double> vector_time;
  stapl::counter<stapl::default_timer> t;

  //Tests
  for (int i =0; i<= 32 ; i++){
    t.reset();
    t.start();
    result = lexicographical_compare(pview1,pview2);
    rmi_fence() ;
    vector_time.push_back(t.stop());
    if (get_location_id()==0)
      if ((pview1[0] > pview2[0] && result) ||
         (pview1[0] < pview2[0] && !result))
        std::cout << "[ERROR] lexicographical_compare didn't worked as attended"
                  << std::endl;
  }
  std::cout<<std::endl;
  compute_stats("test_optim_lexical_compare", vector_time);
  rmi_fence();
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  unsigned long minelem =10;
  unsigned long minelembis=10;
  if (get_location_id()==0){
    std::cout << "Testing lexicograhical_compare\t";
  }
  if (argc == 4){
    minelem = atol(argv[1]);
    minelembis = atol(argv[2]);
    if (atoi(argv[3]) == 1){
      correct_test();
    } else {
      performance_test(minelem,minelembis);
    }
  } else {
    if (get_location_id()==0)
      std::cout << "USAGE: test_lexicographic_compare "
                << "num_elements_array1 num_elements_array2 "
                << "1-correctness/2-performance" << std::endl;
  }

  return EXIT_SUCCESS;
}
