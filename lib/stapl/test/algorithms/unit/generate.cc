/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <stapl/array.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/numeric.hpp>
#include <time.h>
#include "test_util.hpp"

using namespace stapl;

struct wf_random {
  typedef double result_type;

  wf_random()
  {
    srand48(get_location_id());
  }

  double operator()(){
    return drand48();
  }
};

struct wf_set5 {
  typedef int result_type;

  int operator()(){
    return 5;
  }
};

struct wf_set7 {
  typedef int result_type;

  int operator()(){
    return 7;
  }
};

void correct_test(size_t nelem) {
  if (get_location_id() ==0)
    std::cout << "Testing generate and generate_n...\t";

  bool result1 = false;
  bool result2 = false;
  typedef array<int>         p_array_int;
  typedef array_view<p_array_int>     view_array_int;
  p_array_int p_array_test(nelem);
  p_array_int p_array_reference(nelem, 5);
  p_array_int p_array_seven(nelem, 7);
  view_array_int p_view_test(p_array_test);
  view_array_int p_view_reference(p_array_reference);
  view_array_int p_view_seven(p_array_seven,
    view_array_int::domain_type(1,p_view_test.size()-5));
  view_array_int p_view_trunc(p_view_test.container(),
    view_array_int::domain_type(1,p_view_test.size()-5));

  //Tests
  generate(p_view_test, wf_set5());
  if (stapl::equal(p_view_test,p_view_reference))
    result1=true;

  generate_n(p_view_test, 1, p_view_test.size()-5, wf_set7());
  if (stapl::equal(p_view_trunc,p_view_seven))
    result2=true;

  //Print result
  if (get_location_id() ==0){
    if (result1 && result2){
      std::cout <<"[PASSED]"<<std::endl;
    } else {
      std::cout <<"[FAILED]"<<std::endl;
    }
  }
  rmi_fence();
}

void performance_test(long nelem2){
  if (get_location_id() ==0)
    std::cout << "Timing generate and generate_n" <<std::endl;

  unsigned long nelem = nelem2*get_num_locations();
  typedef array<double> p_array_double;
  typedef array_view<p_array_double>     view_array_double;
  stapl::counter<stapl::default_timer> t;
  std::vector <double> vector_time;
  std::vector <double> vector_time2;
  p_array_double p_array_test(nelem);
  view_array_double p_view_test(p_array_test);
  view_array_double p_view_trunc(p_view_test.container(),
                      view_array_double::domain_type(1,p_view_test.size()-5));

  for (int i=0;i<=32;i++){
    //Already existing pgenerate
    t.reset();
    rmi_fence();
    t.start();
    generate(p_view_test,wf_random());
    if (accumulate(p_view_test,0.000)/nelem < 0.3 ||
        accumulate(p_view_test,0.000)/nelem > 0.7)
      if (get_location_id()==0)
        std::cout <<"[ERROR] generate didn't worked as well"<<std::endl;

    rmi_fence();
    vector_time.push_back(t.stop());

    //pgenerate_n
    t.reset();
    rmi_fence();
    t.start();
    generate_n(p_view_test,1,p_view_test.size()-5,wf_random());
    rmi_fence();
    vector_time2.push_back(t.stop());
    if (accumulate(p_view_trunc,0.000)/(nelem-6) < 0.3 ||
        accumulate(p_view_trunc,0.000)/(nelem-6) > 0.8)
      if (get_location_id()==0)
        std::cout <<"[ERROR] generate_n didn't worked as well"<<std::endl;
  }
  //Print results
  compute_stats("test_generate", vector_time);
  compute_stats("test_generate_n", vector_time2);
  rmi_fence();
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  long nelem = 10;
  if (argc == 3) {
    nelem = atol(argv[1]);
    if (atoi(argv[2]) == 1){
      correct_test(nelem);
    } else {
      performance_test(nelem);
    }
  } else {
    if (get_location_id() == 0)
      std::cout << "[USAGE]: test_generate num_elements "
                << "1-correctness/2-performance\n"
                << std::endl;
  }
  return EXIT_SUCCESS;
}
