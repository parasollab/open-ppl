/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/containers/generators/functor.hpp>
#include "../../test_report.hpp"

using namespace stapl;

//random char generator to test find_end algorithm
const char alfa[] =
{'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s',
 't','u','v','w','x','y','z'};

struct rand_functor
{
  typedef char    index_type;
  typedef char    result_type;

  char operator()(index_type) const
  {
    return alfa[rand()%26];
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  srand(get_location_id()+time(NULL));

  size_t elt=1000*get_num_locations(); //number of characters in the text

  std::string pattern1 = "firstpattern";
  std::string pattern2 = "secondpattern";
  std::string pattern3 = "thirdpattern";
  std::string pattern4 = "helloiamnotinthistext";

  typedef array<char>                              p_char_type;
  typedef array_view<p_char_type>                  pcharView;
  typedef pcharView::reference                     Ref_t;
  typedef functor_container<rand_functor>          rand_container_type;
  typedef array_view<rand_container_type>          randGenView;


  p_char_type p_text(elt);
  pcharView text_view(p_text);

  p_char_type p_pattern1(pattern1.size());
  pcharView pattern1_view(p_pattern1);
  p_char_type p_pattern2(pattern2.size());
  pcharView pattern2_view(p_pattern2);
  p_char_type p_pattern3(pattern3.size());
  pcharView pattern3_view(p_pattern3);
  p_char_type p_pattern4(pattern4.size());
  pcharView pattern4_view(p_pattern4);

  if (get_location_id()==0) {
    for (size_t i=0; i<pattern1.size();i++)
      pattern1_view[i]=pattern1[i];

    for (size_t i=0; i<pattern2.size();i++)
      pattern2_view[i]=pattern2[i];

    for (size_t i=0; i<pattern3.size();i++)
      pattern3_view[i]=pattern3[i];

    for (size_t i=0; i<pattern4.size();i++)
      pattern4_view[i]=pattern4[i];
  }
  rmi_fence();

  rand_container_type rgc(elt,rand_functor());
  randGenView rgenv(rgc);

  // Copy from a generator view into the parray
  copy(rgenv,text_view);

  if (get_location_id()==0) {
    for (size_t i=0; i<pattern1.size();i++)
      text_view[i]=pattern1[i];

    for (size_t i=0; i<pattern2.size();i++) {
      text_view[28*get_num_locations()+i]=pattern2[i];
      text_view[15*get_num_locations()+i]=pattern2[i];
    }

    for (size_t i=0; i<pattern3.size();i++) {
      text_view[50*get_num_locations()+i]=pattern3[i];
      text_view[elt-pattern3.size()+i]=pattern3[i];
    }
  }
  rmi_fence();

  Ref_t result1=find_end(text_view, pattern1_view);

  Ref_t result2=find_end(text_view, pattern2_view);

  Ref_t result3=find_end(text_view, pattern3_view);

  Ref_t result4=find_end(text_view, pattern4_view);

  bool res = (index_of(result1) == 0 &&
              index_of(result2) == 28*get_num_locations() &&
              index_of(result3) == elt-pattern3.size() &&
              is_null_reference(result4));

  STAPL_TEST_REPORT(res,"Testing find_end");

  return EXIT_SUCCESS;
}
