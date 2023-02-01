/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <algorithm>
#include <functional>
#include <iterator>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/sorting.hpp>
#include <stapl/algorithms/generator.hpp>

#include "test.h"

#include <boost/function.hpp>
#include <boost/bind.hpp>

using boost::bind;
using boost::function;
using stapl::sequence;
using stapl::random_sequence;


//////////////////////////////////////////////////////////////////////
/// @brief Test function for the radix_sort algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
/// @todo Fix memory consumption issues.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_radix_sort(const unsigned int n, const unsigned int i,
                     traits_type const& t, char const* name,
                     std::string const& ds = "none")
{
  typedef void                      ret_type;
  typedef test_one_view<ret_type>   test_type;
  typedef ret_type                  (*pfun1_t)(view_type);
  typedef ret_type                  (*sfun1_t)(iterator_type, iterator_type);
  typedef function<ret_type (view_type&)>                    pfp_t;
  typedef function<ret_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t) stapl::radix_sort<view_type>, _1));
  sfp_t sfp1(bind((sfun1_t) std::sort<iterator_type>, _1, _2));

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  // FIXME: resolve issues with memory consumption.
  // Decrease the data size so the runtime is on the appropriate scale.
  if ((ds != "tiny") && (ds != "none"))
  {
    if (ds == "small")
      n_elems /= 1000;
    else
      n_elems /= 10000;
  }

  timed_test_result tres1(name);
  test_type(data_descriptor<pcontainer_type, view_type, random_sequence>
      (n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems)))
    (tres1, i, pfp1, sfp1);

  test_report(tres1);
}


//////////////////////////////////////////////////////////////////////
/// @brief Test function for the sort algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_sort(const unsigned int n, const unsigned int i,
               traits_type const& t, char const* name,
               std::string const& ds = "none")
{
  typedef void                         ret_type;
  typedef test_one_view<ret_type>      test_type;
  typedef stapl::greater<data_type>    comp;

  typedef ret_type (*pfun1_t)(view_type&);
  typedef ret_type (*pfun2_t)(view_type&, comp);

  typedef ret_type (*sfun1_t)(iterator_type, iterator_type);
  typedef ret_type (*sfun2_t)(iterator_type, iterator_type, comp);

  typedef function<ret_type (view_type&)>                    pfp_t;
  typedef function<ret_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t) stapl::sort<view_type>, _1));
  pfp_t pfp2(bind((pfun2_t) stapl::sort<view_type, comp>, _1, comp()));

  sfp_t sfp1(bind((sfun1_t) std::sort<iterator_type>, _1, _2));
  sfp_t sfp2(bind((sfun2_t) std::sort<iterator_type, comp>, _1, _2, comp()));

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, random_sequence>(
      n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems)))
    (tres1, i, pfp1, sfp1);

  std::stringstream pred_name;
  pred_name << name << "-comp";
  timed_test_result tres2(pred_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, random_sequence>(
      n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems)))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}


//////////////////////////////////////////////////////////////////////
/// @brief Test function for the sample_sort algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_sample_sort(const unsigned int n, const unsigned int i,
                      traits_type const& t, char const* name,
                      std::string const& ds = "none")
{
  typedef void                         ret_type;
  typedef test_one_view<ret_type>      test_type;
  typedef stapl::greater<data_type>    comp;

  typedef ret_type (*pfun1_t)(view_type&);
  typedef ret_type (*pfun2_t)(view_type&, comp);

  typedef ret_type (*sfun1_t)(iterator_type, iterator_type);
  typedef ret_type (*sfun2_t)(iterator_type, iterator_type, comp);

  typedef function<ret_type (view_type&)>                    pfp_t;
  typedef function<ret_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t) stapl::sample_sort<view_type>, _1));
  pfp_t pfp2(bind((pfun2_t) stapl::sample_sort<view_type, comp>,
    _1, comp()));

  sfp_t sfp1(bind((sfun1_t) std::sort<iterator_type>, _1, _2));
  sfp_t sfp2(bind((sfun2_t) std::sort<iterator_type, comp>, _1, _2, comp()));

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  timed_test_result tres1(name);
  test_type(data_descriptor<pcontainer_type, view_type, random_sequence>
      (n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems)))
    (tres1, i, pfp1, sfp1);

  std::stringstream pred_name;
  pred_name << name << "-comp";
  timed_test_result tres2(pred_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, random_sequence>
      (n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems)))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}

//////////////////////////////////////////////////////////////////////
/// @brief Test function for the sort algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
/// @param name The name of the executed test.
/// @param ds Defines the data size. "none" indicates no size was specified.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_stable_sort(const unsigned int n, const unsigned int i,
               traits_type const& t, char const* name,
               std::string const& ds = "none")
{
  typedef void                         ret_type;
  typedef test_one_view<ret_type>      test_type;
  typedef stapl::greater<data_type>    comp;

  typedef ret_type (*pfun1_t)(view_type&);
  typedef ret_type (*pfun2_t)(view_type&, comp);

  typedef ret_type (*sfun1_t)(iterator_type, iterator_type);
  typedef ret_type (*sfun2_t)(iterator_type, iterator_type, comp);

  typedef function<ret_type (view_type&)>                    pfp_t;
  typedef function<ret_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t) stapl::stable_sort<view_type>, _1));
  pfp_t pfp2(bind((pfun2_t) stapl::stable_sort<view_type, comp>, _1, comp()));

  sfp_t sfp1(bind((sfun1_t) std::stable_sort<iterator_type>, _1, _2));
  sfp_t sfp2(bind((sfun2_t) std::stable_sort<iterator_type, comp>, _1, _2, comp()));

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, random_sequence>(
      n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems)))
    (tres1, i, pfp1, sfp1);

  std::stringstream pred_name;
  pred_name << name << "-comp";
  timed_test_result tres2(pred_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, random_sequence>(
      n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems)))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}


//////////////////////////////////////////////////////////////////////
/// @brief Test function for the is_sorted algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_is_sorted(const unsigned int n, const unsigned int i,
                    traits_type const& t, char const* name,
                    std::string const& ds = "none")
{
  typedef bool                             ret_type;
  typedef test_one_view<ret_type>          test_type;
  typedef stapl::less<data_type>           comp;

  typedef ret_type (*pfun1_t)(view_type const&);
  typedef ret_type (*pfun2_t)(view_type const&, comp);
  typedef ret_type (*sfun1_t)(iterator_type, iterator_type);
  typedef ret_type (*sfun2_t)(iterator_type, iterator_type, comp);

  typedef function<ret_type (view_type const&)>              pfp_t;
  typedef function<ret_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t) stapl::is_sorted<view_type>, _1));
  pfp_t pfp2(bind((pfun2_t) stapl::is_sorted<view_type, comp>, _1, comp()));

  sfp_t sfp1(bind((sfun1_t) std::is_sorted<iterator_type>, _1, _2));
  sfp_t sfp2(bind((sfun2_t)
    std::is_sorted<iterator_type, comp>, _1, _2, comp()));

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  timed_test_result tres1(name);
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres1, i, pfp1, sfp1);

  std::stringstream pred_name;
  pred_name << name << "-comp";
  timed_test_result tres2(pred_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres2, i, pfp2, sfp2);

  std::stringstream short_name;
  short_name << name << "-short";
  timed_test_result tres3(short_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0],
       sequence<data_type>(n_elems, -1)))
    (tres3, i, pfp1, sfp1);

  short_name << "_comp";
  timed_test_result tres4(short_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0],
       sequence<data_type>(n_elems, -1)))
    (tres4, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
  test_report(tres3);
  test_report(tres4);
}


//////////////////////////////////////////////////////////////////////
/// @brief Test function for the is_sorted_until algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_is_sorted_until(const unsigned int n, const unsigned int i,
                          traits_type const& t, char const* name,
                          std::string const& ds = "none")
{
  typedef view_type                  ret_type;
  typedef test_one_view<ret_type>    test_type;

  typedef stapl::less<data_type> comp;
  typedef ret_type(*pfun1_t)(view_type const&);
  typedef ret_type(*pfun2_t)(view_type const&, comp const&);

  typedef iterator_type(*sfun1_t)(iterator_type, iterator_type);
  typedef iterator_type(*sfun2_t)(iterator_type, iterator_type, comp);

  typedef function<ret_type (view_type const&)>                          pfp_t1;
  typedef function<iterator_type(iterator_type, iterator_type) >         sfp_t1;

  pfp_t1 pfp1(bind((pfun1_t)
    stapl::is_sorted_until<view_type>, _1));
  sfp_t1 sfp1(bind((sfun1_t)
    std::is_sorted_until<iterator_type>, _1, _2));

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  std::stringstream short_name;
  short_name << name << "-short";
  timed_test_result tres1(short_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, random_sequence>
    (n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)))
     (tres1, i, pfp1, sfp1);

  timed_test_result tres2(name);
  test_type(data_descriptor<pcontainer_type, view_type,sequence<data_type> >
    (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1,1)))
     (tres2, i, pfp1, sfp1);

  pfp_t1 pfp2(bind((pfun2_t)
    stapl::is_sorted_until<view_type, comp>, _1, comp()));
  sfp_t1 sfp2(bind((sfun2_t)
    std::is_sorted_until<iterator_type, comp>, _1, _2, comp()));

  short_name << "_comp";
  timed_test_result tres3(short_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, random_sequence>
    (n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)))
     (tres3, i, pfp2, sfp2);

  std::stringstream comp_name;
  comp_name << name << "-comp";
  timed_test_result tres4(comp_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
    (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1,1)))
     (tres4, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
  test_report(tres3);
  test_report(tres4);
}


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper for the parallel nth_element algorithm.
/// @param v The input view for the algorithm.
/// @param o Offset for the sort partition point.
///
/// The default '<' comparison function is used to compare elements.
//////////////////////////////////////////////////////////////////////
template<typename View>
void nth_element_wrapper(View& v, const unsigned int o)
{
  stapl::nth_element(v, v.begin() + o);
}

//////////////////////////////////////////////////////////////////////
/// @brief Wrapper for the parallel nth_element algorithm.
/// @param v The input view for the algorithm.
/// @param o Offset for the sort partition point.
/// @param c Comparison function for partitioning elements.
///
/// The comparison function c is used to compare elements.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comp>
void nth_element_wrapper(View& v, const unsigned int o, Comp c)
{
  stapl::nth_element(v, v.begin() + o, c);
}

//////////////////////////////////////////////////////////////////////
/// @brief Wrapper for the sequential nth_element algorithm.
/// @param first Begin iterator of the range sort.
/// @param last  End iterator of the range sort.
/// @param o     Offset for the sort partition point.
///
/// The default '<' comparison function is used to compare elements.
//////////////////////////////////////////////////////////////////////
template<typename Iterator>
void nth_element_wrapper(Iterator first, Iterator last, const unsigned int o)
{
  std::nth_element(first, first + o, last);
}

//////////////////////////////////////////////////////////////////////
/// @brief Wrapper for the sequential nth_element algorithm.
/// @param first Begin iterator of the range of elements to sort.
/// @param last  End iterator of the range of elements to sort.
/// @param o     Offset for the sort partition point.
/// @param c     Comparison function for partitioning elements.
///
/// The comparison function c is used to compare elements.
//////////////////////////////////////////////////////////////////////
template<typename Iterator, typename Comp>
void nth_element_wrapper(Iterator first, Iterator last, const unsigned int o,
                         Comp c)
{
  std::nth_element(first, first + o, last, c);
}

//////////////////////////////////////////////////////////////////////
/// @brief Validator to check the correctness of n_partition.
/// @param v The output of the parallel algorithm.
/// @param b Begin iterator of the sequential sorted range.
/// @param e End iterator of the sequential sorted range.
/// @param o Offset for the sort partition point.
/// @return bool Returns true in case of success, false otherwise.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Iterator>
bool nth_element_validator(View& v, Iterator b, Iterator e,
                           const unsigned int o)
{
  bool b1 = *(v.begin() + o) == *(b + o);

  std::sort(v.begin(), v.begin() + o);
  std::sort(b, b + o);

  bool b2 = std::equal(b, b + o, v.begin());

  std::sort(v.begin() + o, v.end());
  std::sort(b + o, e);

  bool b3 = std::equal(b + o, e, v.begin() + o);

  return b1 && b2 && b3;
}

//////////////////////////////////////////////////////////////////////
/// @brief Test function for the nth_element algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_nth_element(const unsigned int n, const unsigned int i,
                      traits_type const& t, char const* name,
                      std::string const& ds = "none")
{
  typedef void                         ret_t;
  typedef test_one_view<ret_t, 1>   test_t;
  typedef stapl::less<data_type>       comp_t;

  typedef ret_t (*pfun1_t)(view_type&, unsigned int);
  typedef ret_t (*pfun2_t)(view_type&, unsigned int, comp_t);

  typedef ret_t (*sfun1_t)(iterator_type, iterator_type, unsigned int);
  typedef ret_t (*sfun2_t)(iterator_type, iterator_type, unsigned int, comp_t);

  typedef
    bool (*valid_t)(view_type&, iterator_type, iterator_type, unsigned int);

  typedef function<ret_t (view_type&)>                              pfp_t;
  typedef function<ret_t (iterator_type, iterator_type)>            sfp_t;
  typedef function<bool (view_type&, iterator_type, iterator_type)> validator_t;

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  const unsigned int o = n_elems/2;

  validator_t validator(bind((valid_t)
    nth_element_validator<view_type, iterator_type>, _1, _2, _3, o));

  pfp_t pfp1(bind((pfun1_t) nth_element_wrapper<view_type>, _1, o));
  pfp_t pfp2(bind((pfun2_t)
    nth_element_wrapper<view_type, comp_t>, _1, o, comp_t()));

  sfp_t sfp1(bind((sfun1_t) nth_element_wrapper<iterator_type>, _1, _2, o));
  sfp_t sfp2(bind((sfun2_t)
    nth_element_wrapper<iterator_type, comp_t>, _1, _2, o, comp_t()));

  timed_test_result tres1(name);
  test_t(
    data_descriptor<pcontainer_type, view_type, random_sequence>(
      n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)))
    (tres1, i, pfp1, sfp1, validator);

  std::stringstream comp_name;
  comp_name << name << "-comp";
  timed_test_result tres2(comp_name.str().c_str());
  test_t(
    data_descriptor<pcontainer_type, view_type, random_sequence>(
      n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)))
    (tres2, i, pfp2, sfp2, validator);

  test_report(tres1);
  test_report(tres2);
}


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper for the parallel partial_sort algorithm.
/// @param v The input view for the algorithm.
/// @param o The offset to sort up to.
///
/// The default '<' comparison function is used to compare elements.
//////////////////////////////////////////////////////////////////////
template<typename View>
void partial_sort_wrapper(View& v, const unsigned long o)
{
  stapl::partial_sort(v, v.begin() + o);
}

//////////////////////////////////////////////////////////////////////
/// @brief Wrapper for the parallel partial_sort algorithm.
/// @param v The input view for the algorithm.
/// @param o The offset to sort up to.
/// @param c Comparison function for sorting elements.
///
/// The comparison function c is used to compare elements.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comp>
void partial_sort_wrapper(View& v, const unsigned long o, Comp c)
{
  stapl::partial_sort(v, v.begin() + o, c);
}

//////////////////////////////////////////////////////////////////////
/// @brief Wrapper for the sequential partial_sort algorithm.
/// @param first Begin iterator of the range of elements to sort.
/// @param last  End iterator of the range of elements to sort.
/// @param o     The offset to sort up to.
///
/// The default '<' comparison function is used to compare elements.
//////////////////////////////////////////////////////////////////////
template<typename Iterator>
void partial_sort_wrapper(Iterator first, Iterator last, const unsigned long o)
{
  std::partial_sort(first, first + o, last);
}

//////////////////////////////////////////////////////////////////////
/// @brief Wrapper for the sequential partial_sort algorithm.
/// @param first Begin iterator of the range of elements to sort.
/// @param last  End iterator of the range of elements to sort.
/// @param o     The offset to sort up to.
/// @param c     Comparison function for sorting elements.
///
/// The comparison function c is used to compare elements.
//////////////////////////////////////////////////////////////////////
template<typename Iterator, typename Comp>
void partial_sort_wrapper(Iterator first, Iterator last, const unsigned long o,
                          Comp c)
{
  std::partial_sort(first, first + o, last, c);
}

//////////////////////////////////////////////////////////////////////
/// @brief Validator to check the correctness of n_partition.
/// @param v The output of the parallel algorithm.
/// @param b Begin iterator of the output resulting container for the sequential
///   algorithm.
/// @param e End iterator of the output resulting container for the sequential
///   algorithm.
/// @param o The offset to sort up to.
/// @return bool Returns true in case of success, false otherwise.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Iterator>
bool partial_sort_validator(View& v, Iterator b, Iterator e,
                            const unsigned long o)
{
  return std::equal(v.begin(), v.begin() + o, b);
}

//////////////////////////////////////////////////////////////////////
/// @brief Test function for the partial_sort algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_partial_sort(const unsigned int n, const unsigned int i,
                       traits_type const& t, char const* name,
                       std::string const& ds = "none")
{
  typedef void                          ret_t;
  typedef test_one_view<ret_t, 1>    test_t;
  typedef stapl::less<data_type>        comp_t;

  typedef ret_t (*pfun1_t)(view_type&, const unsigned long);
  typedef ret_t (*pfun2_t)(view_type&, const unsigned long, comp_t);

  typedef ret_t (*sfun1_t)(iterator_type, iterator_type, const unsigned long);
  typedef ret_t (*sfun2_t)(iterator_type, iterator_type, const unsigned long,
                           comp_t);

  typedef bool (*valid_t)(view_type&, iterator_type, iterator_type,
                          const unsigned long);

  typedef function<ret_t (view_type&)>                              pfp_t;
  typedef function<ret_t (iterator_type, iterator_type)>            sfp_t;
  typedef function<bool (view_type&, iterator_type, iterator_type)> validator_t;

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  const unsigned long o = n_elems/2;

  validator_t validator(bind((valid_t)
    partial_sort_validator<view_type, iterator_type>, _1, _2, _3, o));

  pfp_t pfp1(bind((pfun1_t) partial_sort_wrapper<view_type>, _1, o));
  pfp_t pfp2(bind((pfun2_t)
    partial_sort_wrapper<view_type, comp_t>, _1, o, comp_t()));

  sfp_t sfp1(bind((sfun1_t) partial_sort_wrapper<iterator_type>, _1, _2, o));
  sfp_t sfp2(bind((sfun2_t)
    partial_sort_wrapper<iterator_type, comp_t>, _1, _2, o, comp_t()));

  timed_test_result tres1(name);
  test_t(
    data_descriptor<pcontainer_type, view_type, random_sequence>(
      n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)))
    (tres1, i, pfp1, sfp1, validator);

  std::stringstream comp_name;
  comp_name << name << "-comp";
  timed_test_result tres2(comp_name.str().c_str());
  test_t(
    data_descriptor<pcontainer_type, view_type, random_sequence>(
      n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)))
    (tres2, i, pfp2, sfp2, validator);

  test_report(tres1);
  test_report(tres2);
}


//////////////////////////////////////////////////////////////////////
/// @brief Validator to check the correctness of the parallel algorithm against
///   the sequential output.
/// @param piv      Input view given to the parallel algorithm.
/// @param pov      Output view from the parallel algorithm.
/// @param siibegin Begin input iterator to the sequential algorithm.
/// @param siiend   End input iterator to the sequential algorithm.
/// @param soibegin Begin input iterator resulting from the sequential
///   algorithm.
/// @param soiend   End output iterator resulting from the sequential algorithm.
/// @return bool Returns true in case of success, false otherwise.
//////////////////////////////////////////////////////////////////////
template<typename PInputView, typename POutputView,
         typename SInputIt, typename SOutputIt>
bool partial_sort_copy_validator(PInputView& piv, POutputView& pov,
                                 SInputIt siibegin, SInputIt siiend,
                                 SOutputIt soibegin, SOutputIt soiend)
{
  bool b1 = std::equal(siibegin, siiend, piv.begin());
  bool b2 = std::equal(soibegin, soiend, pov.begin());

  return b1 && b2;
}

//////////////////////////////////////////////////////////////////////
/// @brief Test function for the partial_sort_copy algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_partial_sort_copy(const unsigned int n, const unsigned int i,
                            traits_type const& t, char const* name,
                            std::string const& ds = "none")
{
  typedef view_type                       pret_t;
  typedef iterator_type                   sret_t;
  typedef test_two_view<pret_t>           test_t;
  typedef stapl::less<data_type>          comp_t;

  typedef function<pret_t (view_type, view_type)> pfp_t;
  typedef function<sret_t
    (iterator_type, iterator_type, iterator_type, iterator_type)> sfp_t;

  typedef pret_t (*pfun1_t)(view_type, view_type);
  typedef pret_t (*pfun2_t)(view_type, view_type, comp_t);
  typedef sret_t (*sfun1_t)
    (iterator_type, iterator_type, iterator_type, iterator_type);
  typedef sret_t (*sfun2_t)
    (iterator_type, iterator_type, iterator_type, iterator_type, comp_t);

  typedef bool (*valid_t)(view_type&, view_type&,
                    iterator_type, iterator_type, iterator_type, iterator_type);
  typedef function<bool (view_type&, view_type&,
                    iterator_type, iterator_type, iterator_type, iterator_type)>
            validator_t;

  pfp_t pfp1(bind((pfun1_t)
    stapl::partial_sort_copy<view_type, view_type>, _1, _2));
  pfp_t pfp2(bind((pfun2_t)
    stapl::partial_sort_copy<view_type, view_type, comp_t>,
    _1,_2, comp_t()));

  sfp_t sfp1(bind((sfun1_t)
    std::partial_sort_copy<iterator_type, iterator_type>, _1, _2, _3, _4));
  sfp_t sfp2(bind((sfun2_t)
    std::partial_sort_copy<iterator_type, iterator_type, comp_t>,
    _1, _2, _3, _4, comp_t()));

  validator_t validator(bind((valid_t)partial_sort_copy_validator
                           <view_type, view_type, iterator_type, iterator_type>,
                         _1, _2, _3, _4, _5, _6));

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  // Reduce the data size so the runtime is on the appropriate scale.
  if ((ds != "tiny") && (ds != "none"))
    n_elems /= 50;

  timed_test_result tres1(name);
  test_t(
    data_descriptor<pcontainer_type, view_type, random_sequence>(
      n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)),
    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
      n_elems/2, t.ct_create[0], t.vw_create[0], null_sequence<data_type>()))
    (tres1, i, pfp1, sfp1, validator);

  std::stringstream comp_name;
  comp_name << name << "-comp";
  timed_test_result tres2(comp_name.str().c_str());
  test_t(
    data_descriptor<pcontainer_type, view_type, random_sequence>(
      n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)),
    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
      n_elems/2, t.ct_create[0], t.vw_create[0], null_sequence<data_type>()))
    (tres2, i, pfp2, sfp2, validator);

  test_report(tres1);
  test_report(tres2);
}


template<typename T>
void test_lower_bound(const unsigned int n, const unsigned int i,
                      traits_type const& t, char const* name,
                      std::string const& ds = "none")
{
  typedef view_type::reference       ret_type;
  typedef test_one_view<ret_type>   test_type;
  typedef stapl::less<data_type>      comp;

  typedef ret_type (*pfun1_t)(view_type const&, data_type const&);
  typedef ret_type (*pfun2_t)(view_type const&, data_type const&, comp);
  typedef iterator_type
    (*sfun1_t)(iterator_type, iterator_type, data_type const&);
  typedef iterator_type
    (*sfun2_t)(iterator_type, iterator_type, data_type const&, comp);

  typedef function<ret_type (view_type const&)>                   pfp_t;
  typedef function<iterator_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t) stapl::lower_bound<view_type, data_type>, _1, -11));
  sfp_t sfp1(bind((sfun1_t) std::lower_bound<iterator_type, data_type>, _1, _2, -11));

  const unsigned long int n_elems = set_data_size(n, ds, "sorting");

  timed_test_result tres1(name);
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1,1)))
    (tres1, i, pfp1, sfp1);


  pfp_t pfp2(bind((pfun1_t) stapl::lower_bound<view_type, data_type>, _1, 5));
  sfp_t sfp2(bind((sfun1_t) std::lower_bound<iterator_type, data_type>, _1, _2, 5));

  std::stringstream comp_name;
  comp_name << name << "-comp";
  timed_test_result tres2(comp_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1,1)))
    (tres2, i, pfp2, sfp2);


  pfp_t pfp3(bind((pfun1_t) stapl::lower_bound<view_type, data_type>, _1, n/2));
  sfp_t sfp3(bind((sfun1_t) std::lower_bound<iterator_type, data_type>, _1, _2, n/2));

  std::stringstream short_name;
  short_name << name << "-short";
  timed_test_result tres3(short_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1,1)))
    (tres3, i, pfp3, sfp3);

  pfp_t pfp4(bind((pfun1_t) stapl::lower_bound<view_type, data_type>, _1, n));
  sfp_t sfp4(bind((sfun1_t) std::lower_bound<iterator_type, data_type>, _1, _2, n));

  short_name << "_comp";
  timed_test_result tres4(short_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1,1)))
    (tres4, i, pfp4, sfp4);

  test_report(tres1);
  test_report(tres2);
  test_report(tres3);
  test_report(tres4);
}

template<typename T>
void test_upper_bound(const unsigned int n, const unsigned int i,
                      traits_type const& t, char const* name,
                      std::string const& ds = "none")
{
  typedef view_type::reference       ret_type;
  typedef test_one_view<ret_type>   test_type;
  typedef stapl::less<data_type>      comp;

  typedef ret_type (*pfun1_t)(view_type const&, data_type const&);
  typedef ret_type (*pfun2_t)(view_type const&, data_type const&, comp);
  typedef iterator_type
    (*sfun1_t)(iterator_type, iterator_type, data_type const&);
  typedef iterator_type
    (*sfun2_t)(iterator_type, iterator_type, data_type const&, comp);

  typedef function<ret_type (view_type const&)>                         pfp_t;
  typedef function<iterator_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t) stapl::upper_bound<view_type, data_type>, _1, -11));
  sfp_t sfp1(bind((sfun1_t) std::upper_bound<iterator_type, data_type>, _1, _2, -11));

  const unsigned long int n_elems = set_data_size(n, ds, "sorting");

  timed_test_result tres1(name);
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1,1)))
    (tres1, i, pfp1, sfp1);


  pfp_t pfp2(bind((pfun1_t) stapl::upper_bound<view_type, data_type>, _1, 5));
  sfp_t sfp2(bind((sfun1_t) std::upper_bound<iterator_type, data_type>, _1, _2, 5));

  std::stringstream comp_name;
  comp_name << name << "-comp";
  timed_test_result tres2(comp_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1,1)))
    (tres2, i, pfp2, sfp2);


  pfp_t pfp3(bind((pfun1_t) stapl::upper_bound<view_type, data_type>, _1, n/2));
  sfp_t sfp3(bind((sfun1_t) std::upper_bound<iterator_type, data_type>, _1, _2, n/2));

  std::stringstream short_name;
  short_name << name << "-short";
  timed_test_result tres3(short_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1,1)))
    (tres3, i, pfp3, sfp3);

  pfp_t pfp4(bind((pfun1_t) stapl::upper_bound<view_type, data_type>, _1, n));
  sfp_t sfp4(bind((sfun1_t) std::upper_bound<iterator_type, data_type>, _1, _2, n));

  short_name << "_comp";
  timed_test_result tres4(short_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1,1)))
    (tres4, i, pfp4, sfp4);

  test_report(tres1);
  test_report(tres2);
  test_report(tres3);
  test_report(tres4);
}
/*
template<typename T>
void test_equal_range(const unsigned int n, const unsigned int i,
                      traits_type const& t, char const* name,
                      std::string const& ds = "none")
{
  using std::equal_range;
  using stapl::equal_range;

  typedef std::pair<view_type::iterator, view_type::iterator>  ret_type;
  typedef std::pair<iterator_type, iterator_type>              seq_ret_type;
  typedef test_one_view<ret_type>                              test_type;
  typedef stapl::greater<data_type>                              comp;

  typedef ret_type (*pfun1_t)(view_type const&, data_type const&);
  typedef ret_type (*pfun2_t)(view_type const&, data_type const&, comp);
  typedef seq_ret_type
    (*sfun1_t)(iterator_type, iterator_type, data_type const&);
  typedef seq_ret_type
    (*sfun2_t)(iterator_type, iterator_type, data_type const&, comp);

  typedef function<ret_type (view_type const&)>                  pfp_t;
  typedef function<seq_ret_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t) equal_range<view_type, data_type>, _1, 10));
  sfp_t sfp1(bind((sfun1_t) equal_range<iterator_type, data_type>, _1, _2, 10));

  const unsigned long int n_elems = set_data_size(n, ds, "sorting");

  timed_test_result tres1(name);
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1,1)))
    (tres1, i, pfp1, sfp1);

  pfp_t pfp2(bind((pfun2_t)
    equal_range<view_type, data_type,comp>, _1, 10, comp()));
  sfp_t sfp2(bind((sfun2_t)
    equal_range<iterator_type, data_type, comp>, _1, _2, 10, comp()));

  timed_test_result tres2(name);
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1,1)))
    (tres2, i, pfp2, sfp2);

  test_report(tres1&tres2);
}
*/

template<typename T>
void test_binary_search(const unsigned int n, const unsigned int i,
                        traits_type const& t, char const* name,
                        std::string const& ds = "none")
{
  using std::binary_search;
  using stapl::binary_search;

  typedef bool                       ret_type;
  typedef test_one_view<ret_type>    test_type;
  typedef stapl::greater<data_type>    comp;

  typedef ret_type (*pfun1_t)(view_type const&, data_type const&);
  typedef ret_type (*pfun2_t)(view_type const&, data_type, comp);
  typedef ret_type (*sfun1_t)(iterator_type, iterator_type, data_type const&);
  typedef ret_type
    (*sfun2_t)(iterator_type, iterator_type, data_type const&, comp);

  typedef function<ret_type (view_type const&)>              pfp_t;
  typedef function<ret_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t)stapl::binary_search<view_type>, _1, 5));
  sfp_t sfp1(bind((sfun1_t)std::binary_search<iterator_type, data_type>, _1, _2, 5));

  const unsigned long int n_elems = set_data_size(n, ds, "sorting");

  std::stringstream found1_name;
  found1_name << name << "-found_case1";
  timed_test_result tres1(found1_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres1, i, pfp1, sfp1);

  pfp_t pfp2(bind((pfun2_t)
    stapl::binary_search<view_type, comp>, _1, 5, comp()));
  sfp_t sfp2(bind((sfun2_t)
    std::binary_search<iterator_type, data_type, comp>, _1, _2, 5, comp()));

  found1_name << "_comp";
  timed_test_result tres2(found1_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres2, i, pfp2, sfp2);

  pfp_t pfp3(bind((pfun1_t) stapl::binary_search<view_type>, _1, 11));
  sfp_t sfp3(bind((sfun1_t)
    std::binary_search<iterator_type, data_type>, _1, _2, 11));

  std::stringstream found2_name;
  found2_name << name << "-found_case2";
  timed_test_result tres3(found2_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres3, i, pfp3, sfp3);

  pfp_t pfp4(bind((pfun2_t)
    stapl::binary_search<view_type, comp>, _1, 11, comp()));
  sfp_t sfp4(bind((sfun2_t)
    std::binary_search<iterator_type, data_type, comp>, _1, _2, 11, comp()));

  found2_name << "_comp";
  timed_test_result tres4(found2_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres4, i, pfp4, sfp4);

  //search for non existing element
  pfp_t pfp5(bind((pfun1_t) stapl::binary_search<view_type>, _1, n + 1));
  sfp_t sfp5(bind((sfun1_t)
    std::binary_search<iterator_type, data_type>, _1, _2, n + 1));

  timed_test_result tres5(name);
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres5, i, pfp5, sfp5);

  pfp_t pfp6(bind((pfun2_t)
    stapl::binary_search<view_type, comp>, _1, n + 1, comp()));
  sfp_t sfp6(bind((sfun2_t)
    std::binary_search<iterator_type, data_type, comp>, _1, _2, n + 1, comp()));

  std::stringstream comp_name;
  comp_name << name << "-comp";
  timed_test_result tres6(comp_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, sequence<data_type> >
      (n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres6, i, pfp6, sfp6);

  test_report(tres1);
  test_report(tres2);
  test_report(tres3);
  test_report(tres4);
  test_report(tres5);
  test_report(tres6);
}

template <typename T>
void test_merge(const unsigned int n, const unsigned i,
                traits_type const& t, char const* name,
                std::string const& ds = "none")
{
  typedef void                       ret_type;
  typedef iterator_type              ret_stl;
  typedef test_three_view<ret_stl>  test_type;

  typedef ret_type (*pfun_t)(view_type const&, view_type const&, view_type&);
  typedef ret_stl (*sfun_t)(iterator_type, iterator_type, iterator_type, iterator_type, iterator_type);

  function<ret_type (view_type, view_type, view_type)>
    pfp = stapl::merge<view_type, view_type, view_type>;

  function<ret_stl (iterator_type, iterator_type, iterator_type, iterator_type, iterator_type)>
    sfp(bind((sfun_t)std::merge<iterator_type, iterator_type, iterator_type>, _1, _2, _3, _4, _5));

  const unsigned long int n_elems = set_data_size(n, ds, "sorting");

  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(2, 2)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 2)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems*2, t.ct_create[2], t.vw_create[2], sequence<data_type>(1, 2)))
    (tres1, i, pfp, sfp);
  test_report(tres1);
}


//////////////////////////////////////////////////////////////////////
/// @brief Test function for the min_element algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_min_element(const unsigned int n, const unsigned int i,
                      traits_type const& t, char const* name,
                      std::string const& ds = "none")
{
  typedef view_type::reference       ret_type;
  typedef test_one_view<ret_type>    test_type;

  typedef stapl::less<data_type> comp;
  typedef ret_type(*pfun1_t)(view_type const&);
  typedef ret_type(*pfun2_t)(view_type const&, comp);
  typedef iterator_type(*sfun1_t)(iterator_type, iterator_type);
  typedef iterator_type(*sfun2_t)(iterator_type, iterator_type, comp);

  typedef function<ret_type (view_type const&)>                   pfp_t;
  typedef function<iterator_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t) stapl::min_element<view_type>, _1));
   sfp_t sfp1(bind((sfun1_t) std::min_element<iterator_type>, _1, _2));

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  timed_test_result tres1(name);
   test_type(data_descriptor<pcontainer_type, view_type, random_sequence>
    (n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)))
     (tres1, i, pfp1, sfp1);

 pfp_t pfp2(bind((pfun2_t) stapl::min_element<view_type, comp>, _1, comp()));
   sfp_t sfp2(bind((sfun2_t) std::min_element<iterator_type, comp>, _1, _2,
              comp()));

  std::stringstream comp_name;
  comp_name << name << "-comp";
  timed_test_result tres2(comp_name.str().c_str());
   test_type(data_descriptor<pcontainer_type, view_type, random_sequence>
    (n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)))
     (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}

//////////////////////////////////////////////////////////////////////
/// @brief Test function for the min_value algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_min_value(const unsigned int n, const unsigned int i,
                      traits_type const& t, char const* name,
                      std::string const& ds = "none")
{
  typedef data_type       ret_type;
  typedef test_one_view<ret_type, 1>    test_type;

  typedef stapl::less<data_type> comp;
  typedef ret_type(*pfun1_t)(view_type const&);
  typedef ret_type(*pfun2_t)(view_type const&, comp);
  typedef iterator_type(*sfun1_t)(iterator_type, iterator_type);
  typedef iterator_type(*sfun2_t)(iterator_type, iterator_type, comp);

  typedef function<ret_type (view_type const&)>                   pfp_t;
  typedef function<iterator_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t) stapl::min_value<view_type>, _1));
   sfp_t sfp1(bind((sfun1_t) std::min_element<iterator_type>, _1, _2));

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  timed_test_result tres1(name);
   test_type(data_descriptor<pcontainer_type, view_type, random_sequence>
    (n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)))
     (tres1, i, pfp1, sfp1);

 pfp_t pfp2(bind((pfun2_t) stapl::min_value<view_type, comp>, _1, comp()));
   sfp_t sfp2(bind((sfun2_t) std::min_element<iterator_type, comp>, _1, _2,
              comp()));

  std::stringstream comp_name;
  comp_name << name << "-comp";
  timed_test_result tres2(comp_name.str().c_str());
   test_type(data_descriptor<pcontainer_type, view_type, random_sequence>
    (n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)))
     (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}

//////////////////////////////////////////////////////////////////////
/// @brief Test function for the max_value algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_max_value(const unsigned int n, const unsigned int i,
                      traits_type const& t, char const* name,
                      std::string const& ds = "none")
{
  typedef data_type       ret_type;
  typedef test_one_view<ret_type, true>    test_type;

  typedef stapl::less<data_type> comp;
  typedef ret_type (*pfun1_t)(view_type const&);
  typedef ret_type (*pfun2_t)(view_type const&, comp);
  typedef iterator_type (*sfun1_t)(iterator_type, iterator_type);
  typedef iterator_type (*sfun2_t)(iterator_type, iterator_type, comp);

  typedef function<ret_type (view_type const&)>                   pfp_t;
  typedef function<iterator_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t) stapl::max_value<view_type>, _1));
  sfp_t sfp1(bind((sfun1_t) std::max_element<iterator_type>, _1, _2));

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  timed_test_result tres1(name);
  test_type(data_descriptor<pcontainer_type, view_type, random_sequence>
    (n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)))
    (tres1, i, pfp1, sfp1);

  pfp_t pfp2(bind((pfun2_t) stapl::max_value<view_type, comp>, _1, comp()));
  sfp_t sfp2(bind((sfun2_t) std::max_element<iterator_type, comp>, _1, _2,
             comp()));

  std::stringstream comp_name;
  comp_name << name << "-comp";
  timed_test_result tres2(comp_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, random_sequence>
    (n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}

//////////////////////////////////////////////////////////////////////
/// @brief Test function for the max_element algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_max_element(const unsigned int n, const unsigned int i,
                      traits_type const& t, char const* name,
                      std::string const& ds = "none")
{
  typedef view_type::reference       ret_type;
  typedef test_one_view<ret_type>    test_type;

  typedef stapl::less<data_type> comp;
  typedef ret_type (*pfun1_t)(view_type const&);
  typedef ret_type (*pfun2_t)(view_type const&, comp);
  typedef iterator_type (*sfun1_t)(iterator_type, iterator_type);
  typedef iterator_type (*sfun2_t)(iterator_type, iterator_type, comp);

  typedef function<ret_type (view_type const&)>                   pfp_t;
  typedef function<iterator_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t) stapl::max_element<view_type>, _1));
  sfp_t sfp1(bind((sfun1_t) std::max_element<iterator_type>, _1, _2));

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  timed_test_result tres1(name);
  test_type(data_descriptor<pcontainer_type, view_type, random_sequence>
    (n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)))
    (tres1, i, pfp1, sfp1);

  pfp_t pfp2(bind((pfun2_t) stapl::max_element<view_type, comp>, _1, comp()));
  sfp_t sfp2(bind((sfun2_t) std::max_element<iterator_type, comp>, _1, _2,
             comp()));

  std::stringstream comp_name;
  comp_name << name << "-comp";
  timed_test_result tres2(comp_name.str().c_str());
  test_type(data_descriptor<pcontainer_type, view_type, random_sequence>
    (n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems, true)))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}

//////////////////////////////////////////////////////////////////////
/// @brief Test function for the lexicographical_compare algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_lexicographical_compare(const unsigned int n, const unsigned int i,
                                  traits_type const& t, char const* name,
                                  std::string const& ds = "none")
{
  typedef bool                    ret_type;
  typedef test_two_view<ret_type> test_type;

  typedef ret_type (*pfun_t)(view_type const&, view_type const&);
  function<ret_type (view_type const&, view_type const&)>
     pfp(bind((pfun_t)stapl::lexicographical_compare<view_type, view_type>,
              _1, _2));

  typedef ret_type (*sfun_t)(iterator_type, iterator_type, iterator_type,
                             iterator_type);
  function<ret_type (iterator_type, iterator_type, iterator_type,
                     iterator_type)>
     sfp(bind((sfun_t) std::lexicographical_compare<iterator_type,
                                                    iterator_type>,
                _1, _2, _3, _4 ));

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

   /*timed_test_result tres(name);
   test_type(
     data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
       n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
     data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
       n_elems, t.ct_create[1], t.vw_create[1], null_sequence<data_type>()))
     (tres, i, pfp, sfp);*/

  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 1)))
    (tres1, i, pfp, sfp);


   test_report(tres1);
}


//////////////////////////////////////////////////////////////////////
/// @brief Test function for the next_permutation algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_next_permutation(const unsigned int n, const unsigned int i,
               traits_type const& t, char const* name,
               std::string const& ds = "none")
{
  typedef bool                         ret_type;
  typedef test_one_view<ret_type>      test_type;
  typedef stapl::greater<data_type>    comp;

  typedef ret_type (*pfun1_t)(view_type&);
  typedef ret_type (*pfun2_t)(view_type&, comp);

  typedef ret_type (*sfun1_t)(iterator_type, iterator_type);
  typedef ret_type (*sfun2_t)(iterator_type, iterator_type, comp);

  typedef function<ret_type (view_type&)>                    pfp_t;
  typedef function<ret_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t) stapl::next_permutation<view_type>, _1));
  pfp_t pfp2(bind((pfun2_t) stapl::next_permutation<view_type, comp>,
                            _1, comp()));

  sfp_t sfp1(bind((sfun1_t) std::next_permutation<iterator_type>, _1, _2));
  sfp_t sfp2(bind((sfun2_t) std::next_permutation<iterator_type, comp>,
             _1, _2, comp()));

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, random_sequence>(
      n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems)))
    (tres1, i, pfp1, sfp1);

  std::stringstream comp_name;
  comp_name << name << "-comp";
  timed_test_result tres2(comp_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, random_sequence>(
      n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems)))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}

//////////////////////////////////////////////////////////////////////
/// @brief Test function for the prev_permutation algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
/// @bug   prev_permutation gives different results with same 'comp' function.
///   For example, using stapl::greater<data_type> in both stapl and std
///   prev_permutation. On the other hand, using stapl::greater<data_type> in
///   stapl::prev_permutation and std::less<data_type> in std::prev_permutation
///   gives the same result. This may be due to std specification for 'comp':
///   comp -	comparison function object (i.e. an object that satisfies the
///   requirements of Compare) which returns 'TRUE' if the first argument is
///   'LESS' than the second, or something is upside-down in
///   lexicographical_permutation.
/// @bug   When the input is the first sequence of all permutations,
///   stapl::prev_permutation resturns the same, meanwhile std::prev_permutation
///   returns the last sequence of all permutations.
///   For example, on input [0, 1, 2], stapl returns [0, 1, 2] and std
///   returns [2, 1, 0].
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_prev_permutation(const unsigned int n, const unsigned int i,
               traits_type const& t, char const* name,
               std::string const& ds = "none")
{
  typedef bool                         ret_type;
  typedef test_one_view<ret_type>      test_type;
  typedef stapl::greater<data_type>    comp;

  typedef ret_type (*pfun1_t)(view_type&);
  typedef ret_type (*pfun2_t)(view_type&, comp);

  typedef ret_type (*sfun1_t)(iterator_type, iterator_type);
  typedef ret_type (*sfun2_t)(iterator_type, iterator_type, comp);

  typedef function<ret_type (view_type&)>                    pfp_t;
  typedef function<ret_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1(bind((pfun1_t) stapl::prev_permutation<view_type>, _1));
  pfp_t pfp2(bind((pfun2_t) stapl::prev_permutation<view_type, comp>,
                            _1, comp()));

  sfp_t sfp1(bind((sfun1_t) std::prev_permutation<iterator_type>, _1, _2));
  sfp_t sfp2(bind((sfun2_t) std::prev_permutation<iterator_type, comp>,
             _1, _2, comp()));

  unsigned long int n_elems = set_data_size(n, ds, "sorting");

  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, random_sequence>(
      n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems)))
    (tres1, i, pfp1, sfp1);

  std::stringstream comp_name;
  comp_name << name << "-comp";
  timed_test_result tres2(comp_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, random_sequence>(
      n_elems, t.ct_create[0], t.vw_create[0], random_sequence(n_elems)))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef void (*test_func_type)(const unsigned int, const unsigned int,
                                 traits_type const&, char const*,
                                 std::string const&);

/*
 *  This is the list of algorithms tested with the old views
 */
/*
  test_pair<test_func_type> tests[] = {
    {"sort",           test_sort<data_type, pcontainer_type>},
    {"stable_sort",    test_stable_sort<data_type, pcontainer_type>},
    {"partial_sort",   test_partial_sort<data_type, pcontainer_type>},
    {"partial_sort_copy",
     test_partial_sort_copy<data_type, pcontainer_type>},
#ifndef TEST_P_LIST
    {"sample_sort",       test_sample_sort<int>},
    {"nth_element",       test_nth_element<int>},
#endif
    {"is_sorted",         test_is_sorted<int>},
    {"lower_bound",       test_lower_bound<int>},
    {"upper_bound",       test_upper_bound<int>},
    {"equal_range",       test_equal_range<int>},

    {"merge",         test_merge<data_type, pcontainer_type>},
    {"inplace_merge", test_inplace_merge<data_type, pcontainer_type>},

    {"set_includes",     test_set_includes<data_type, pcontainer_type>},
    {"set_union",        test_set_union<data_type, pcontainer_type>},
    {"set_intersection", test_set_intersection<data_type, pcontainer_type>},
    {"set_difference",   test_set_difference<data_type, pcontainer_type>},
    {"set_symmetric_difference",
     test_set_symmetric_difference<data_type, pcontainer_type>},

    {"lexicographical_compare",
      test_lexicographical_compare<data_type, pcontainer_type>},
    {"lexicographical_compare_3way", test_sort<data_type, pcontainer_type>},

    {"prev_permutation", test_prev_permutation<data_type, pcontainer_type>},
    {"next_permutation", test_next_permutation<data_type, pcontainer_type>}
  };
*/

  test_pair<test_func_type> tests[] = {
    {"stable_sort",             test_stable_sort<int>},
    {"partial_sort",            test_partial_sort<int>},
    {"partial_sort_copy",       test_partial_sort_copy<int>},
    {"sample_sort",             test_sample_sort<int>},
    {"sort",                    test_sort<int>},
    {"nth_element",             test_nth_element<int>},
    {"is_sorted",               test_is_sorted<int>},
    {"is_sorted_until",         test_is_sorted_until<int>},
    {"radix_sort",              test_radix_sort<int>},
    {"lower_bound",             test_lower_bound<int>},
    {"upper_bound",             test_upper_bound<int>},
// {"equal_range",       test_equal_range},
//    {"merge",                   test_merge<int>},
    {"binary_search",           test_binary_search<int>},
//    {"inplace_merge", test_inplace_merge},
    {"min_element",             test_min_element<int>},
    {"min_value",               test_min_value<int>},
    {"max_element",             test_max_element<int>},
    {"max_value",               test_max_value<int>},
    {"lexicographical_compare", test_lexicographical_compare<int>},
    {"prev_permutation",        test_prev_permutation<int>},
    {"next_permutation",        test_next_permutation<int>}
  };

  test_execute(argc, argv, tests,
               tests+(sizeof (tests)/sizeof (test_pair<test_func_type>)));

  return EXIT_SUCCESS;
}
