/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <numeric>
#include <functional>
#include <iterator>
#include <stapl/algorithms/functional.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/numeric.hpp>
#include <stapl/algorithms/generator.hpp>
#include <stapl/algorithms/sequential/weighted_inner_product.hpp>
#include <stapl/algorithms/sequential/weighted_norm.hpp>
#include "test.h"
#include <boost/function.hpp>
#include <boost/bind.hpp>

using boost::bind;
using boost::function;


template<typename T>
void test_accumulate(const unsigned long int n, const unsigned int i,
                     traits_type const& t, char const* name,
                     std::string const& ds)
{
  typedef data_type               ret_type;
  typedef test_one_view<ret_type> test_type;

  typedef std::plus<data_type>   add;
  typedef stapl::plus<data_type> par_add;

  typedef ret_type (*pfun1_t)(view_type const&, data_type);
  typedef ret_type (*pfun2_t)(view_type const&, data_type, par_add);
  typedef ret_type (*sfun1_t)(iterator_type, iterator_type, data_type);
  typedef ret_type (*sfun2_t)(iterator_type, iterator_type, data_type, add);

  const unsigned long int n_elems = set_data_size(n, ds, "numeric");

  const data_type init = 0;

  function<ret_type (view_type const&)> pfp1
    (bind((pfun1_t) accumulate<view_type>, _1, init));

  function<ret_type (iterator_type, iterator_type)> sfp1
    (bind((sfun1_t) std::accumulate<iterator_type, data_type>, _1, _2, init));

  function<ret_type (view_type const&)> pfp2
    (bind((pfun2_t) accumulate<view_type>, _1, init, par_add()));

  function<ret_type (iterator_type, iterator_type)> sfp2 =
    (bind((sfun2_t) std::accumulate<iterator_type, data_type>,
          _1, _2, init, add()));

  stapl::sequence<data_type> seq(1,1);

  timed_test_result tres1(name);

  std::stringstream op_name;
  op_name << name << "-op";
  timed_test_result tres2(op_name.str().c_str());

  test_type(
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], seq))
    (tres1, i, pfp1, sfp1);

  test_type(
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], seq))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}


template<typename T>
void test_inner_product(const unsigned long int n, const unsigned int i,
                        traits_type const& t, char const* name,
                        std::string const& ds)
{
  typedef data_type                   ret_type;
  typedef test_two_view<ret_type>     test_type;
  typedef std::plus<data_type>        fun1;
  typedef std::multiplies<data_type>  fun2;

  data_type init = 0;
  stapl::sequence<data_type> seq(1,1);

  typedef ret_type (*pfun1_t)(view_type const&, view_type const&, data_type);
  typedef ret_type (*sfun1_t)(iterator_type, iterator_type, iterator_type,
                              data_type);

  const unsigned long int n_elems = set_data_size(n, ds, "numeric");

  function<ret_type (view_type const&, view_type const&)> pfp1
    (bind((pfun1_t) inner_product<view_type, view_type>,
          _1, _2, init));

  function<ret_type (iterator_type, iterator_type, iterator_type)> sfp1
    (bind((sfun1_t) std::inner_product<iterator_type, iterator_type, data_type>,
          _1, _2, _3, init));

  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], seq),
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], seq))
    (tres1, i, pfp1, sfp1);

  typedef ret_type (*pfun2_t)(view_type const&, view_type const&, data_type,
                              fun1, fun2);
  typedef ret_type (*sfun2_t)(iterator_type, iterator_type, iterator_type,
                              data_type, fun1, fun2);

  function<ret_type (view_type const&, view_type const&)> pfp2 =
    bind((pfun2_t) inner_product<view_type, view_type, fun1, fun2>,
         _1, _2, init, fun1(), fun2());

  function<ret_type (iterator_type, iterator_type, iterator_type)> sfp2 =
    bind((sfun2_t) std::inner_product<iterator_type, iterator_type, data_type,
                                      fun1, fun2>,
         _1, _2, _3, init, fun1(), fun2());

  std::stringstream op_name;
  op_name << name << "-op";
  timed_test_result tres2(op_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], seq),
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], seq))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}


template<typename T>
void test_weighted_inner_product(const unsigned long int n,
                                 const unsigned int i,
                                 traits_type const& t, char const* name,
                                 std::string const& ds)
{
  typedef data_type                   ret_type;
  typedef test_three_view<ret_type>   test_type;
  typedef stapl::plus<data_type>        fun1;
  typedef stapl::multiplies<data_type>  fun2;

  data_type init = 0;
  stapl::sequence<data_type> seq(1,1);

  using pfun1_t = ret_type (*) (view_type const&, view_type const&,
                                view_type const&, data_type);
  using sfun1_t = ret_type (*) (iterator_type, iterator_type, iterator_type,
                                iterator_type, data_type);

  const unsigned long int n_elems = set_data_size(n, ds, "numeric");

  function<ret_type (view_type const&, view_type const&, view_type const&)>
    pfp1{ bind((pfun1_t) stapl::weighted_inner_product<view_type, view_type,
                                                       view_type>,
                                                       _1, _2, _3, init)};

  function<ret_type (iterator_type, iterator_type, iterator_type,
                     iterator_type)> sfp1{
    bind((sfun1_t) stapl::sequential::weighted_inner_product< iterator_type,
                     iterator_type, iterator_type>,
                     _1, _2, _3, _4, init)};

  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], seq),
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], seq),
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[2], t.vw_create[2], seq))
    (tres1, i, pfp1, sfp1);

  typedef ret_type (*pfun2_t)(view_type const&, view_type const&,
                              view_type const&, data_type, fun1, fun2);
  typedef ret_type (*sfun2_t)(iterator_type, iterator_type, iterator_type,
                              iterator_type, data_type, fun1, fun2);

  function<ret_type (view_type const&, view_type const&, view_type const&)>
    pfp2 = bind((pfun2_t) stapl::weighted_inner_product<view_type, view_type,
                                                        view_type, fun1, fun2>,
                                                        _1, _2, _3, init,
                                                        fun1(), fun2());

   function<ret_type (iterator_type, iterator_type, iterator_type,
                      iterator_type)> sfp2 =
    bind((sfun2_t) stapl::sequential::weighted_inner_product< iterator_type,
                     iterator_type, iterator_type, data_type, fun1, fun2>,
                     _1, _2, _3, _4, init, fun1(), fun2());

  std::stringstream op_name;
  op_name << name << "-op";
  timed_test_result tres2(op_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], seq),
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], seq),
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[2], t.vw_create[2], seq))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}


template<typename T>
void test_weighted_norm(const unsigned long int n,
                                 const unsigned int i,
                                 traits_type const& t, char const* name,
                                 std::string const& ds)
{
  typedef data_type                     ret_type;
  typedef test_two_view<ret_type>       test_type;
  typedef stapl::plus<data_type>        fun1;
  typedef stapl::multiplies<data_type>  fun2;

  data_type init = 0;
  stapl::sequence<data_type> seq(1,1);

  using pfun1_t = ret_type (*) (view_type const&, view_type const&);
  using sfun1_t = ret_type (*) (iterator_type, iterator_type, iterator_type,
                                data_type);

  const unsigned long int n_elems = set_data_size(n, ds, "numeric");

  function<ret_type (view_type const&, view_type const&)>
    pfp1{ bind((pfun1_t) stapl::weighted_norm<view_type, view_type>, _1, _2)};

  function<ret_type (iterator_type, iterator_type, iterator_type)> sfp1{
    bind((sfun1_t) stapl::sequential::weighted_norm< iterator_type,
                     iterator_type>,
                     _1, _2, _3, init)};

  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], seq),
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], seq))
    (tres1, i, pfp1, sfp1);

  typedef ret_type (*pfun2_t)(view_type const&, view_type const&, fun1, fun2);
  typedef ret_type (*sfun2_t)(iterator_type, iterator_type, iterator_type,
                              data_type, fun1, fun2);

  function<ret_type (view_type const&, view_type const&)>
    pfp2 = bind((pfun2_t) stapl::weighted_norm<view_type, view_type,
                            fun1, fun2>,
                            _1, _2, fun1(), fun2());

  function<ret_type (iterator_type, iterator_type, iterator_type)> sfp2 =
    bind((sfun2_t) stapl::sequential::weighted_norm< iterator_type,
                     iterator_type, data_type, fun1, fun2>,
                     _1, _2, _3, init, fun1(), fun2());

  std::stringstream op_name;
  op_name << name << "-op";
  timed_test_result tres2(op_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], seq),
    data_descriptor<pcontainer_type, view_type, stapl::sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], seq))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}

template<typename T>
void test_partial_sum(const unsigned long int n, const unsigned int i,
                      traits_type const& t, char const* name,
                      std::string const& ds)
{
  typedef void             ret_type;
  //  typedef test_two_view<ret_type, true>    test_type;
  typedef test_two_view<ret_type>    test_type;

  typedef view_type::value_type (*pfun_t)(view_type const&, view_type const&,
                                          bool);
  typedef iterator_type (*sfun_t)(iterator_type, iterator_type, iterator_type);

  typedef function<view_type::value_type (view_type const&, view_type const&)>
            pfp_t;
  typedef function<void (iterator_type, iterator_type, iterator_type)> sfp_t;

  const unsigned long int n_elems = set_data_size(n, ds, "numeric");

  pfp_t pfp = bind((pfun_t) stapl::partial_sum<view_type, view_type>,
                   _1, _2, false);
  sfp_t sfp  = bind((sfun_t) std::partial_sum<iterator_type, iterator_type>,
                    _1, _2, _3);

  timed_test_result tres1(name);

  sequence<data_type>       seq(1,0);
  null_sequence<data_type>  null_seq;

  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], seq),
    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], null_seq))
    (tres1, i, pfp, sfp);

  test_report(tres1);
}

template<typename T>
void test_adjacent_difference(const unsigned long int n, const unsigned int i,
                              traits_type const& t, char const* name,
                              std::string const& ds)
{
  typedef void                    ret_type;
  typedef test_two_view<ret_type> test_type;

  typedef ret_type (*pfun_t)(view_type const&, view_type const&);
  typedef iterator_type (*sfun_t)(iterator_type, iterator_type, iterator_type);

  typedef function<ret_type (view_type const&, view_type const&)>       pfp_t;
  typedef function<void (iterator_type, iterator_type, iterator_type)>  sfp_t;

  const unsigned long int n_elems = set_data_size(n, ds, "numeric");

  pfp_t pfp (bind((pfun_t) stapl::adjacent_difference<view_type, view_type>,
                  _1, _2));
  sfp_t sfp (bind((sfun_t) std::adjacent_difference<iterator_type,
                                                    iterator_type>,
                  _1, _2, _3));

  timed_test_result tres(name);

  sequence<data_type>       seq(0,2);
  null_sequence<data_type>  null_seq;

  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], seq),
    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], null_seq))
    (tres, i, pfp, sfp);

  test_report(tres);
}

template<typename T>
void test_iota(const unsigned long int n, const unsigned int i,
               traits_type const& t, char const* name,
               std::string const& ds)
{
  typedef void                      ret_type;
  typedef test_one_view<ret_type>   test_type;

  typedef ret_type (*pfun_t)(view_type const&, data_type const&);
  typedef ret_type (*sfun_t)(iterator_type, iterator_type, data_type);

  const unsigned long int n_elems = set_data_size(n, ds, "numeric");

  function<ret_type (view_type const&)>
    pfp(bind((pfun_t) stapl::iota<view_type>, _1, 5));

  function<ret_type (iterator_type, iterator_type)>
    sfp(bind((sfun_t) std::iota<iterator_type, data_type>, _1, _2, 5));

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres, i, pfp, sfp);

  test_report(tres);
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef void (*test_func_type)(const unsigned long int, const unsigned int,
                                 traits_type const&, char const*,
                                 std::string const&);
  test_pair<test_func_type> tests[] =
  {
    {"iota",                   test_iota<int>},
    {"accumulate",             test_accumulate<int>},
    {"inner_product",          test_inner_product<int>},
    {"weighted_inner_product", test_weighted_inner_product<int>},
    {"weighted_norm",          test_weighted_norm<int>},
    {"partial_sum",            test_partial_sum<int>},
    {"adjacent_difference",    test_adjacent_difference<int>}
  };

  test_execute(argc, argv, tests,
               tests+(sizeof (tests)/sizeof (test_pair<test_func_type>)));

  return EXIT_SUCCESS;
}
