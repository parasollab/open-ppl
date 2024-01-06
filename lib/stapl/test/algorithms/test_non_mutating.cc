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
#include <stapl/algorithms/generator.hpp>
#include "test.h"
#include <boost/bind.hpp>
#include <boost/function.hpp>


using boost::bind;
using boost::function;


template <typename T>
void test_find(const unsigned int n, const unsigned i,
               traits_type const& t, char const* name,
               std::string const& ds = "none")
{
  typedef view_type::reference        ret_type;
  typedef test_one_view<ret_type>    test_type;

  typedef ret_type (*pfun_t)(view_type const&, data_type const&);
  typedef iterator_type (*sfun_t)(iterator_type, iterator_type,
                                  data_type const&);

  typedef function<ret_type      (view_type const&)>              pfp_t;
  typedef function<iterator_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1 (bind((pfun_t) stapl::find<view_type,data_type>, _1, 42));
  sfp_t sfp1 (bind((sfun_t) std::find<iterator_type, data_type>, _1, _2, 42));

  pfp_t pfp2 (bind((pfun_t) stapl::find<view_type,data_type>, _1, -1));
  sfp_t sfp2 (bind((sfun_t) std::find<iterator_type, data_type>, _1, _2, -1));

  pfp_t pfp3 (bind((pfun_t) stapl::find<view_type,data_type>, _1, 1));
  sfp_t sfp3 (bind((sfun_t) std::find<iterator_type, data_type>, _1, _2, 1));

  unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  // one match
  std::stringstream short_name;
  short_name << name << "-one_match";
  timed_test_result tres1(short_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres1, i, pfp1, sfp1);

  // no match
  timed_test_result tres2(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres2, i, pfp2, sfp2);

  // multiple matches
  std::stringstream multi_name;
  multi_name << name << "-multi_match";
  timed_test_result tres3(multi_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, block_sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0],
      block_sequence<data_type>(1, 10, 1)))
    (tres3, i, pfp3, sfp3);

  test_report(tres1);
  test_report(tres2);
  test_report(tres3);
}


template <typename T>
void test_find_if(const unsigned int n, const unsigned i,
                  traits_type const& t, char const* name,
                  std::string const& ds = "none")
{
  typedef view_type::reference        ret_type;
  typedef test_one_view<ret_type>    test_type;
  typedef std::equal_to<data_type>   fun;
  typedef stapl::binder2nd<fun,data_type,false> const&        pred;

  typedef ret_type (*pfun_t)(view_type const&, pred);
  typedef iterator_type (*sfun_t)(iterator_type, iterator_type, pred);

  typedef function<ret_type      (view_type const&)>              pfp_t;
  typedef function<iterator_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1 (bind((pfun_t) stapl::find_if<view_type, pred>,
                   _1, stapl::bind2nd(fun(), data_type(42))));
  sfp_t sfp1 (bind((sfun_t) std::find_if<iterator_type, pred>,
                   _1, _2, stapl::bind2nd(fun(), data_type(42))));

  pfp_t pfp2 (bind((pfun_t) stapl::find_if<view_type,pred>,
                   _1, stapl::bind2nd(fun(), data_type(-1))));
  sfp_t sfp2 (bind((sfun_t) std::find_if<iterator_type, pred>,
                   _1, _2, stapl::bind2nd(fun(), data_type(-1))));

  pfp_t pfp3 (bind((pfun_t) stapl::find_if<view_type,pred>,
                   _1, stapl::bind2nd(fun(), data_type(1))));
  sfp_t sfp3 (bind((sfun_t) std::find_if<iterator_type, pred>,
                   _1, _2, stapl::bind2nd(fun(), data_type(1))));

  const unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  // one match
  std::stringstream short_name;
  short_name << name << "-one_match";
  timed_test_result tres1(short_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres1, i, pfp1, sfp1);

  // no match
  timed_test_result tres2(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres2, i, pfp2, sfp2);

  // multiple matches
  std::stringstream multi_name;
  multi_name << name << "-multi_match";
  timed_test_result tres3(multi_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, block_sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0],
      block_sequence<data_type>(1, 10, 1)))
    (tres3, i, pfp3, sfp3);

  test_report(tres1);
  test_report(tres2);
  test_report(tres3);
}


template <typename T>
void test_find_if_not(const unsigned int n, const unsigned i,
                  traits_type const& t, char const* name,
                  std::string const& ds = "none")
{
  typedef view_type::reference                                    ret_type;
  typedef test_one_view<ret_type>                                 test_type;
  typedef std::not_equal_to<data_type>                            fun;
  typedef stapl::binder2nd<fun,data_type,false> const&            pred;

  typedef ret_type (*pfun_t)(view_type const&, pred);
  typedef iterator_type (*sfun_t)(iterator_type, iterator_type, pred);

  typedef function<ret_type      (view_type const&)>              pfp_t;
  typedef function<iterator_type (iterator_type, iterator_type)>  sfp_t;

  const unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  pfp_t pfp1 (bind((pfun_t) stapl::find_if_not<view_type, pred>,
                   _1, stapl::bind2nd(fun(), data_type(n_elems/2))));
  sfp_t sfp1 (bind((sfun_t) std::find_if_not<iterator_type, pred>,
                   _1, _2, stapl::bind2nd(fun(), data_type(n_elems/2))));

  pfp_t pfp2 (bind((pfun_t) stapl::find_if_not<view_type,pred>,
                   _1, stapl::bind2nd(fun(), data_type(-1))));
  sfp_t sfp2 (bind((sfun_t) std::find_if_not<iterator_type, pred>,
                   _1, _2, stapl::bind2nd(fun(), data_type(-1))));

  pfp_t pfp3 (bind((pfun_t) stapl::find_if_not<view_type,pred>,
                   _1, stapl::bind2nd(fun(), data_type(1))));
  sfp_t sfp3 (bind((sfun_t) std::find_if_not<iterator_type, pred>,
                   _1, _2, stapl::bind2nd(fun(), data_type(1))));

  // one match
  std::stringstream short_name;
  short_name << name << "-one_match";
  timed_test_result tres1(short_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres1, i, pfp1, sfp1);

  // no match
  timed_test_result tres2(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres2, i, pfp2, sfp2);

  // multiple matches
  std::stringstream multi_name;
  multi_name << name << "-multi_match";
  timed_test_result tres3(multi_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, block_sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0],
      block_sequence<data_type>(1, 10, 1)))
    (tres3, i, pfp3, sfp3);

  test_report(tres1);
  test_report(tres2);
  test_report(tres3);
}


template<typename T>
void test_is_partitioned(const unsigned int n, const unsigned int i,
                         traits_type const& t, char const* name,
                         std::string const& ds = "none")
{
  typedef bool ret_type;
  typedef test_one_view<ret_type> test_type;
  typedef std::less<data_type> fun;
  typedef stapl::binder2nd<fun,data_type,false> pred;

  typedef ret_type (*pfun_t)(view_type const&, pred);
  typedef ret_type (*sfun_t)(iterator_type, iterator_type, pred);

  typedef function<ret_type (view_type const&)>              pfp_t;
  typedef function<ret_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1 (bind((pfun_t) stapl::is_partitioned<view_type const&, pred>,
                   _1, stapl::bind2nd(fun(), data_type(42))));

  sfp_t sfp1 (bind((sfun_t) std::is_partitioned<iterator_type, pred>,
                   _1, _2, stapl::bind2nd(fun(), data_type(42))));

  unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  // one match
  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres1, i, pfp1, sfp1);

  test_report(tres1);
}


template <typename T>
void test_adjacent_find(const unsigned int n, const unsigned i,
           traits_type const& t, char const* name,
           std::string const& ds = "none")
{
  typedef view_type::reference        ret_type;
  typedef std::equal_to<data_type>   fun;
  typedef test_one_view<ret_type>    test_type;

  typedef ret_type (*pfun1_t)(view_type const&);
  typedef ret_type (*pfun2_t)(view_type const&, fun);
  typedef iterator_type (*sfun1_t)(iterator_type, iterator_type);
  typedef iterator_type (*sfun2_t)(iterator_type, iterator_type, fun);

  typedef function<ret_type      (view_type const&)>              pfp_t;
  typedef function<iterator_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1 (bind((pfun1_t) stapl::adjacent_find<view_type>, _1));
  pfp_t pfp2 (bind((pfun2_t) stapl::adjacent_find<view_type, fun>, _1, fun()));

  sfp_t sfp1 (bind((sfun1_t) std::adjacent_find<iterator_type>, _1, _2));
  sfp_t sfp2 (bind((sfun2_t) std::adjacent_find<iterator_type, fun>,
                   _1, _2, fun()));

  const unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  // no match
  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(0, 1)))
    (tres1, i, pfp1, sfp1);

  std::stringstream no_name;
  no_name << name << "-pred";
  timed_test_result tres2(no_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(0, 1)))
    (tres2, i, pfp2, sfp2);

  // match in beginning
  std::stringstream one_name;
  one_name << name << "-one_match";
  timed_test_result tres3(one_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0],
      sequence<data_type>(1,0)))
    (tres3, i, pfp1, sfp1);

  one_name << "_pred";
  timed_test_result tres4(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0],
      sequence<data_type>(1,0)))
    (tres4, i, pfp2, sfp2);

  // match in subrange
  std::stringstream sub_name;
  sub_name << name << "-sub_match";
  timed_test_result tres5(sub_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, wave_sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0],
      wave_sequence<data_type>(1, n_elems/2, 1,
        wave_sequence<data_type>::Both)))
    (tres5, i, pfp1, sfp1);

  sub_name << "_pred";
  timed_test_result tres6(sub_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, wave_sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0],
      wave_sequence<data_type>(1, n_elems/2, 1,
        wave_sequence<data_type>::Both)))
    (tres6, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
  test_report(tres3);
  test_report(tres4);
  test_report(tres5);
  test_report(tres6);
}


template <typename T>
void test_find_first_of(const unsigned int n, const unsigned i,
                        traits_type const& t, char const* name,
                        std::string const& ds = "none")
{
  typedef view_type::reference                 ret_type;
  typedef test_two_view<ret_type>             test_type;
  //typedef std::equal_to<data_type>            fun;
  //typedef stapl::binder2nd<fun,data_type,false> const&          pred;

  typedef ret_type (*pfun1_t)(view_type const&, view_type const&);
  //typedef ret_type (*pfun2_t)(view_type const&, view_type const&, pred);
  typedef iterator_type (*sfun1_t)(iterator_type, iterator_type, iterator_type,
                                   iterator_type);
  //typedef iterator_type (*sfun2_t)(iterator_type, iterator_type,
  //                                 iterator_type, iterator_type, pred);

  typedef function<ret_type (view_type const&, view_type const&)> pfp_t;
  typedef function<iterator_type (iterator_type, iterator_type, iterator_type,
                                  iterator_type)> sfp_t;

  // FIXME:  stapl::generate on a view of a container with fewer than
  // p elements does not work.
  bool use_seq_gen = generate_sequential_data();
  if (!use_seq_gen)
    enable_sequential_generation();

  pfp_t pfp1 (bind((pfun1_t) stapl::find_first_of<view_type, view_type>,
                   _1, _2));

  sfp_t sfp1 (bind((sfun1_t) std::find_first_of<iterator_type, iterator_type>,
                   _1, _2, _3, _4));

  /*pfp_t pfp2 (bind((pfun2_t) stapl::find_first_of<view_type, view_type, pred>,
                     _1, _2,fun()));

  sfp_t sfp2 (bind((sfun2_t) std::find_first_of<iterator_type, iterator_type,
                                                pred>,
                   _1, _2, _3, _4, fun()));
*/

  unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  std::stringstream one_name;
  one_name << name << "-one_match";
  timed_test_result tres1(one_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      1, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 1)))
    (tres1, i, pfp1, sfp1);

  timed_test_result tres2(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      10, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 1)))
    (tres2, i, pfp1, sfp1);

  if (!use_seq_gen)
    disable_sequential_generation();
/*
   // one match
   timed_test_result tres3(name);
   test_type(
     data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
       n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(0, 1)),
     data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
       1, t.ct_create[1], t.vw_create[1], sequence<data_type>(-1, 0)))
     (tres3, i, pfp1, sfp1);
*/
/*   timed_test_result tres4(name);
   test_type(
     data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
       n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(0, 1)),
     data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
       1, t.ct_create[1], t.vw_create[1], sequence<data_type>(-1, 0)))
     (tres4, i, pfp2, sfp2);*/
  // multiple match
/*   timed_test_result tres5(name);
   test_type(
     data_descriptor<pcontainer_type, view_type, block_sequence<data_type> >(
       n_elems, t.ct_create[0], t.vw_create[0],
       block_sequence<data_type>(1, 10, 1)),
     data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
       1, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 0)))
     (tres5, i, pfp1, sfp1);
*/
/*
   timed_test_result tres6(name);
   test_type(
     data_descriptor<pcontainer_type, view_type, block_sequence<data_type> >(
       n_elems, t.ct_create[0], t.vw_create[0],
       block_sequence<data_type>(1, 10, 1)),
     data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
       1, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 0)))
     (tres6, i, pfp2, sfp2);
*/
  test_report(tres1);
  test_report(tres2);
}


template <typename T>
void test_count(const unsigned int n, const unsigned i,
                traits_type const& t, char const* name,
                std::string const& ds = "none")
{
  typedef view_type::iterator::difference_type  ret_type;
  typedef test_one_view<ret_type>               test_type;

  typedef ret_type (*pfun_t)(view_type const&, data_type const&);
  typedef ret_type (*sfun_t)(iterator_type, iterator_type, data_type const&);

  function<ret_type (view_type const&)> pfp =
    bind((pfun_t) stapl::count<view_type, data_type>, _1, 4);

  function<ret_type (iterator_type, iterator_type)> sfp =
    bind((sfun_t) std::count<iterator_type, data_type>, _1 , _2, 4);

  const unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  timed_test_result tres(name);

  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres, i, pfp, sfp);

  test_report(tres);
}


template <typename T>
struct my_functor1
  : public ro_unary_function<T, bool>
{
  T val;

  my_functor1(T v)
    : val(v)
  { }

  void define_type(typer& t)
  {
    t.member(val);
  }

  template<typename Ref>
  bool operator() (Ref x) const
  {
    if (x <= val)
      return true;
    return false;
  }
};

template <typename T>
struct my_functor2
  : public ro_unary_function<T, bool>
{
  T val;

  my_functor2(T v)
    : val(v)
  { }

  //my_functor2(const my_functor2& wf, std::size_t offset)
  //  : val(wf.val + offset) {}

  void define_type(typer& t)
  {
    t.member(val);
  }

  template<typename Ref>
  bool operator() (Ref x)
  {
    return ++val == x;
  }
};

template <typename T>
void test_count_if(const unsigned int n, const unsigned i,
                   traits_type const& t, char const* name,
                   std::string const& ds = "none")
{
  typedef view_type::iterator::difference_type    ret_type;
  typedef test_one_view<ret_type>                 test_type;
  typedef my_functor1<data_type>                  mf1;
  typedef my_functor2<data_type>                  mf2;

  typedef ret_type (*pfun1_t)(view_type const&, mf1);
  typedef ret_type (*pfun2_t)(view_type const&, mf2);
  typedef ret_type (*sfun1_t)(iterator_type, iterator_type, mf1);
  typedef ret_type (*sfun2_t)(iterator_type, iterator_type, mf2);

  const int init = 10;

  function<ret_type (view_type const&)> pfp1 =
    bind((pfun1_t) stapl::count_if<view_type, mf1>, _1, mf1(init));

  function<ret_type (iterator_type, iterator_type)> sfp1 =
    bind((sfun1_t) std::count_if<iterator_type, mf1>, _1 , _2, mf1(init));

  function<ret_type (view_type const&)> pfp2 =
    bind((pfun2_t) stapl::count_if<view_type, mf2>, _1, mf2(init));

  function<ret_type (iterator_type, iterator_type)> sfp2 =
    bind((sfun2_t) std::count_if<iterator_type, mf2>, _1 , _2, mf2(init));

  const unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres1, i, pfp1, sfp1);

  std::stringstream no_name;
  no_name << name << "-no_match";
  timed_test_result tres2(no_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}


template <typename T>
void test_mismatch(const unsigned int n, const unsigned i,
                   traits_type const& t, char const* name,
                   std::string const& ds = "none")
{
  typedef std::pair<view_type::reference, view_type::reference>   ret_type;
  typedef std::pair<iterator_type, iterator_type>               seq_ret_type;
  typedef test_two_view<ret_type>   test_type;
  typedef std::equal_to<data_type>                              comp;

  typedef ret_type (*pfun1_t)(view_type const&, view_type const&);
  typedef ret_type (*pfun2_t)(view_type const&, view_type const&, comp);
  typedef seq_ret_type (*sfun1_t)(iterator_type, iterator_type,
                                  iterator_type);
  typedef seq_ret_type (*sfun2_t)(iterator_type, iterator_type,
                                  iterator_type, comp);

  typedef function<ret_type     (view_type const&, view_type const&)>  pfp_t;
  typedef function<seq_ret_type (iterator_type, iterator_type,
                                 iterator_type)>                       sfp_t;

  pfp_t pfp1(bind((pfun1_t) stapl::mismatch<view_type, view_type>, _1, _2));
  sfp_t sfp1(bind((sfun1_t) std::mismatch<iterator_type, iterator_type>,
                  _1, _2, _3));
  pfp_t pfp2(bind((pfun2_t) stapl::mismatch<view_type, view_type, comp>,
                  _1, _2, comp()));
  sfp_t sfp2(bind((sfun2_t) std::mismatch<iterator_type, iterator_type, comp>,
                  _1, _2, _3, comp()));

 const unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  // no mismatch
  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 1)))
    (tres1, i, pfp1, sfp1);

  std::stringstream pred_name;
  pred_name << name << "-pred";
  timed_test_result tres2(pred_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 1)))
    (tres2, i, pfp2, sfp2);

  // mismatch
  std::stringstream short_name;
  short_name << name << "-short";
  timed_test_result tres3(short_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 2)))
    (tres3, i, pfp1, sfp1);

  short_name << "_pred";
  timed_test_result tres4(short_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 2)))
    (tres4, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
  test_report(tres3);
  test_report(tres4);
}


template <typename T>
void test_equal(const unsigned int n, const unsigned i,
                traits_type const& t, char const* name,
                std::string const& ds = "none")
{
  typedef bool                       ret_type;
  typedef test_two_view<ret_type>    test_type;

  typedef ret_type (*pfun_t)(view_type const&, view_type const&);
  typedef ret_type (*sfun_t)(iterator_type, iterator_type, iterator_type);

  function<ret_type (view_type const&, view_type const&)>
    pfp(bind((pfun_t) stapl::equal<view_type, view_type>, _1, _2));

  function<ret_type (iterator_type, iterator_type, iterator_type)>
    sfp(bind((sfun_t) std::equal<iterator_type, iterator_type>, _1, _2, _3));

  const unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  //equal
  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 1)))
    (tres1, i, pfp, sfp);

  //not equal
  std::stringstream short_name;
  short_name << name << "-short";
  timed_test_result tres2(short_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 2)))
    (tres2, i, pfp, sfp);

  test_report(tres1);
  test_report(tres2);
}


template <typename T>
void test_search(const unsigned int n, const unsigned i,
                 traits_type const& t, char const* name,
                 std::string const& ds = "none")
{
  typedef view_type::reference      ret_type;
  typedef test_two_view<ret_type>  test_type;
  typedef std::equal_to<data_type> fun;

  typedef ret_type (*pfun1_t)(view_type const&, view_type const&);
  typedef ret_type (*pfun2_t)(view_type const&, view_type const&, fun);
  typedef iterator_type (*sfun1_t)(iterator_type, iterator_type,
                                   iterator_type, iterator_type);
  typedef iterator_type (*sfun2_t)(iterator_type, iterator_type,
                                   iterator_type, iterator_type, fun);

  function<ret_type (view_type const&, view_type const&)> pfp1
    (bind((pfun1_t) stapl::search<view_type, view_type>, _1, _2));

  function<iterator_type (iterator_type, iterator_type, iterator_type,
                          iterator_type)> sfp1
    (bind((sfun1_t) std::search<iterator_type, iterator_type>, _1, _2, _3, _4));

  function<ret_type (view_type const&, view_type const&)> pfp2
    (bind((pfun2_t) stapl::search<view_type, view_type, fun>, _1, _2, fun()));

  function<iterator_type (iterator_type, iterator_type, iterator_type,
                          iterator_type)> sfp2
    (bind((sfun2_t) std::search<iterator_type, iterator_type, fun>,
          _1, _2, _3, _4, fun()));

  const unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  // no match
  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      2, t.ct_create[1], t.vw_create[1], sequence<data_type>(0, 1)))
    (tres1, i, pfp1, sfp1);

  std::stringstream no_name;
  no_name << name << "-pred";
  timed_test_result tres2(no_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      2, t.ct_create[1], t.vw_create[1], sequence<data_type>(0, 1)))
    (tres2, i, pfp2, sfp2);

  // one match
  std::stringstream one_name;
  one_name << name << "-one";
  timed_test_result tres3(one_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      2, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 1)))
    (tres3, i, pfp1, sfp1);

  one_name << "_pred";
  timed_test_result tres4(one_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      2, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 1)))
    (tres4, i, pfp2, sfp2);

  // multiple match
  std::stringstream multi_name;
  multi_name << name << "-multi";
  timed_test_result tres5(multi_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type,
                    repetitive_sequence<data_type> >(
      2, t.ct_create[1], t.vw_create[1],
      repetitive_sequence<data_type>(1, 1, n_elems/get_num_locations())))
    (tres5, i, pfp1, sfp1);

  multi_name << "_pred";
  timed_test_result tres6(multi_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type,
                    repetitive_sequence<data_type> >(
      2, t.ct_create[1], t.vw_create[1],
      repetitive_sequence<data_type>(1, 1, n_elems/get_num_locations())))
    (tres6, i, pfp2, sfp2);

  // subview boundary match - balanced/blocked (not cyclic) partition
  std::stringstream bdry_name;
  bdry_name << name << "-bdry";
  timed_test_result tres7(bdry_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      5, t.ct_create[1], t.vw_create[1],
      sequence<data_type>(n_elems/get_num_locations() - 2, 1)))
    (tres7, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
  test_report(tres3);
  test_report(tres4);
  test_report(tres5);
  test_report(tres6);
  test_report(tres7);
}

template <typename T>
void test_search_n(const unsigned int n, const unsigned int i,
                   traits_type const& t, char const* name,
                   std::string const& ds = "none")
{
  typedef view_type::reference            ret_type;
  typedef test_one_view<ret_type>        test_type;
  typedef std::equal_to<data_type>       fun;

  typedef ret_type (*pfun1_t)(view_type const&, int, data_type);
  typedef ret_type (*pfun2_t)(view_type const&, int, data_type, fun);
  typedef iterator_type (*sfun1_t)(iterator_type, iterator_type, int,
                                   data_type const&);
  typedef iterator_type (*sfun2_t)(iterator_type, iterator_type, int,
                                   data_type const&, fun);

  typedef function<ret_type (view_type const&)>                      pfp_t;
  typedef function<iterator_type (iterator_type, iterator_type)>     sfp_t;

  const unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  const size_t elem = n_elems/2;
  pfp_t pfp1(bind((pfun1_t) stapl::search_n<view_type>, _1, 2, 0));
  pfp_t pfp3(bind((pfun1_t) stapl::search_n<view_type>, _1, 1, elem));
  pfp_t pfp5(bind((pfun1_t) stapl::search_n<view_type>, _1, 21, 7));

  pfp_t pfp2(bind((pfun2_t) stapl::search_n<view_type, fun>, _1, 2, 0, fun()));
  pfp_t pfp4(bind((pfun2_t) stapl::search_n<view_type, fun>,
                  _1, 1, data_type(elem), fun()));
  pfp_t pfp6(bind((pfun2_t) stapl::search_n<view_type, fun>,
                  _1, 21, 7, fun()));

  sfp_t sfp1(bind((sfun1_t) std::search_n<iterator_type, int, data_type>,
                  _1, _2, 2, 0));
  sfp_t sfp3(bind((sfun1_t) std::search_n<iterator_type, int, data_type>,
                  _1, _2, 1, elem));
  sfp_t sfp5(bind((sfun1_t) std::search_n<iterator_type, int, data_type>,
                  _1, _2, 21, 7));

  sfp_t sfp2(bind((sfun2_t) std::search_n<iterator_type, int, data_type, fun>,
                  _1, _2, 2, 0, fun()));
  sfp_t sfp4(bind((sfun2_t) std::search_n<iterator_type, int, data_type, fun>,
                  _1, _2, 1, elem, fun()));
  sfp_t sfp6(bind((sfun2_t) std::search_n<iterator_type, int, data_type, fun>,
                  _1, _2, 21, 7, fun()));

  // no match
  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres1, i, pfp1, sfp1);

  std::stringstream pred_name;
  pred_name << name << "-pred";
  timed_test_result tres2(pred_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres2, i, pfp2, sfp2);

  // match in subrange
  std::stringstream sub_name;
  sub_name << name << "-subrange_match";
  timed_test_result tres3(sub_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres3, i, pfp3, sfp3);

  sub_name << "_pred";
  timed_test_result tres4(sub_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres4, i, pfp4, sfp4);

  // match in subrange - the test in search_n.cc is more interesting
  std::stringstream bdry_name;
  bdry_name << name << "-bdry_match";
  timed_test_result tres5(bdry_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(7, 0)))
    (tres5, i, pfp5, sfp5);

  bdry_name << "_pred";
  timed_test_result tres6(bdry_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(7, 0)))
    (tres6, i, pfp6, sfp6);

  test_report(tres1);
  test_report(tres2);
  test_report(tres3);
  test_report(tres4);
  test_report(tres5);
  test_report(tres6);
}


template <typename T>
void test_find_end(const unsigned int n, const unsigned int i,
                   traits_type const& t, char const* name,
                   std::string const& ds = "none")
{
  typedef view_type::reference         ret_type;
  typedef test_two_view<ret_type>     test_type;
  typedef std::equal_to<data_type>    fun;

  typedef ret_type (*pfun1_t)(view_type const&, view_type const&);
  typedef ret_type (*pfun2_t)(view_type const&, view_type const&, fun);
  typedef iterator_type (*sfun1_t)(iterator_type, iterator_type,
                                   iterator_type, iterator_type);
  typedef iterator_type (*sfun2_t)(iterator_type, iterator_type,
                                   iterator_type, iterator_type, fun);

  function<ret_type (view_type const&, view_type const&)> pfp1
    (bind((pfun1_t) stapl::find_end<view_type, view_type>, _1, _2));

  function<iterator_type (iterator_type, iterator_type, iterator_type,
           iterator_type)> sfp1
    (bind((sfun1_t) std::find_end<iterator_type, iterator_type>, _1, _2, _3, _4));

  function<ret_type (view_type const&, view_type const&)> pfp2
    (bind((pfun2_t) stapl::find_end<view_type, view_type, fun>, _1, _2, fun()));

  function<iterator_type (iterator_type, iterator_type, iterator_type,
           iterator_type)> sfp2
    (bind((sfun2_t) std::find_end<iterator_type, iterator_type, fun>,
          _1, _2, _3, _4, fun()));

  const unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  // no match
  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      2, t.ct_create[1], t.vw_create[1], sequence<data_type>(0, 1)))
    (tres1, i, pfp1, sfp1);

  std::stringstream pred_name;
  pred_name << name << "-pred";
  timed_test_result tres2(pred_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      2, t.ct_create[1], t.vw_create[1], sequence<data_type>(0, 1)))
    (tres2, i, pfp2, sfp2);

  // one match
  std::stringstream one_name;
  one_name << name << "-one_match";
  timed_test_result tres3(one_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      2, t.ct_create[1], t.vw_create[1], sequence<data_type>(n_elems/2, 1)))
    (tres3, i, pfp1, sfp1);

  one_name << "_pred";
  timed_test_result tres4(one_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      2, t.ct_create[1], t.vw_create[1], sequence<data_type>(n_elems/2, 1)))
    (tres4, i, pfp2, sfp2);

  // multiple match
  std::stringstream multi_name;
  multi_name << name << "-multi_match";
  timed_test_result tres5(multi_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type,
                    repetitive_sequence<data_type> >(
      2, t.ct_create[1], t.vw_create[1],
      repetitive_sequence<data_type>(1, 1, n_elems/get_num_locations())))
    (tres5, i, pfp1, sfp1);

  multi_name << "_pred";
  timed_test_result tres6(multi_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type,
                    repetitive_sequence<data_type> >(
      2, t.ct_create[1], t.vw_create[1],
      repetitive_sequence<data_type>(1, 1, n_elems/get_num_locations())))
    (tres6, i, pfp2, sfp2);

  // subview boundary match - balanced/blocked (not cyclic) partition
  std::stringstream bdry_name;
  bdry_name << name << "-bdry_match";
  timed_test_result tres7(bdry_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      5, t.ct_create[1], t.vw_create[1],
      sequence<data_type>(n_elems/get_num_locations() - 2, 1)))
    (tres7, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
  test_report(tres3);
  test_report(tres4);
  test_report(tres5);
  test_report(tres6);
  test_report(tres7);
}


template <typename T>
void test_any_of(const unsigned int n, const unsigned i,
                 traits_type const& t, char const* name,
                 std::string const& ds = "none")
{
  typedef bool        ret_type;
  typedef test_one_view<ret_type>    test_type;
  typedef std::equal_to<data_type>   fun;
  typedef stapl::binder2nd<fun,data_type,false> const& pred;

  typedef ret_type (*pfun_t)(view_type const&, pred);
  typedef ret_type (*sfun_t)(iterator_type, iterator_type, pred);

  typedef function<ret_type      (view_type const&)>        pfp_t;
  typedef function<ret_type (iterator_type, iterator_type)> sfp_t;

  pfp_t pfp1 (bind((pfun_t) stapl::any_of<view_type, pred>,
                   _1, stapl::bind2nd(fun(), data_type(42))));
  sfp_t sfp1 (bind((sfun_t) std::any_of<iterator_type, pred>,
                   _1, _2, stapl::bind2nd(fun(), data_type(42))));

  pfp_t pfp2 (bind((pfun_t) stapl::any_of<view_type,pred>,
                   _1, stapl::bind2nd(fun(), data_type(-1))));
  sfp_t sfp2 (bind((sfun_t) std::any_of<iterator_type, pred>,
                   _1, _2, stapl::bind2nd(fun(), data_type(-1))));

  const unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  // returns true
  std::stringstream short_name;
  short_name << name << "-short";
  timed_test_result tres1(short_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres1, i, pfp1, sfp1);

  // returns false
  timed_test_result tres2(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}


template <typename T>
void test_all_of(const unsigned int n, const unsigned i,
                 traits_type const& t, char const* name,
                 std::string const& ds = "none")
{
  typedef bool        ret_type;
  typedef test_one_view<ret_type>    test_type;
  typedef std::equal_to<data_type>   fun;
  typedef stapl::binder2nd<fun,data_type,false> const&        pred;

  typedef ret_type (*pfun_t)(view_type const&, pred);
  typedef ret_type (*sfun_t)(iterator_type, iterator_type, pred);

  typedef function<ret_type      (view_type const&)>              pfp_t;
  typedef function<ret_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1 (bind((pfun_t) stapl::all_of<view_type, pred>,
                   _1, stapl::bind2nd(fun(), data_type(42))));
  sfp_t sfp1 (bind((sfun_t) std::all_of<iterator_type, pred>,
                   _1, _2, stapl::bind2nd(fun(), data_type(42))));

  pfp_t pfp2 (bind((pfun_t) stapl::all_of<view_type,pred>,
                   _1, stapl::bind2nd(fun(), data_type(1))));
  sfp_t sfp2 (bind((sfun_t) std::all_of<iterator_type, pred>,
                   _1, _2, stapl::bind2nd(fun(), data_type(1))));

  const unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  // returns true
  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(42, 0)))
    (tres1, i, pfp1, sfp1);

  // returns false
  std::stringstream short_name;
  short_name << name << "-short";
  timed_test_result tres2(short_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}


template <typename T>
void test_none_of(const unsigned int n, const unsigned i,
                  traits_type const& t, char const* name,
                  std::string const& ds = "none")
{
  typedef bool        ret_type;
  typedef test_one_view<ret_type>    test_type;
  typedef std::equal_to<data_type>   fun;
  typedef stapl::binder2nd<fun,data_type,false>        pred;

  typedef ret_type (*pfun_t)(view_type const&, pred);
  typedef ret_type (*sfun_t)(iterator_type, iterator_type, pred);

  typedef function<ret_type      (view_type const&)>              pfp_t;
  typedef function<ret_type (iterator_type, iterator_type)>  sfp_t;

  pfp_t pfp1 (bind((pfun_t) stapl::none_of<view_type, pred>,
                   _1, stapl::bind2nd(fun(), data_type(42))));
  sfp_t sfp1 (bind((sfun_t) std::none_of<iterator_type, pred>,
                   _1, _2, stapl::bind2nd(fun(), data_type(42))));

  pfp_t pfp2 (bind((pfun_t) stapl::none_of<view_type,pred>,
                   _1, stapl::bind2nd(fun(), data_type(-1))));
  sfp_t sfp2 (bind((sfun_t) std::none_of<iterator_type, pred>,
                   _1, _2, stapl::bind2nd(fun(), data_type(-1))));


  const unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  // returns false
  std::stringstream short_name;
  short_name << name << "-short";
  timed_test_result tres1(short_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres1, i, pfp1, sfp1);

  // returns true
  timed_test_result tres2(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}


template <typename T>
void test_is_permutation(const unsigned int n, const unsigned i,
                         traits_type const& t, char const* name,
                         std::string const& ds = "none")
{
  typedef bool                       ret_type;
  typedef test_two_view<ret_type>    test_type;

  typedef ret_type (*pfun_t)(view_type const&, view_type const&);
  typedef ret_type (*sfun_t)(iterator_type, iterator_type, iterator_type);

  function<ret_type (view_type const&, view_type const&)>
    pfp(bind((pfun_t) stapl::is_permutation<view_type, view_type>, _1, _2));

  function<ret_type (iterator_type, iterator_type, iterator_type)>
    sfp(bind((sfun_t) std::is_permutation<iterator_type, iterator_type>,
             _1, _2, _3));

  unsigned long int n_elems = set_data_size(n, ds, "nonmutating");

  // Reduce the data size so the runtime is on the appropriate scale.
  if ((ds != "tiny") && (ds != "none"))
  {
    if (ds == "small")
      n_elems /= 1000;
  }

  //is permutation
  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1],
      sequence<data_type>(n_elems, -1)))
    (tres1, i, pfp, sfp);

  //is not permutation
  std::stringstream short_name;
  short_name << name << "-short";
  timed_test_result tres2(short_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 2)))
    (tres2, i, pfp, sfp);

  test_report(tres1);
  test_report(tres2);
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef void (*test_func_type)(const unsigned int, const unsigned int,
                                 traits_type const&, char const*,
                                 std::string const&);
  test_pair<test_func_type> tests[] =
  {
    {"all_of",          test_all_of<int>},
    {"any_of",          test_any_of<int>},
    {"none_of",         test_none_of<int>},
    {"find",            test_find<int>},
    {"find_if",         test_find_if<int>},
    {"find_if_not",     test_find_if_not<int>},
    {"is_partitioned",  test_is_partitioned<int>},
    {"adjacent_find",   test_adjacent_find<int>},
    {"find_first_of",   test_find_first_of<int>},
    {"count",           test_count<int>},
    {"count_if",        test_count_if<int>},
    {"mismatch",        test_mismatch<int>},
    {"equal",           test_equal<int>},
    {"search",          test_search<int>},
    {"search_n",        test_search_n<int>},
    {"find_end",        test_find_end<int>},
    {"is_permutation",  test_is_permutation<int>}
  };

  test_execute(argc, argv, tests,
               tests+(sizeof (tests)/sizeof (test_pair<test_func_type>)));

  return EXIT_SUCCESS;
}
