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
#include <boost/function.hpp>
#include <boost/bind.hpp>

using boost::bind;
using boost::function;

using std::generate;
using std::replace;
using std::replace_if;
using std::fill;
using std::transform;
using std::swap_ranges;
using std::remove_if;
using std::remove;
using std::remove_copy;
using std::remove_copy_if;
using std::reverse;
using std::reverse_copy;
using std::unique;
using std::unique_copy;
using std::copy;
using std::copy_backward;


template<typename U>
struct seq_sum_op
{
  U val;

  seq_sum_op(U v)
    : val(std::move(v))
  { }

  template<typename T>
  void operator()(T& elem) const
  { elem = elem + val; }
};


template<typename U>
struct sum_op
{
  U m_val;

  sum_op(U v)
    : m_val(std::move(v))
  { }

  void define_type(stapl::typer& t)
  { t.member(m_val); }

  template<typename Reference>
  void operator()(Reference elem) const
  { elem = elem + m_val; }
};


template <typename T>
void test_for_each(const unsigned int n, const unsigned i,
                   traits_type const& t, char const* name,
                   std::string const& ds = "none")
{
  const unsigned long int n_elems = set_data_size(n, ds, "mutating");
  const data_type val             = 10;

  timed_test_result tres(name);

  test_one_view<void>(
    make_data_descriptor(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(3, 1)))
    (tres, i,
     [val](view_type const& v0)
       { return stapl::for_each(v0, sum_op<data_type>(val)); },
     [val](iterator_type b, iterator_type e)
       { return std::for_each(b, e, seq_sum_op<data_type>(val)); });

  test_report(tres);
}


// stapl::copy returns void, whereas std::copy returns iterator
template<typename T>
void test_copy(const unsigned int n, const unsigned int i,
               traits_type const& t, char const* name,
               std::string const& ds = "none")
{
  const unsigned long int n_elems = set_data_size(n, ds, "mutating");

  boost::function<
    void (iterator_type, iterator_type, iterator_type)> sfp(
     [](iterator_type b1, iterator_type e1, iterator_type b2)
       { std::copy(b1, e1, b2); });

  timed_test_result tres(name);

  test_two_view<void>(
    make_data_descriptor(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    make_data_descriptor(
      n_elems, t.ct_create[1], t.vw_create[1], null_sequence<data_type>()))
    (tres, i,
     [](view_type const& v0, view_type const& v1) { stapl::copy(v0, v1); },
     std::move(sfp));

  test_report(tres);
}


template<typename T>
void test_copy_n(const unsigned int n, const unsigned int i,
                 traits_type const& t, char const* name,
                 std::string const& ds = "none")
{
  using size_type = const unsigned long int;

  size_type n_elems = set_data_size(n, ds, "mutating");

  boost::function<
    void (iterator_type, iterator_type)> sfp(
     [n_elems](iterator_type b1, iterator_type b2)
       { std::copy_n(b1, n_elems, b2); });

  timed_test_result tres(name);

  test_two_view<void>(
    make_data_descriptor(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    make_data_descriptor(
      n_elems, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 1)))
    (tres, i,
     [n_elems](view_type const& v0, view_type const& v1)
       { stapl::copy_n(v0, v1, n_elems); },
     std::move(sfp));

  test_report(tres);
}


// there is no copy_if in ISO C++
template<typename T>
void test_copy_if(const unsigned int n,
                  const unsigned int i,
                  traits_type const& t,
                  char const* name,
                  std::string const& ds = "none")
{
  using p_fun = stapl::equal_to<data_type>;
  using fun   = std::equal_to<data_type>;

  const unsigned long int n_elems = set_data_size(n, ds, "mutating");
  const data_type init            = n_elems/2;

  boost::function<
    iterator_type (iterator_type, iterator_type, iterator_type)> sfp(
    [init](iterator_type b_iter0, iterator_type e_iter0, iterator_type b_iter1)
       {
         return std::copy_if(b_iter0, e_iter0,
                            b_iter1,  stapl::bind2nd(fun(), init));
       });

  timed_test_result tres(name);

  test_two_view<view_type>(
    make_data_descriptor(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(0, 1)),
    make_data_descriptor(
      n_elems, t.ct_create[1], t.vw_create[1], null_sequence<data_type>())
  )(tres, i,
    [init](view_type const& v0, view_type& v1)
       { return stapl::copy_if(v0, v1, stapl::bind2nd(p_fun(), init)); },
    std::move(sfp),
    [init](view_type& vw0, view_type& vw1,
            iterator_type b_iter1, iterator_type e_iter1,
            iterator_type b_iter2, iterator_type e_iter2)
       {
         const int sz    = vw0.size();
         const bool b1   = sz == std::distance(b_iter1, e_iter1);
         const bool b2   = std::equal(b_iter1, e_iter1, vw0.begin());
         iterator_type s = std::partition_point(b_iter2, b_iter2 + vw0.size(),
                                                stapl::bind2nd(fun(), init));
         const bool b3   = std::equal(b_iter2, s, vw1.begin());

         return b1&& b2 && b3;
       });

  test_report(tres);
}


//template<typename Iterator, typename Size>
//void copy_backward_wrapper(Iterator begin, Iterator end, Iterator con,
//                           Size size)
//{
//  std::advance(con, size);
//  std::copy_backward(begin, end, con);
//}


//template<typename T>
//void test_copy_backward(const unsigned int n, const unsigned int i,
//                        traits_type const& t, char const* name,
//                        std::string const& ds = "none")
//{
//  typedef void                      ret_type;
//  typedef test_two_view<ret_type>   test_type;

//  typedef ret_type (*pfun_t)(view_type const&, view_type const&);
//  function<ret_type (view_type const&, view_type const&)>
//    pfp(bind((pfun_t)copy_backward<view_type, view_type>, _1, _2));

// const unsigned long int n_elems = set_data_size(n, ds, "mutating");

//  typedef ret_type (*sfun_t)(iterator_type, iterator_type, iterator_type,
//                             size_t);
//  function<ret_type (iterator_type, iterator_type, iterator_type)>
//    sfp(bind((sfun_t)copy_backward_wrapper<iterator_type, size_t>,
//             _1, _2, _3, n_elems));

//  timed_test_result tres(name);
//  test_type(
//    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
//      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
//    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
//      n_elems, t.ct_create[1], t.vw_create[1], null_sequence<data_type>()))
//    (tres, i, pfp, sfp);

//  test_report(tres);
//}


//std::swap_ranges returns an iterator, whereas stapl::swap_ranges returns void
//////////////////////////////////////////////////////////////////////
/// @brief Test function for the swap ranges algorithm.
/// @param n The number of elements to generate.
/// @param i The number of times the test will be run.
/// @param t Traits class containing the types of the containers and views to be
///   used and the function objects to instantiate them.
//////////////////////////////////////////////////////////////////////
template<typename T>
void test_swap_ranges(const unsigned int n, const unsigned int i,
                      traits_type const& t, char const* name,
                      std::string const& ds = "none")
{
  const unsigned long int n_elems = set_data_size(n, ds, "mutating");

  function<void (iterator_type, iterator_type, iterator_type)> sfp(
    [](iterator_type b1, iterator_type e1, iterator_type b2)
      { std::swap_ranges(b1, e1, b2); });

  timed_test_result tres(name);

  test_two_view<void>(
    make_data_descriptor(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 2)),
    make_data_descriptor(
      n_elems, t.ct_create[1], t.vw_create[1], sequence<data_type>(2, 2)))
    (tres, i,
     [](view_type& v0, view_type& v1)
       { stapl::swap_ranges(v0, v1); },
     std::move(sfp));

  test_report(tres);
}


template<typename T>
struct increment
{
  T operator()(T val) const
  { return ++val; }
};


template<typename T>
void test_transform(const unsigned int n, const unsigned int i,
                    traits_type const& t, char const* name,
                    std::string const& ds = "none")
{
  typedef void                      ret_type;
  typedef test_two_view<ret_type>   test_type1;
  typedef test_three_view<ret_type> test_type2;
  typedef increment<data_type>      unary_fun;
  typedef std::plus<data_type>      binary_fun;

  typedef iterator_type (*sfun1_t)(iterator_type, iterator_type, iterator_type,
                                   unary_fun);
  function< void (iterator_type, iterator_type, iterator_type)>
    sfp1(bind((sfun1_t)std::transform<iterator_type, iterator_type, unary_fun>,
              _1, _2, _3, unary_fun()));

  typedef ret_type (*pfun1_t)(view_type const&, view_type const&, unary_fun);
  function<ret_type (view_type const&, view_type const&)>
    pfp1(bind((pfun1_t)stapl::transform<view_type, view_type, unary_fun>,
              _1, _2, unary_fun()));

  const unsigned long int n_elems = set_data_size(n, ds, "mutating");

  timed_test_result tres1(name);
  test_type1(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 1)))
    (tres1, i, pfp1, sfp1);

  typedef ret_type (*pfun2_t)(view_type&, view_type&, view_type&, binary_fun);
  function<ret_type (view_type &, view_type &, view_type &)>
    pfp2(bind((pfun2_t)stapl::transform<view_type, view_type, view_type,
                                        binary_fun>,
              _1, _2, _3, binary_fun()));

  typedef iterator_type (*sfun2_t)(iterator_type, iterator_type, iterator_type,
                                   iterator_type, binary_fun);
  function< iterator_type (iterator_type, iterator_type, iterator_type,
                           iterator_type)>
    sfp2(bind((sfun2_t)std::transform<iterator_type, iterator_type,
                                      iterator_type, binary_fun>,
              _1, _2, _3, _4, binary_fun()));

  std::stringstream fun_name;
  fun_name << name << "-op";
  timed_test_result tres2(fun_name.str().c_str());
  test_type2(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
      n_elems, t.ct_create[2], t.vw_create[2], null_sequence<data_type>()))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}


template<typename T>
void test_replace(const unsigned int n, const unsigned int i,
                  traits_type const& t, char const* name,
                  std::string const& ds = "none")
{
  typedef void                    ret_type;
  typedef test_one_view<ret_type> test_type;
  typedef view_type::value_type data_type;

  const data_type init1 = 10;
  const data_type init2 = 99;

  typedef void (*pfun_t)(view_type&, data_type const&, data_type const&);
  function<void (view_type&)>
    pfp(bind((pfun_t)stapl::replace<view_type>, _1, init1, init2));

  typedef void (*sfun_t)(iterator_type, iterator_type, data_type const&,
                         data_type const&);
  function<void (iterator_type, iterator_type)>
    sfp(bind((sfun_t)std::replace<iterator_type, data_type>,
             _1, _2, init1, init2));

  const unsigned long int n_elems = set_data_size(n, ds, "mutating");

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(0, 1)))
    (tres, i, pfp, sfp);

  test_report(tres);
}

template<typename T>
void test_replace_copy(const unsigned int n, const unsigned int i,
                       traits_type const& t, char const* name,
                       std::string const& ds = "none")
{
  typedef void                    ret_type;
  typedef test_two_view<ret_type> test_type;

  const data_type int1 = 10;
  const data_type int2 = 99;

  typedef view_type::iterator (*pfun_t)(view_type&, view_type&, data_type,
                                        data_type);

  function<ret_type (view_type&, view_type&)>
    pfp( bind((pfun_t) stapl::replace_copy<view_type, view_type>,
                       _1, _2, int1, int2));

  typedef iterator_type (*sfun_t)(iterator_type, iterator_type, iterator_type,
                                  data_type const&, data_type const&);

  function<ret_type (iterator_type, iterator_type, iterator_type)>
    sfp( bind((sfun_t) std::replace_copy<iterator_type, iterator_type,
                                         data_type>,
                                         _1, _2, _3, int1, int2));

  const unsigned long int n_elems = set_data_size(n, ds, "mutating");

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(0, 1)),
    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], null_sequence<data_type>()))
    (tres, i, pfp, sfp);

  test_report(tres);
}

template<typename T>
void test_replace_if(const unsigned int n, const unsigned int i,
                     traits_type const& t, char const* name,
                     std::string const& ds = "none")
{
  typedef void                         ret_type;
  typedef test_one_view<ret_type>      test_type;
  typedef view_type::value_type data_type;

  typedef stapl::less<data_type>       p_fun;
  typedef stapl::binder2nd<p_fun,data_type,false>      p_pred;

  typedef std::less<data_type>         fun;
  typedef stapl::binder2nd<fun,data_type,false>          pred;

  const data_type init1 = 10;
  const data_type init2 = 99;

  typedef ret_type (*pfun_t)(view_type &, p_pred, data_type const&);
  function<ret_type (view_type &)>
    pfp(bind((pfun_t) replace_if<view_type, p_pred>,
             _1, stapl::bind2nd(p_fun(), init1), init2));

  typedef ret_type (*sfun_t)(iterator_type, iterator_type, pred,
                             data_type const&);
  function<ret_type (iterator_type, iterator_type)>
    sfp(bind((sfun_t) std::replace_if<iterator_type, pred, data_type>,
             _1, _2, stapl::bind2nd(fun(), init1), init2));

  const unsigned long int n_elems = set_data_size(n, ds, "mutating");

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(0, 1)))
    (tres, i, pfp, sfp);

  test_report(tres);
}

template<typename T>
void test_replace_copy_if(const unsigned int n, const unsigned int i,
                          traits_type const& t, char const* name,
                          std::string const& ds = "none")
{
  typedef void                          ret_type;
  typedef test_two_view<ret_type>       test_type;

  typedef stapl::less<data_type>        p_fun;
  typedef stapl::binder2nd<p_fun,data_type,false>      p_pred;

  typedef std::less<data_type>          fun;
  typedef stapl::binder2nd<fun,data_type,false>        pred;

  const data_type init1 = 10;
  const data_type init2 = 99;

  typedef view_type::iterator (*pfun_t)(view_type&, view_type&, p_pred,
                                        const data_type);

  function<ret_type (view_type&, view_type&)>
    pfp(bind((pfun_t) stapl::replace_copy_if<view_type, view_type, p_pred>,
             _1, _2, stapl::bind2nd(p_fun(), init1), init2));

  typedef iterator_type (*sfun_t)(iterator_type, iterator_type, iterator_type,
                                  pred, data_type const&);

  function<ret_type (iterator_type, iterator_type, iterator_type)>
    sfp(bind((sfun_t) std::replace_copy_if<iterator_type, iterator_type, pred,
                                     data_type>,
             _1, _2, _3, stapl::bind2nd(fun(),init1), init2));

  const unsigned long int n_elems = set_data_size(n, ds, "mutating");

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(0, 1)),
    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], null_sequence<data_type>()))
    (tres, i, pfp, sfp);

  test_report(tres);
}



template<typename T>
void test_fill(const unsigned int n, const unsigned int i,
               traits_type const& t, char const* name,
               std::string const& ds = "none")
{
  typedef void                       ret_type;
  typedef test_one_view<ret_type>    test_type;

  const data_type init = 42;

  typedef ret_type (*pfun_t)(view_type const&, data_type);
  function<ret_type (view_type&)> pfp =
    bind((pfun_t)stapl::fill<view_type>, _1, init);

  typedef ret_type (*sfun_t)(iterator_type, iterator_type, data_type const&);
  function<ret_type (iterator_type, iterator_type)>
    sfp(bind((sfun_t)std::fill<iterator_type, data_type>, _1, _2, init));

  const unsigned long int n_elems = set_data_size(n, ds, "mutating");

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], null_sequence<data_type>()))
    (tres, i, pfp, sfp);

  test_report(tres);
}


// The wrapper is needed to supply the consistent interface with the two
// iterators to the test functions.
template<typename Iterator, typename Size, typename T>
void fill_n_wrapper(Iterator i1, Iterator, Size s, const T& v)
{ std::fill_n(i1, s, v); }


template<typename T>
void test_fill_n(const unsigned int n, const unsigned int i,
                 traits_type const& t, char const* name,
                 std::string const& ds = "none")
{
  typedef void                     ret_type;
  typedef test_one_view<ret_type>  test_type;

  const unsigned long int n_elems = set_data_size(n, ds, "mutating");

  const data_type init = 42;
  const size_t size = n_elems/2;

  typedef ret_type (*pfun_t)(view_type&, data_type, size_t);
  function<ret_type (view_type&)>
    pfp = bind((pfun_t)stapl::fill_n<view_type>, _1, init, size);

  typedef ret_type (*sfun_t)(iterator_type, iterator_type, size_t,
                             data_type const&);
  function<ret_type (iterator_type, iterator_type)>
    sfp(bind((sfun_t)fill_n_wrapper<iterator_type, size_t, data_type>,
             _1, _2, size, init));

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], null_sequence<data_type>()))
    (tres, i, pfp, sfp);

  test_report(tres);
}


template<typename T>
void test_generate(const unsigned int n, const unsigned int i,
                   traits_type const& t, char const* name,
                   std::string const& ds = "none")
{
  typedef void                         ret_type;
  typedef test_one_view<ret_type>      test_type;

  typedef ret_type (*sfun_t)(iterator_type, iterator_type, sequence<data_type>);
  typedef ret_type (*pfun_t)(view_type const&, sequence<data_type>);

  function<ret_type (iterator_type, iterator_type)>
    sfp(bind((sfun_t) std::generate<iterator_type, sequence<data_type> >,
             _1, _2, sequence<data_type>(0, 1)));

  function<ret_type (view_type const&)>
    pfp = bind((pfun_t) stapl::generate<view_type, sequence<data_type> >,
               _1, sequence<data_type>(0, 1));

  const unsigned long int n_elems = set_data_size(n, ds, "mutating");

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], null_sequence<data_type>()))
    (tres, i, pfp, sfp);

  test_report(tres);
}


// The wrapper is needed to supply the consistent interface with the
// two iterators to the test functions.
template<typename Iterator, typename Size, typename Generator>
void generate_n_wrapper(Iterator i1, Iterator, Size count, Generator g)
{ std::generate_n(i1, count, g); }


// This should be higher in the test dependency tree (if any)
// std::generate_n doesn't exist and this test is not the best one
// (see that it always fills the container)
template<typename T>
void test_generate_n(const unsigned int n, const unsigned int i,
                     traits_type const& t, char const* name,
                     std::string const& ds = "none")
{
  typedef void                                    ret_type;
  typedef test_one_view<ret_type>                 test_type;

  typedef ret_type (*sfun_t)(iterator_type, iterator_type, size_t,
                             sequence<data_type>);
  typedef ret_type (*pfun_t)(view_type const&, size_t, size_t,
                             sequence<data_type>);

  unsigned long int n_elems = set_data_size(n, ds, "mutating");

  //FIXME: Resolve memory consumtion issues.
  // Reduce the data size so the runtime is on the appropriate scale.
  if ((ds != "tiny") && (ds != "none"))
  {
    n_elems /= 1000;
  }

  const size_t size = n_elems/2;

  function<ret_type (view_type const&)>
    pfp(bind((pfun_t) stapl::generate_n<view_type, sequence<data_type> >,
             _1, 0, size, sequence<data_type>(0, 1)));

  function<ret_type (iterator_type, iterator_type)>
    sfp(bind((sfun_t) generate_n_wrapper<iterator_type, size_t,
                                         sequence<data_type> >,
             _1, _2, size, sequence<data_type>(0, 1)));

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], null_sequence<data_type>()))
    (tres, i, pfp, sfp);

  test_report(tres);
}


template<typename T>
void test_remove(const unsigned int n, const unsigned int i,
                 traits_type const& t, char const* name,
                 std::string const& ds = "none")
{
  using ret_type = view_type;
  using pfun_t   = ret_type (*)(view_type&, data_type);

  const data_type init            = 3;
  const unsigned long int n_elems = set_data_size(n, ds, "mutating");

  function<ret_type (view_type &)>
    pfp(bind((pfun_t)stapl::remove<view_type>, _1, init));

  using sfun_t =
    iterator_type (*)(iterator_type, iterator_type, data_type const&);

  function<iterator_type (iterator_type, iterator_type)>
    sfp(bind((sfun_t)remove<iterator_type, data_type>, _1, _2, init));

  timed_test_result tres(name);

  test_one_view<ret_type>(
    make_data_descriptor(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(0, 1)))
    (tres, i, pfp, sfp);

  test_report(tres);
}


template<typename T>
void test_keep_if(const unsigned int n, const unsigned int i,
                    traits_type const& t, char const* name,
                    std::string const& ds = "none")
{
  typedef view_type         ret_type;
  typedef test_one_view<ret_type>     test_type;

  typedef std::not_equal_to<data_type>    fun;
  typedef stapl::binder2nd<fun,data_type,false>    pred;

  typedef stapl::equal_to<data_type>  p_fun;
  typedef stapl::binder2nd<p_fun,data_type,false>  p_pred;

  unsigned long int n_elems = set_data_size(n, ds, "mutating");

  typedef view_type::value_type data_type;
  data_type init = n_elems / 2;

  typedef ret_type (*pfun_t)(view_type const&, p_pred);
  function<ret_type (view_type const&)>
    pfp(bind((pfun_t)stapl::keep_if<view_type, p_pred>,
             _1, stapl::bind2nd(p_fun(), init)));

  typedef iterator_type (*sfun_t)(iterator_type, iterator_type, pred);
  function<iterator_type (iterator_type, iterator_type)>
    sfp(bind((sfun_t)remove_if<iterator_type, pred>,
             _1, _2, stapl::bind2nd(fun(), init)));

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(0, 1)))
    (tres, i, pfp, sfp);

  test_report(tres);
}

template<typename T>
void test_remove_if(const unsigned int n, const unsigned int i,
                    traits_type const& t, char const* name,
                    std::string const& ds = "none")
{
  typedef view_type         ret_type;
  typedef test_one_view<ret_type>     test_type;

  typedef std::equal_to<data_type>    fun;
  typedef stapl::binder2nd<fun,data_type,false>    pred;

  typedef stapl::equal_to<data_type>  p_fun;
  typedef stapl::binder2nd<p_fun,data_type,false>  p_pred;

  unsigned long int n_elems = set_data_size(n, ds, "mutating");

  typedef view_type::value_type data_type;
  data_type init = n_elems / 2;

  typedef ret_type (*pfun_t)(view_type const&, p_pred);
  function<ret_type (view_type const&)>
    pfp(bind((pfun_t)stapl::remove_if<view_type, p_pred>,
             _1, stapl::bind2nd(p_fun(), init)));

  typedef iterator_type (*sfun_t)(iterator_type, iterator_type, pred);
  function<iterator_type (iterator_type, iterator_type)>
    sfp(bind((sfun_t)remove_if<iterator_type, pred>,
             _1, _2, stapl::bind2nd(fun(), init)));

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(0, 1)))
    (tres, i, pfp, sfp);

  test_report(tres);
}


template<typename T>
void test_remove_copy(const unsigned int n, const unsigned int i,
                      traits_type const& t, char const* name,
                      std::string const& ds = "none")
{
  using ret_type  = view_type;
  using data_type = view_type::value_type;

  const unsigned long int n_elems = set_data_size(n, ds, "mutating");
  const data_type init            = n_elems / 2;

  boost::function<
    iterator_type (iterator_type, iterator_type, iterator_type)> sfp(
    [init](iterator_type b_iter0, iterator_type e_iter0, iterator_type b_iter1)
       { return std::remove_copy(b_iter0, e_iter0, b_iter1, init); });

  boost::function<
    bool (view_type&, view_type&,
          iterator_type, iterator_type, iterator_type, iterator_type)
  > validator(
     [init](view_type& vw0, view_type& vw1,
            iterator_type b_iter1, iterator_type e_iter1,
            iterator_type b_iter2, iterator_type e_iter2)
       {
         const int sz    = vw0.size();
         const bool b1   = sz == std::distance(b_iter1, e_iter1);
         const bool b2   = std::equal(b_iter1, e_iter1, vw0.begin());
         // Rerun sequential to get middle iterator.  Currently, framework
         // doesn't pass the return value to the validator.
         iterator_type s = std::remove_copy(b_iter1, e_iter1, b_iter2, init);
         const bool b3   = std::equal(b_iter2, s, vw1.begin());

         return b1 && b2 && b3;
       });

  timed_test_result tres(name);

  test_two_view<ret_type>(
    make_data_descriptor(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(0, 1)),
    make_data_descriptor(
      n_elems, t.ct_create[1], t.vw_create[1], null_sequence<data_type>()))
    (tres, i,
     [init](view_type& v0, view_type& v1)
       { return stapl::remove_copy(v0, v1, init); },
    std::move(sfp),
    std::move(validator));

  test_report(tres);
}


template<typename T>
void test_remove_copy_if(const unsigned int n, const unsigned int i,
                         traits_type const& t, char const* name,
                         std::string const& ds = "none")
{
  using ret_type  = view_type;
  using data_type = view_type::value_type;

  const unsigned long int n_elems = set_data_size(n, ds, "mutating");
  const data_type init            = n_elems / 2;

  function<ret_type (view_type&, view_type&)> pfp(
    [init](view_type& v0, view_type& v1)
    {
      return stapl::remove_copy_if(
        v0, v1, stapl::bind2nd(stapl::equal_to<data_type>(), init));
    });

  function<iterator_type (iterator_type, iterator_type, iterator_type)> sfp(
    [init](iterator_type b_iter0, iterator_type e_iter0, iterator_type b_iter1)
    {
      return std::remove_copy_if(
        b_iter0, e_iter0, b_iter1,
        stapl::bind2nd(std::equal_to<data_type>(), init)
      );
    });

  timed_test_result tres(name);

  test_two_view<ret_type>(
    make_data_descriptor(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(0, 1)),
    make_data_descriptor(
      n_elems, t.ct_create[1], t.vw_create[1], null_sequence<data_type>()))
    (tres, i, pfp, sfp);

  test_report(tres);
}


template<typename T>
void test_unique(const unsigned int n, const unsigned int i,
                 traits_type const& t, char const* name,
                 std::string const& ds = "none")
{
  typedef void         ret_type;
  typedef test_one_view<ret_type>     test_type;
  typedef stapl::equal_to<data_type>    fun;

  typedef std::pair<view_type,view_type> (*pfun1_t)(view_type);
  function<ret_type (view_type)>
    pfp1(bind((pfun1_t)stapl::unique<view_type>, _1) );

  typedef std::pair<view_type,view_type> (*pfun2_t)(view_type, fun);
  function<ret_type (view_type)>
    pfp2(bind((pfun2_t)stapl::unique<view_type, fun>, _1, fun()));

  typedef iterator_type (*sfun1_t)(iterator_type, iterator_type);
  function<iterator_type (iterator_type, iterator_type)>
    sfp1(bind((sfun1_t)std::unique<iterator_type>, _1, _2));

  typedef iterator_type (*sfun2_t)(iterator_type, iterator_type, fun);
  function<iterator_type (iterator_type, iterator_type)>
    sfp2(bind((sfun2_t)std::unique<iterator_type, fun>, _1, _2, fun()));

  unsigned long int n_elems = set_data_size(n, ds, "mutating");

  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type,
                    repetitive_sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0],
      stapl::repetitive_sequence<data_type>(1, 1, 10)))
    (tres1, i, pfp1, sfp1);

  std::stringstream fun_name;
  fun_name << name << "-op";
  timed_test_result tres2(fun_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres2, i, pfp2, sfp2);

  test_report(tres1);
  test_report(tres2);
}


template<typename T>
void test_unique_copy(const unsigned int n, const unsigned int i,
                      traits_type const& t, char const* name,
                      std::string const& ds = "none")
{
  typedef void                        ret_type;
  typedef test_two_view<ret_type>     test_type;
  typedef stapl::equal_to<data_type>  fun;

  typedef std::pair<view_type,view_type> (*pfun1_t)(view_type, view_type);
  function<ret_type (view_type, view_type)>
    pfp1(bind((pfun1_t)stapl::unique_copy<view_type, view_type>, _1, _2));

  typedef std::pair<view_type,view_type>
    (*pfun2_t)(view_type&, view_type&, fun);
  function<ret_type (view_type&, view_type&)>
    pfp2(bind((pfun2_t)stapl::unique_copy<view_type, view_type, fun>,
               _1, _2, fun()));

  typedef iterator_type (*sfun1_t)(iterator_type, iterator_type, iterator_type);
  function<void (iterator_type, iterator_type, iterator_type)>
    sfp1(bind((sfun1_t)std::unique_copy<iterator_type, iterator_type>,
              _1, _2, _3));

  typedef iterator_type (*sfun2_t)(iterator_type, iterator_type, iterator_type,
                                   fun);
  function<void (iterator_type, iterator_type, iterator_type)>
    sfp2(bind((sfun2_t)std::unique_copy<iterator_type, iterator_type, fun>,
              _1, _2, _3, fun()));

  unsigned long int n_elems = set_data_size(n, ds, "mutating");

  // Reduce the data size so the runtime is on the appropriate scale.
  if ((ds != "tiny") && (ds != "none"))
    n_elems /= 100;

  timed_test_result tres1(name);
  test_type(
    data_descriptor<pcontainer_type, view_type,
                    repetitive_sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0],
      stapl::repetitive_sequence<data_type>(1, 1, 10)),
    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], null_sequence<data_type>()))
    (tres1, i, pfp1, sfp1);

  std::stringstream fun_name;
  fun_name << name << "-nodupes";
  timed_test_result tres2(fun_name.str().c_str());
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], null_sequence<data_type>()))
    (tres2, i, pfp1, sfp1);

  test_report(tres1);
  test_report(tres2);
}


template<typename T>
void test_reverse(const unsigned int n, const unsigned int i,
                  traits_type const& t, char const* name,
                  std::string const& ds = "none")
{
  typedef void                        ret_type;
  typedef test_one_view<ret_type>     test_type;

  typedef ret_type (*pfun_t)(view_type const&);
  function<ret_type (view_type&)>
    pfp(bind((pfun_t)stapl::reverse<view_type>, _1));

  typedef ret_type (*sfun_t)(iterator_type, iterator_type);
  function<ret_type (iterator_type, iterator_type)>
    sfp(bind((sfun_t)reverse<iterator_type>, _1, _2));

  const unsigned long int n_elems = set_data_size(n, ds, "mutating");

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres, i, pfp, sfp);

  test_report(tres);
}


template<typename T>
void test_reverse_copy(const unsigned int n, const unsigned int i,
                       traits_type const& t, char const* name,
                       std::string const& ds = "none")
{
  typedef void                       ret_type;
  typedef test_two_view<ret_type>    test_type;

  typedef ret_type (*pfun_t)(view_type const&, view_type const&);
  function<ret_type (view_type const&, view_type const&)>
    pfp(bind((pfun_t)stapl::reverse_copy<view_type, view_type>, _1, _2));

  typedef iterator_type (*sfun_t)(iterator_type, iterator_type, iterator_type);
  function<ret_type (iterator_type, iterator_type, iterator_type)>
    sfp(bind((sfun_t)reverse_copy<iterator_type, iterator_type>, _1, _2, _3));

  const unsigned long int n_elems = set_data_size(n, ds, "mutating");

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, null_sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], null_sequence<data_type>()))
    (tres, i, pfp, sfp);

 test_report(tres);
}


template<typename View>
void rotate_wrapper(View const& view, unsigned int d)
{
  rotate(view, d);
}


template<typename Iterator>
void rotate_wrapper(Iterator first, Iterator last, unsigned int d)
{
  Iterator it = first;
  std::advance(it, d);
  std::rotate(first, it, last);
}


template<typename T>
void test_rotate(const unsigned int n, const unsigned int i,
                 traits_type const& t, char const* name,
                 std::string const& ds = "none")
{
  typedef void                        ret_type;
  typedef test_one_view<ret_type>     test_type;

  unsigned long int n_elems = set_data_size(n, ds, "mutating");

  typedef ret_type (*pfun_t)(view_type const&, unsigned int);
  function<ret_type (view_type const&)>
    pfp(bind((pfun_t)rotate_wrapper<view_type>, _1, n_elems/2));

  typedef ret_type (*sfun_t)(iterator_type, iterator_type, unsigned int);
  function<ret_type (iterator_type, iterator_type)>
    sfp(bind((sfun_t)rotate_wrapper<iterator_type>, _1, _2, n_elems/2));

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)))
    (tres, i, pfp, sfp);

  test_report(tres);
}


template<typename View0, typename View1>
void rotate_copy_wrapper(View0 const& view0, View1 const& view1, unsigned int d)
{
  rotate_copy(view0, view1, d);
}


template<typename Iterator>
void rotate_copy_wrapper(Iterator first, Iterator last, Iterator output,
                         unsigned int d)
{
  Iterator it = first;
  std::advance(it, d);
  std::rotate_copy(first, it, last, output);
}


template<typename T>
void test_rotate_copy(const unsigned int n, const unsigned int i,
                      traits_type const& t, char const* name,
                      std::string const& ds = "none")
{
  typedef void                       ret_type;
  typedef test_two_view<ret_type>    test_type;

  unsigned long int n_elems = set_data_size(n, ds, "mutating");

  typedef ret_type (*pfun_t)(view_type const&, view_type const&, unsigned int);
  function<ret_type (view_type const&, view_type const&)>
    pfp(bind((pfun_t)rotate_copy_wrapper<view_type, view_type>,
             _1, _2, n_elems/2));

  typedef ret_type (*sfun_t)(iterator_type, iterator_type, iterator_type,
                             unsigned int);
  function<ret_type (iterator_type, iterator_type, iterator_type)>
    sfp(bind((sfun_t)rotate_copy_wrapper<iterator_type>,
             _1, _2, _3, n_elems/2));

  timed_test_result tres(name);
  test_type(
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[0], t.vw_create[0], sequence<data_type>(1, 1)),
    data_descriptor<pcontainer_type, view_type, sequence<data_type> >(
      n_elems, t.ct_create[1], t.vw_create[1], sequence<data_type>(0, 0)))
    (tres, i, pfp, sfp);

  test_report(tres);
}


// template<typename T>
// void test_random_shuffle(const unsigned int n, const unsigned int i,
//                          traits_type const& t, char const* name,
//                          std::string const& ds = "none")
// {} // this is random - cannot be checked with the ordinary way


// template<typename T>
// void test_random_sample(const unsigned int, const unsigned int,
//                         traits_type const& t, char const* name,
//                         std::string const& ds = "none"
// {} // this is random - cannot be checked with the ordinary way


// template<typename T>
// void test_random_sample_n(const unsigned int, const unsigned int,
//                           traits_type const& t, char const* name,
//                           std::string const& ds = "none"
// {} // this is random - cannot be checked with the ordinary way


template<typename T>
void test_partition(const unsigned int n,
                    const unsigned int i,
                    traits_type const& t,
                    char const* name,
                    std::string const& ds = "none")
{
  const unsigned long int n_elems = set_data_size(n, ds, "mutating");
  const data_type init            = n_elems/2;

  boost::function<iterator_type (iterator_type, iterator_type)> sfp(
    [init](iterator_type b, iterator_type e)
      {
        return std::partition(
          b, e, stapl::bind2nd(std::less<data_type>(), init));
      });

  timed_test_result tres(name);

  test_one_view<size_t, 2>(
    make_data_descriptor(
      n_elems, t.ct_create[0], t.vw_create[0], stapl::random_sequence(n_elems)))
  (tres, i,
   [init](view_type const& v0)
     {
       return stapl::partition(
         v0, stapl::bind2nd(stapl::less<data_type>(), init));
     },
   std::move(sfp),
   [init](view_type& vw, size_t p,
          iterator_type b, iterator_type e, iterator_type s)
     {
       const bool b1 = (long long int)p == std::distance(b, s);

       const bool b2 = stapl::is_partitioned(
         vw, stapl::bind2nd(stapl::less<data_type>(), init));

       const bool b3 =
         std::is_partitioned(
           b, e, stapl::bind2nd(std::less<data_type>(), init));

       return b1 && b2 && b3;
     });

  test_report(tres);
}


template<typename T>
void test_stable_partition(const unsigned int n, const unsigned int i,
                           traits_type const& t, char const* name,
                           std::string const& ds = "none")
{
  const unsigned long int n_elems = set_data_size(n, ds, "mutating");
  const data_type init            = n_elems / 2;

  timed_test_result tres(name);

  test_one_view<void>(
    make_data_descriptor(
      n_elems, t.ct_create[0], t.vw_create[0], stapl::random_sequence(n_elems)))
    (tres, i,
     [init](view_type const& v0)
       {
         stapl::stable_partition(
           v0, stapl::bind2nd(stapl::less<data_type>(), init));
       },
     [init](iterator_type b, iterator_type e)
       {
         std::stable_partition(
           b, e, stapl::bind2nd(std::less<data_type>(), init));
       });

  test_report(tres);
}


// template<typename T>
// void test_npartition(const unsigned int n, const unsigned int i,
//                      traits_type const& t, char const* name,
//                      std::string const& ds = "none")
// {}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using test_func_type =
    void (*)(const unsigned int, const unsigned int,
             traits_type const&, char const*, std::string const&);

  test_pair<test_func_type> tests[] =
  {
    {"for_each",            test_for_each<int>},
    {"copy",                test_copy<int>},
    {"copy_n",              test_copy_n<int>},
    {"copy_if",             test_copy_if<int>},
//     {"copy_backward",       test_copy_backward<int>},
//     {"swap",                test_swap<int>},
//     {"iter_swap",           test_iter_swap<int>},
    {"swap_ranges",         test_swap_ranges<int>},
    {"transform",           test_transform<int>},
    {"replace",             test_replace<int>},
    {"replace_if",          test_replace_if<int>},
    {"replace_copy",        test_replace_copy<int>},
    {"replace_copy_if",     test_replace_copy_if<int>},
    {"fill",                test_fill<int>},
    {"fill_n",              test_fill_n<int>},
    {"generate",            test_generate<int>},
    {"generate_n",          test_generate_n<int>},
    {"keep_if",             test_keep_if<int>},
    {"remove",              test_remove<int>},
    {"remove_if",           test_remove_if<int>},
    {"remove_copy",         test_remove_copy<int>},
    {"remove_copy_if",      test_remove_copy_if<int>},
    {"unique",              test_unique<int>},
    {"unique_copy",         test_unique_copy<int>},
    {"reverse",             test_reverse<int>},
    {"reverse_copy",        test_reverse_copy<int>},
    {"rotate",              test_rotate<int>},
    {"rotate_copy",         test_rotate_copy<int>},
// #ifndef TEST_P_LIST
//     {"random_shuffle",      test_random_shuffle<int>},
//     {"random_sample",       test_random_sample<int>},
//     {"random_sample_n",     test_random_sample_n<int>},
// #endif
    {"partition",           test_partition<int>},
    {"stable_partition",    test_stable_partition<int>}
  };

  test_execute(argc, argv, tests,
               tests+(sizeof (tests)/sizeof (test_pair<test_func_type>)));

  return EXIT_SUCCESS;
}
