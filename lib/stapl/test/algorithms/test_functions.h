/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef TEST_FUNCTIONS_H
#define TEST_FUNCTIONS_H

#include <stapl/utility/do_once.hpp>
#include <stapl/runtime.hpp>
#include <algorithm>
#include <functional>
#include <iterator>
#include <vector>
#include <utility>
#include <limits>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

// containers
#if defined(TEST_VECTOR)
# include <stapl/vector.hpp>
#elif defined(TEST_ARRAY)
# include <stapl/array.hpp>
#elif defined(TEST_LIST)
# include <stapl/list.hpp>
#else
# include <stapl/array.hpp>
#endif

#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>

#include "../confint.hpp"

using namespace stapl;

#include "test_result.h"
#include "test_report.h"
#include "test_utils.h"

// data type
#if   defined(INT)
  typedef std::size_t                data_type;
#elif defined(DOUBLE)
  typedef double                     data_type;
#else
   typedef std::size_t               data_type;
#endif

// stapl::container and STL container
#if defined(TEST_VECTOR)

using dist_spec          = distribution_spec<>;
using pcontainer_type    = vector<data_type,
                             view_based_partition<dist_spec>,
                             view_based_mapper<dist_spec> >;
using view_type          = vector_view<pcontainer_type>;
using stl_container_type = std::vector<data_type>;
using iterator_type      = stl_container_type::iterator;

#elif defined(TEST_ARRAY)

using dist_spec          = distribution_spec<>;
using pcontainer_type    = array<data_type,
                             view_based_partition<distribution_spec>,
                             view_based_mapper<distribution_spec> >;
using view_type          = array_view<pcontainer_type>;
using stl_container_type = std::vector<data_type>;
using iterator_type      = stl_container_type::iterator;

#elif defined(TEST_LIST)

using dist_spec          = distribution_spec<>;
using pcontainer_type    = list<data_type,
                             view_based_partition<dist_spec>,
                             view_based_mapper<dist_spec> >;
using view_type          = list_view<pcontainer_type>;
typedef std::list<data_type>         stl_container_type;
typedef stl_container_type::iterator iterator_type;

#else

using dist_spec          = distribution_spec<>;
using pcontainer_type    = array<data_type,
                             view_based_partition<dist_spec>,
                             view_based_mapper<dist_spec> >;
using view_type          = array_view<pcontainer_type>;
using stl_container_type = std::vector<data_type>;
using iterator_type      = stl_container_type::iterator;

#endif

template<typename Container, typename View, typename Generator>
struct data_descriptor
{
  using cct_type   = boost::function<Container* (int)>;
  using cvt_type   = boost::function<View (Container&)>;

  int       size;
  cct_type  cct;
  cvt_type  cvt;
  Generator gf;

  data_descriptor(const int sz,
                  cct_type cont_spec,
                  cvt_type view_spec,
                  Generator gen)
    : size(sz),
      cct(std::move(cont_spec)),
      cvt(std::move(view_spec)),
      gf(std::move(gen))
  { }
};


template<typename Container, typename View, typename Generator>
data_descriptor<Container, View, Generator>
make_data_descriptor(int sz,
                     boost::function<Container* (int)> const&  cont_spec,
                     boost::function<View (Container&)> const& view_spec,
                     Generator&& gen)
{
  return data_descriptor<Container, View, Generator>(
    sz, cont_spec, view_spec, gen);
}


template<typename Data, typename Container>
struct test_traits
{
  typedef Data                           data_type;
  typedef Container                      container_type;
#if defined(TEST_VECTOR)
  typedef vector_view<pcontainer_type>   view_type;
#elif defined(TEST_LIST)
  typedef list_view<pcontainer_type>     view_type;
#else
  typedef array_view<pcontainer_type>    view_type;
#endif
  typedef boost::function<container_type* (int)>       ct_t;
  typedef boost::function<view_type (container_type&)> vw_t;

  std::vector<ct_t>         ct_create;
  std::vector<vw_t>         vw_create;

  test_traits(std::vector<ct_t> const& cont_create,
              std::vector<vw_t> const& view_create)
    : ct_create(cont_create),vw_create(view_create)
  {}
};

typedef test_traits<data_type, pcontainer_type> traits_type;

#if defined(TEST_VECTOR)

inline pcontainer_type* allocate_balanced_container(const int n)
{ return new pcontainer_type(stapl::balance(n)); }

inline pcontainer_type* allocate_blocked_container(const int n,
                                                   const int block_size)
{
  return new pcontainer_type(stapl::block(n, block_size));
}

inline pcontainer_type* allocate_block_cyclic_container(const int n,
                                                   const int block_size)
{
  return new pcontainer_type(stapl::block_cyclic(n, block_size));
}

inline view_type allocate_aligned_view(pcontainer_type& ct)
{ return view_type(ct); }

#elif defined(TEST_LIST)

inline pcontainer_type* allocate_balanced_container(const int n)
{ return new pcontainer_type(stapl::balance(n)); }

inline pcontainer_type* allocate_blocked_container(const int n,
                                                   const int block_size)
{ return new pcontainer_type(stapl::block(n, block_size)); }

inline pcontainer_type* allocate_block_cyclic_container(const int n,
                                                        const int block_size)
{
  return new pcontainer_type(stapl::block_cyclic(n, block_size));
}

inline view_type allocate_aligned_view(pcontainer_type& ct)
{ return view_type(ct); }

#else //parray by default

inline pcontainer_type* allocate_balanced_container(const int n)
{
  return new pcontainer_type(stapl::balance(n));
}

inline pcontainer_type* allocate_blocked_container(const int n,
                                                   const int block_size)
{
  return new pcontainer_type(stapl::block(n, block_size));
}

inline pcontainer_type* allocate_block_cyclic_container(const int n,
                                                        const int block_size)
{
  return new pcontainer_type(stapl::block_cyclic(n, block_size));
}

inline view_type allocate_aligned_view(pcontainer_type& ct)
{ return view_type(ct);}

#endif

template<typename T1, typename T2>
boost::function<T1 ()> make_no_param_functor(T2& t2)
{
  return boost::function<T1 ()>(t2);
}

bool container_check = true;
bool sequential_generation = true;
bool sequential_execution = true;
bool parallel_report = true;
bool sequential_report = true;

inline void test_disable_checking(void)
{ container_check = false;       }
inline void test_enable_checking(void)
{ container_check = true;        }
inline bool test_check_containers(void)
{ return container_check;        }

inline void enable_sequential_generation(void)
{ sequential_generation = true;  }
inline void disable_sequential_generation(void)
{ sequential_generation = false; }
inline bool generate_sequential_data(void)
{ return sequential_generation;  }

inline void enable_sequential_execution(void)
{ sequential_execution = false;  }
inline void disable_sequential_execution(void)
{ sequential_execution = false;  }
inline bool sequential_data_needed(void)
{ return sequential_execution;   }

inline void disable_sequential_report(void)
{ sequential_report = false; }
inline void enable_sequential_report(void)
{ sequential_report = true; }
inline bool report_sequential(void)
{ return sequential_report; }

inline void disable_parallel_report(void)
{ parallel_report = false; }
inline void enable_parallel_report(void)
{ parallel_report = true; }
inline bool report_parallel(void)
{ return parallel_report; }


template <typename View, typename Generator>
struct sequential_generate
{
  void operator()(View& view, Generator g)
  { std::generate(view.begin(), view.end(), g); }
};

template <typename View1, typename View2>
struct sequential_copy
{
  void operator()(View1& view, View2& view2)
  {
    std::copy(view.begin(), view.end(), view2.begin());
  }
};

template<typename View1, typename View2, typename Generator>
void generate_data(View1& v, View2& u, Generator g)
{
  stapl::do_once(
    boost::bind<void>(sequential_generate<View1, Generator>(),
                      boost::ref(v), g));
  if (sequential_data_needed())
    stapl::do_once(
      boost::bind<void>(sequential_copy<View1, View2>(),
                        boost::ref(v), boost::ref(u)));
}

template<typename View1, typename View2, typename Generator>
void generate_parallel_data(View1& v, View2& u, Generator g)
{
  stapl::generate(v, g);
  if (sequential_data_needed())
    stapl::do_once(
      boost::bind<void>(sequential_copy<View1, View2>(),
                        boost::ref(v), boost::ref(u)));
}


inline unsigned long int
set_data_size(const unsigned long int n, std::string const& ds,
              std::string const& binary)
{
  if (ds == "none")
    return n;
  else if (ds == "tiny")
    return 1000;

  if ((binary == "numeric") || (binary == "nonmutating"))
  {
    if (ds == "small")
      return 1000000000ul; // one billion
#if __GNUC__
#  if __x86_64__ || __ppc64__
    else if (ds == "medium")
      return 5000000000ul; // five billion
    else if (ds == "large")
      return 25000000000ul; // twenty-five billion
#  else
    else if ((ds == "medium") || (ds == "large"))
    {
      fprintf(stderr,
              "Error: problem size too large to fit in unsigned long int.\n");
      std::exit(1);
    }
#  endif
#else
    else if (ds == "medium")
    {
      fprintf(stderr,
              "Warning: problem size may overflow unsigned long int.\n");
      return 10000000000ul; // ten billion
    }
    else if (ds == "large")
    {
      fprintf(stderr,
              "Warning: problem size may overflow unsigned long int.\n");
      return 20000000000ul; // twenty billion
    }
#endif
    else
    {
      std::exit(EXIT_FAILURE);
      return 0; // silence warning
    }
  }
  else if (binary == "mutating")
  {
    if (ds == "small")
      return 1000000000ul; // one billion
#if __GNUC__
#  if __x86_64__ || __ppc64__
    else if (ds == "medium")
      return 10000000000ul; // ten billion
    else if (ds == "large")
      return 20000000000ul; // twenty billion
#  else
    else if ((ds == "medium") || (ds == "large"))
    {
      fprintf(stderr,
              "Error: problem size too large to fit in unsigned long int.\n");
      std::exit(1);
    }
#  endif
#else
    else if (ds == "medium")
    {
      fprintf(stderr,
              "Warning: problem size may overflow unsigned long int.\n");
      return 10000000000ul; // ten billion
    }
    else if (ds == "large")
    {
      fprintf(stderr,
              "Warning: problem size may overflow unsigned long int.\n");
      return 20000000000ul; // twenty billion
    }
#endif
    else
    {
      std::exit(EXIT_FAILURE);
      return 0; // silence warning
    }
  }
  else if (binary == "sorting")
  {
    if (ds == "small")
      return 50000000ul; // fifty million
    else if (ds == "medium")
      return 200000000ul; // two hundred million
    else if (ds == "large")
      return 1000000000ul; // one billion
    else
    {
      std::exit(EXIT_FAILURE);
      return 0; // silence warning
    }
  }
  else
  {
    std::exit(EXIT_FAILURE);
    return 0; // silence warning
  }
}


inline bool
set_result(timed_test_result& result, double st, double pt, bool res)
{
  result.set_time_sequential(st);
  result.set_time_parallel(pt);
  result.set_result(res, sequential_data_needed());
  return res;
}

struct iterator_end_tag   {};
struct iterator_start_tag {};
struct iterator_alt_tag   {};

template <typename T>
inline double timed_do_once(T functor, const int I = 1)
{
  stapl::rmi_fence();

  double time = 0.0;
  if (stapl::get_location_id() == 0)
  {
    confidence_interval_controller control(I,I);
    stapl::counter<stapl::default_timer> c;
    for (int i=0; i<I; ++i)
    {
      c.reset();
      c.start();
      functor();
      control.push_back(c.stop());
    }
    time = control.stats().avg;
  }
  stapl::rmi_fence();

  return time;
}

namespace test_bases {

using boost::bind;

struct shared_test_base
{
  boost::ptr_vector<pcontainer_type>     c0;
  boost::ptr_vector<view_type>           v0;
  boost::ptr_vector<stl_container_type>  v1;

  template <typename Generator>
  shared_test_base(tuple<
                     data_descriptor<pcontainer_type, view_type, Generator>
                   > const& d)
  {
    c0.push_back(get<0>(d).cct(get<0>(d).size));
    v0.push_back(new view_type(get<0>(d).cvt(c0[0])));
    v1.push_back(new stl_container_type());

    if (generate_sequential_data()) {
      if (stapl::get_location_id() == 0)
        (v1[0]).resize(get<0>(d).size);
      generate_data(v0[0], v1[0], get<0>(d).gf);
    } else {
      generate_parallel_data(v0[0], v1[0], get<0>(d).gf);
    }
    stapl::rmi_fence();
  }

  template <typename Generator0, typename Generator1>
  shared_test_base(tuple<
                     data_descriptor<pcontainer_type, view_type, Generator0>,
                     data_descriptor<pcontainer_type, view_type, Generator1>
                   > const& d)
  {
    c0.push_back(get<0>(d).cct(get<0>(d).size));
    v0.push_back(new view_type(get<0>(d).cvt(c0[0])));
    v1.push_back(new stl_container_type());

    c0.push_back(get<1>(d).cct(get<1>(d).size));
    v0.push_back(new view_type(get<1>(d).cvt(c0[1])));
    v1.push_back(new stl_container_type());

    if (generate_sequential_data()) {
      if (stapl::get_location_id() == 0) {
        (v1[0]).resize(get<0>(d).size);
        (v1[1]).resize(get<1>(d).size);
      }
      generate_data(v0[0], v1[0], get<0>(d).gf);
      generate_data(v0[1], v1[1], get<1>(d).gf);
    } else {
      generate_parallel_data(v0[0], v1[0], get<0>(d).gf);
      generate_parallel_data(v0[1], v1[1], get<1>(d).gf);
    }
    stapl::rmi_fence();
  }

  template <typename Generator0, typename Generator1, typename Generator2>
  shared_test_base(tuple<
                     data_descriptor<pcontainer_type, view_type, Generator0>,
                     data_descriptor<pcontainer_type, view_type, Generator1>,
                     data_descriptor<pcontainer_type, view_type, Generator2>
                   > const& d)
  {
    c0.push_back(get<0>(d).cct(get<0>(d).size));
    v0.push_back(new view_type(get<0>(d).cvt(c0[0])));
    v1.push_back(new stl_container_type());

    c0.push_back(get<1>(d).cct(get<1>(d).size));
    v0.push_back(new view_type(get<1>(d).cvt(c0[1])));
    v1.push_back(new stl_container_type());

    c0.push_back(get<2>(d).cct(get<2>(d).size));
    v0.push_back(new view_type(get<2>(d).cvt(c0[2])));
    v1.push_back(new stl_container_type());

    if (generate_sequential_data()) {
      if (stapl::get_location_id() == 0) {
        (v1[0]).resize(get<0>(d).size);
        (v1[1]).resize(get<1>(d).size);
        (v1[2]).resize(get<2>(d).size);
      }
      generate_data(v0[0], v1[0], get<0>(d).gf);
      generate_data(v0[1], v1[1], get<1>(d).gf);
      generate_data(v0[2], v1[2], get<2>(d).gf);
    } else {
      generate_parallel_data(v0[0], v1[0], get<0>(d).gf);
      generate_parallel_data(v0[1], v1[1], get<1>(d).gf);
      generate_parallel_data(v0[2], v1[2], get<2>(d).gf);
    }
    stapl::rmi_fence();
  }

  struct chk_container
  {
    void operator()(bool& res,
                    view_type::iterator v0_beg, view_type::iterator v0_end,
                    iterator_type v1_beg) const
    {
      // std::cout << "Dumping Containers\n";
      // for ( ; v0_beg != v0_end; ++v0_beg, ++v1_beg)
      // {
      //   std::cout << *v0_beg << " // " << *v1_beg;
      //   if (*v0_beg != *v1_beg)
      //     std::cout << " <=====";
      //   std::cout << std::endl;
      // }
      // res = false;
      res = std::equal(v0_beg, v0_end, v1_beg);
    }
  };

  bool check_container(view_type::iterator v0_beg, view_type::iterator v0_end,
                       iterator_type v1_beg)
  {
    if (test_check_containers()) {
      bool res = false;
      stapl::do_once(bind<void>(chk_container(), boost::ref(res),
                                v0_beg, v0_end, v1_beg));
      return res;
    }
    return false;
  }

  bool check_distance(view_type::reference ref, iterator_type it)
  {
// STAPL list references don't define operator-
#if !defined(TEST_LIST)
    if (test_check_containers())
    {
      // STAPL::null_reference is equivalent to std::x.end(), but it not
      // the same value. Thus, we need to manually detect this condition.
      if (stapl::is_null_reference(ref) && it == v1[0].end() )
        return true;
      else
        return std::distance(v1[0].begin(),it) ==
              (signed)stapl::index_of(ref) - (signed)v0[0].begin().index();
    }
    return false;
#else
    return true;
#endif
  }

  bool compare_containers()
  {
    if (test_check_containers())
    {
      //fixme - it's written this way to avoid boolean short-circuit
      //(bad for concurrent operation :( )
      if (v0.size() == 3)
      {
        bool b1 = check_container(v0[0].begin(), v0[0].end(), v1[0].begin());
        bool b2 = check_container(v0[1].begin(), v0[1].end(), v1[1].begin());
        bool b3 = check_container(v0[2].begin(), v0[2].end(), v1[2].begin());
        return b1 && b2 && b3;
      }

      if (v0.size() == 2)
      {
        bool b1 = check_container(v0[0].begin(), v0[0].end(), v1[0].begin());
        bool b2 = check_container(v0[1].begin(), v0[1].end(), v1[1].begin());
        return b1 && b2;
      }
      return check_container(this->v0[0].begin(), this->v0[0].end(),
                             this->v1[0].begin());
    }
    return false;
  }


  //fixme nest in check_return?
  template<typename SRet, typename PRet>
  struct chk_ret
  {
    void operator()(bool& res, PRet& pv, SRet& sv) const
    { res = std::equal_to<SRet>()(pv,sv); }
  };

  template<typename SRet, typename PRet>
  bool check_return(SRet& sv, PRet& pv) const
  {
    if (test_check_containers()) {
      bool r_eq = false;
      stapl::do_once(bind<void>(chk_ret<SRet,PRet>(), boost::ref(r_eq),
                                boost::ref(pv), boost::ref(sv)));
      return r_eq;
    }
    return false;
  }

  bool check_return(iterator_type it, view_type::reference ref) const
  {
    if (test_check_containers()) {
      if (stapl::is_null_reference(ref))
      {
        if (it==v1[0].end())
          return true;
        return false;
      }
      if (it==v1[0].end())
        return false;
      return *it == ref;
    }
    return false;
  }
};

struct test_one_view_base  : public shared_test_base
{
  typedef shared_test_base Base;

  template <typename Generator>
  test_one_view_base(
    data_descriptor<pcontainer_type, view_type, Generator> const& d)
    : Base(tuple<
             data_descriptor<pcontainer_type, view_type, Generator>
           >(d))
  { }

  template<typename Func,typename Ret>
  double run_parallel_test(Func& palg, Ret& pv, const int I)
  {
    if (report_parallel())
    {
      confidence_interval_controller control(I,I);
      stapl::counter<stapl::default_timer> t;
      for (int i=0; i<I; ++i)
      {
        t.reset();
        t.start();
        pv = palg(this->v0[0]);
        control.push_back(t.stop());
      }
      return control.stats().avg;
    }
    return 0.0;
  }

  template<typename Func>
  double run_parallel_test(Func& palg, view_type** pv, const int I)
  {
    if (report_parallel())
    {
      confidence_interval_controller control(I,I);
      stapl::counter<stapl::default_timer> t;
      for (int i=0; i<I; ++i)
      {
        if (*pv != nullptr)
          delete *pv;
        t.reset();
        t.start();
        *pv = new view_type(palg(this->v0[0]));
        control.push_back(t.stop());
      }
      return control.stats().avg;
    }
    return 0.0;
  }

  template<typename Func, typename Ret>
  Ret run_parallel_test(Func& palg, const int I, double& time)
  {
    time = 0.0;
    if (report_parallel())
    {
      confidence_interval_controller control(I,I);
      stapl::counter<stapl::default_timer> t;
      t.start();
      Ret ref = palg(this->v0[0]);
      control.push_back(t.stop());
      for (int i=0; i<I-1; ++i)
      {
        t.reset();
        t.start();
        ref = palg(this->v0[0]);
        control.push_back(t.stop());
      }
      time = control.stats().avg;
      return ref;
    }
    return *this->v0[0].end();
  }

  template<typename Func>
  double run_parallel_test(Func& palg, const int I)
  {
    if (report_parallel())
    {
      confidence_interval_controller control(I,I);
      stapl::counter<stapl::default_timer> t;
      for (int i=0; i<I; ++i)
      {
        t.reset();
        t.start();
        palg(this->v0[0]);
        control.push_back(t.stop());
      }
      return control.stats().avg;
    }
    return 0.0;
  }

  template<typename Ret>
  struct sequential_test
  {
    void operator()(Ret& sv,
                    boost::function<Ret (iterator_type, iterator_type)>& salg,
                    stl_container_type& v1) const
    { sv = salg(v1.begin(), v1.end()); }
  };

  struct sequential_test_void
  {
    void operator()(boost::function<void (iterator_type, iterator_type)>& salg,
                    stl_container_type& v1) const
    { salg(v1.begin(), v1.end()); }
  };

  template<typename Ret>
  double run_sequential_test(
           boost::function<Ret (iterator_type, iterator_type)>& salg,
           Ret& sv, const int I)
  {
    if (sequential_data_needed())
      return timed_do_once(bind<void>(sequential_test<Ret>(), boost::ref(sv),
                           boost::ref(salg), boost::ref(this->v1[0])), I);
    return 0.0;
  }

  double run_sequential_test(
           boost::function<void (iterator_type, iterator_type)>& salg,
           const int I)
  {
    if (sequential_data_needed())
      return timed_do_once(bind<void>(sequential_test_void(), boost::ref(salg),
                                      boost::ref(this->v1[0])), I);
    return 0.0;
  }
};

struct test_two_view_base
  : public shared_test_base
{
  typedef shared_test_base Base;

  template <typename Generator1, typename Generator2>
  test_two_view_base(
    data_descriptor<pcontainer_type, view_type, Generator1> const& d1,
    data_descriptor<pcontainer_type, view_type, Generator2> const& d2)
    : Base(tuple<
             data_descriptor<pcontainer_type, view_type, Generator1>,
             data_descriptor<pcontainer_type, view_type, Generator2>
           >(d1, d2))
  {}

  template<typename Func, typename Ret>
  double run_parallel_test(Func &palg, Ret& pv, const int I)
  {
    if (report_parallel())
    {
      confidence_interval_controller control(I,I);
      stapl::counter<stapl::default_timer> t;
      for (int i=0; i<I; ++i)
      {
        t.reset();
        t.start();
        pv = palg(this->v0[0], this->v0[1]);
        control.push_back(t.stop());
      }
      return control.stats().avg;
    }
    return 0.0;
  }

  template<typename Func>
  double run_parallel_test(Func &palg, view_type** pv, const int I)
  {
    if (report_parallel())
    {
      confidence_interval_controller control(I,I);
      stapl::counter<stapl::default_timer> t;
      for (int i=0; i<I; ++i)
      {
        if (*pv != nullptr)
          delete *pv;
        t.reset();
        t.start();
        *pv = new view_type(palg(this->v0[0], this->v0[1]));
        control.push_back(t.stop());
      }
      return control.stats().avg;
    }
    return 0.0;
  }

  template<typename Func, typename Ret>
  Ret run_parallel_test(Func& palg, const int I, double& time)
  {
    confidence_interval_controller control(I,I);
    stapl::counter<stapl::default_timer> t;
    t.reset();
    t.start();
    Ret ref = palg(this->v0[0], this->v0[1]);
    control.push_back(t.stop());
    for (int i=0; i<I-1; ++i)
    {
      t.reset();
      t.start();
      ref = palg(this->v0[0], this->v0[1]);
      control.push_back(t.stop());
    }
    time = control.stats().avg;
    return ref;
  }

  double run_parallel_test(boost::function<void (view_type&, view_type&)> &palg,
                           const int I)
  {
    if (report_parallel())
    {
      confidence_interval_controller control(I,I);
      stapl::counter<stapl::default_timer> t;
      for (int i=0; i<I; ++i)
      {
        t.reset();
        t.start();
        palg(this->v0[0], this->v0[1]);
        control.push_back(t.stop());
      }
      return control.stats().avg;
    }
    return 0.0;
  }

  template<typename Ret>
  struct sequential_test_4arg
  {
    void operator()(Ret& sv,
                    boost::function<Ret (iterator_type, iterator_type,
                                         iterator_type, iterator_type)>& salg,
                    stl_container_type& v1, stl_container_type& v2) const
    {
      //note - one of the test_twoews take the end of 2nd as well.
      //for other salgs, bind just ignores 4th argument.
      sv = salg(v1.begin(), v1.end(), v2.begin(), v2.end());
    }
  };

  struct sequential_test_2arg_void
  {
    void operator()(boost::function<void (iterator_type, iterator_type)>& salg,
                    stl_container_type& v1, stl_container_type& v2) const
    {
      salg(v1.begin(), v2.begin());
    }
  };

  template<typename Ret>
  struct sequential_test
  {
    void operator()(Ret& sv, boost::function<Ret (iterator_type, iterator_type,
                                                  iterator_type)>& salg,
                    stl_container_type& v1, stl_container_type& v2) const
    {
      //note - one of the test_twoews take the end of 2nd as well. for other
      //salgs, bind just ignores 4th argument.

      sv = salg(v1.begin(), v1.end(), v2.begin());
    }
  };

  struct sequential_test_void
  {
    void operator()(boost::function<void (iterator_type, iterator_type,
                                          iterator_type)>& salg,
                    stl_container_type& v1, stl_container_type& v2) const
    {
      salg(v1.begin(), v1.end(), v2.begin());
    }
  };

  template<typename Ret>
  double run_sequential_test_4arg(
           boost::function<Ret (iterator_type, iterator_type, iterator_type,
                                iterator_type)>& salg, Ret& sv, const int I)
  {
    if (sequential_data_needed())
      return timed_do_once(
               bind<void>(sequential_test_4arg<Ret>(), boost::ref(sv),
                          boost::ref(salg), boost::ref(this->v1[0]),
                          boost::ref(this->v1[1])), I);
    else
      return 0.0;
  }

  double run_sequential_test_2arg(
           boost::function<void (iterator_type, iterator_type)>& salg,
           const int I)
  {
    if (sequential_data_needed())
      return timed_do_once(
               bind<void>(sequential_test_2arg_void(), boost::ref(salg),
                          boost::ref(this->v1[0]), boost::ref(this->v1[1])), I);
    else
      return 0.0;
  }

  template<typename Ret>
  double
  run_sequential_test(
    boost::function<Ret (iterator_type, iterator_type, iterator_type)>& salg,
    Ret& sv, const int I)
  {
    if (sequential_data_needed())
      return timed_do_once(
               bind<void>(sequential_test<Ret>(), boost::ref(sv),
                          boost::ref(salg), boost::ref(this->v1[0]),
                          boost::ref(this->v1[1])), I);
    else
      return 0.0;
  }

  double
  run_sequential_test(
    boost::function<void (iterator_type, iterator_type, iterator_type)>& salg,
    const int I)
  {
    if (sequential_data_needed())
      return timed_do_once(
               bind<void>(sequential_test_void(), boost::ref(salg),
                          boost::ref(this->v1[0]), boost::ref(this->v1[1])), I);
    else
      return 0.0;
  }
};

struct test_three_view_base
  : public shared_test_base
{
  typedef shared_test_base Base;

  template <typename Generator1, typename Generator2, typename Generator3>
  test_three_view_base(
    data_descriptor<pcontainer_type, view_type, Generator1> const& d1,
    data_descriptor<pcontainer_type, view_type, Generator2> const& d2,
    data_descriptor<pcontainer_type, view_type, Generator3> const& d3)
    : Base(tuple<
             data_descriptor<pcontainer_type, view_type, Generator1>,
             data_descriptor<pcontainer_type, view_type, Generator2>,
             data_descriptor<pcontainer_type, view_type, Generator3>
           >(d1, d2, d3))
  { }

  template<typename Ret>
  double
  run_parallel_test(
    boost::function<Ret (view_type&, view_type&, view_type&)> &palg, Ret& pv,
    const int I)
  {
    if (report_parallel())
    {
      confidence_interval_controller control(I,I);
      stapl::counter<stapl::default_timer> t;
      for (int i=0; i<I; ++i)
      {
        t.reset();
        t.start();
        pv = palg(this->v0[0], this->v0[1], this->v0[2]);
        control.push_back(t.stop());
      }
      return control.stats().avg;
    }
    return 0.0;
  }

  double
  run_parallel_test(
    boost::function<void (view_type&, view_type&, view_type&)> &palg,
    const int I)
  {
    if (report_parallel())
    {
      confidence_interval_controller control(I,I);
      stapl::counter<stapl::default_timer> t;
      for (int i=0; i<I; ++i)
      {
        t.reset();
        t.start();
        palg(this->v0[0], this->v0[1], this->v0[2]);
        control.push_back(t.stop());
      }
      return control.stats().avg;
    }
    return 0.0;
  }

  double
  run_parallel_test_const(
   boost::function<void (view_type const&, view_type const&, view_type&)> &palg,
    const int I)
  {
    if (report_parallel())
    {
      confidence_interval_controller control(I,I);
      stapl::counter<stapl::default_timer> t;
      for (int i=0; i<I; ++i)
      {
        t.reset();
        t.start();
        palg(this->v0[0], this->v0[1], this->v0[2]);
        control.push_back(t.stop());
      }
      return control.stats().avg;
    }
    return 0.0;
  }

  template<typename Ret>
  struct sequential_test
  {
    void operator()(Ret& sv,
                    boost::function<Ret (iterator_type, iterator_type,
                                         iterator_type, iterator_type)>& salg,
                    stl_container_type& v1, stl_container_type& v2,
                    stl_container_type& v3) const
     {
        sv = salg(v1.begin(), v1.end(), v2.begin(), v3.begin());
     }
   };

  template<typename Ret>
  struct sequential_test_5arg
  {
    void operator()(Ret& sv,
                    boost::function<Ret (iterator_type, iterator_type,
                          iterator_type, iterator_type, iterator_type)>& salg,
                    stl_container_type& v1, stl_container_type& v2,
                    stl_container_type& v3) const
     {
        sv = salg(v1.begin(), v1.end(), v2.begin(), v2.end(), v3.begin());
     }
   };

  template<typename Ret>
  double
  run_sequential_test(
    boost::function<Ret(iterator_type, iterator_type, iterator_type,
                        iterator_type)>& salg,
    Ret& sv, const int I)
  {
    if (sequential_data_needed())
      return timed_do_once(
               bind<void>(sequential_test<Ret>(), boost::ref(sv),
                          boost::ref(salg), boost::ref(this->v1[0]),
                          boost::ref(this->v1[1]), boost::ref(this->v1[2])), I);
    else
      return 0.0;
  }

  template<typename Ret>
  double run_sequential_test_5arg(
    boost::function<Ret(iterator_type, iterator_type, iterator_type,
      iterator_type, iterator_type)>& salg,
    Ret& sv, const int I)
  {
    if (sequential_data_needed())
      return timed_do_once(
               bind<void>(sequential_test_5arg<Ret>(), boost::ref(sv),
                          boost::ref(salg), boost::ref(this->v1[0]),
                          boost::ref(this->v1[1]), boost::ref(this->v1[2])), I);
    else
      return 0.0;
  }
};

}

using test_bases::shared_test_base;
using test_bases::test_one_view_base;
using test_bases::test_two_view_base;
using test_bases::test_three_view_base;

template <typename Ret, int CustomValidation = 0>
struct test_one_view
  : public test_one_view_base
{
  typedef boost::function<Ret (view_type&)>                   PAlgorithm;
  typedef boost::function<Ret (iterator_type, iterator_type)> SAlgorithm;

  template <typename Generator>
  test_one_view(const data_descriptor<pcontainer_type, view_type, Generator>& d)
    : test_one_view_base(d)
  {}

  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm salg)
  {
    Ret sv, pv;
    double pt = this->run_parallel_test(palg, pv, I);
    double st = this->run_sequential_test(salg, sv, I);

    //fixme - can't inline this && into set_result, short circuit causes the
    //non do_once'ers to not enter 2nd func
    bool b1 = this->compare_containers();
    bool b2 = this->check_return(sv, pv);

    return set_result(result, st, pt, b1 && b2);
  }
};

template <>
struct test_one_view<data_type, 1>
  : public test_one_view_base
{
  typedef boost::function<data_type (view_type&)>         PAlgorithm;
  typedef boost::function<iterator_type (iterator_type, iterator_type)>
            SAlgorithm;

  template <typename Generator>
  test_one_view(const data_descriptor<pcontainer_type, view_type,
    Generator>& d)
    : test_one_view_base(d)
  { }

  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm salg, iterator_start_tag = iterator_start_tag())
  {
    double pt;
    data_type pv =
      this->run_parallel_test<PAlgorithm, data_type>(palg, I, pt);

    // Initialize to first element of sequential container to prevent
    // segfault when dereferenced in call to check_return
    iterator_type sv = this->v1[0].begin();

    double st = this->run_sequential_test(salg, sv, I);

    bool b1 = this->compare_containers();
    bool b2 = this->check_return(pv, *sv);
    stapl::rmi_fence();
    return set_result(result, st, pt, b1 && b2);
  }
};

template <>
struct test_one_view<void>
  : public test_one_view_base
{
  typedef boost::function<void (view_type&)>                   PAlgorithm;
  typedef boost::function<void (iterator_type, iterator_type)> SAlgorithm;

  template <typename Generator>
  test_one_view(const data_descriptor<pcontainer_type, view_type, Generator>& d)
    : test_one_view_base(d)
  {}

  bool operator()(timed_test_result& result, const int I,
                  PAlgorithm palg, SAlgorithm salg)
  {
    double pt = this->run_parallel_test(palg, I);
    double st = this->run_sequential_test(salg, I);

    return set_result(result, st, pt, this->compare_containers());
  }
};

// Specialization for partition
template<>
struct test_one_view<size_t, 2>
  : public test_one_view_base
{
  typedef boost::function<size_t (view_type&)>
    PAlgorithm;
  typedef boost::function<iterator_type (iterator_type, iterator_type)>
    SAlgorithm;
  typedef boost::function<bool (view_type&, size_t, iterator_type,
                                iterator_type, iterator_type)>
            Validator;

  template <typename Generator>
  test_one_view(const data_descriptor<pcontainer_type, view_type, Generator>& d)
    : test_one_view_base(d)
  { }

  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm salg, Validator validator)
  {
    size_t p;
    iterator_type s;
    double pt = this->run_parallel_test(palg, p, I);
    double st = this->run_sequential_test(salg, s, I);


    bool b1=true;
    if (test_check_containers())
    {
      b1 = stapl::do_once(validator,this->v0[0], p, this->v1[0].begin(),
        this->v1[0].end(), s);
    }
    return set_result(result, st, pt, b1);
  }
};

//fixme - this and other void specialization can be collapsed by properly
//defining a proper default value for validator
template <>
struct test_one_view<void, 1>
  : public test_one_view_base
{
  typedef boost::function<void (view_type&)>                        PAlgorithm;
  typedef boost::function<void (iterator_type, iterator_type)>      SAlgorithm;
  typedef boost::function<bool (view_type&, iterator_type, iterator_type)>
            Validator;

  template <typename Generator>
  test_one_view(const data_descriptor<pcontainer_type, view_type, Generator>& d)
    : test_one_view_base(d)
  {}

  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm salg, Validator validator)
  {
    double pt = this->run_parallel_test(palg, I);
    double st = this->run_sequential_test(salg, I);

    bool ret_val = (!test_check_containers())
                    ? true
                    : stapl::do_once(validator, this->v0[0],
                                     this->v1[0].begin(),
                                     this->v1[0].end());

    return set_result(result, st, pt, ret_val);
  }
};

template <>
struct test_one_view<view_type::reference> : public test_one_view_base
{
  typedef view_type::reference view_reference_type;

  typedef boost::function<view_reference_type (view_type&)>         PAlgorithm;
  typedef boost::function<iterator_type (iterator_type, iterator_type)>
            SAlgorithm;

  template <typename Generator>
  test_one_view(const data_descriptor<pcontainer_type, view_type,
    Generator>& d)
    : test_one_view_base(d)
  { }

  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm salg, iterator_start_tag = iterator_start_tag())
  {
    double pt;
    view_reference_type pv =
      this->run_parallel_test<PAlgorithm, view_reference_type>(palg, I, pt);
    iterator_type sv;
    double st = this->run_sequential_test(salg, sv, I);

    bool b1 = this->compare_containers();
    bool b2 = this->check_return(sv, pv);
    stapl::rmi_fence();
    bool b3 = this->check_distance(pv,sv);
    return set_result(result, st, pt, b1 && b2 && b3);
  }
};

template <>
struct test_one_view<view_type> : public test_one_view_base
{
  typedef boost::function<view_type (view_type&)>       PAlgorithm;
  typedef boost::function<iterator_type (iterator_type, iterator_type)>
            SAlgorithm;

  template <typename Generator>
  test_one_view(const data_descriptor<pcontainer_type, view_type,
    Generator>& d)
    : test_one_view_base(d)
  { }

  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm salg)
  {
    iterator_type sv;
    view_type*    pv(nullptr);
    double pt = this->run_parallel_test(palg, &pv, I);
    double st = this->run_sequential_test(salg, sv, I);

    bool b1 = true;
    if (pv != nullptr)
    {
      b1 = this->check_container(pv->begin(), pv->end(), this->v1[0].begin());
      delete pv;
    }
    return set_result(result, st, pt, b1);
  }
};

//specialization for equal_range
template<>
struct test_one_view<std::pair<view_type::iterator, view_type::iterator> >
  : public test_one_view_base
{
  typedef view_type::iterator view_iterator_type;

  typedef boost::function<std::pair<view_iterator_type, view_iterator_type>
                          (view_type&)>
            PAlgorithm;
  typedef boost::function<std::pair<iterator_type, iterator_type>
                          (iterator_type, iterator_type)>
            SAlgorithm;

  template <typename Generator>
  test_one_view(const data_descriptor<pcontainer_type, view_type, Generator>& d)
    : test_one_view_base(d)
  {}

  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm salg)
  {
    std::pair<view_iterator_type, view_iterator_type> pv;
    std::pair<iterator_type, iterator_type>           sv;

    double pt = this->run_parallel_test(palg, pv, I);
    double st = this->run_sequential_test(salg, sv, I);

    bool b1 = this->compare_containers();
    bool b2 = this->check_container(pv.first,  this->v0[0].end(), sv.first);
    bool b3 = this->check_container(pv.second, this->v0[0].end(), sv.second);

    return set_result(result, st, pt, b1 && b2 && b3);
  }
};


// test_two_view specializations
template<typename Ret>
struct test_two_view
  : public test_two_view_base
{
  typedef boost::function<Ret (view_type&, view_type&)>
            PAlgorithm;
  typedef boost::function<Ret (iterator_type, iterator_type, iterator_type)>
            SAlgorithm;
  typedef boost::function<Ret (iterator_type, iterator_type, iterator_type,
                               iterator_type)>
            SAlgorithm2;

  template <typename Generator1, typename Generator2>
  test_two_view(const data_descriptor<pcontainer_type,view_type,Generator1>& d1,
                const data_descriptor<pcontainer_type,view_type,Generator2>& d2)
    : test_two_view_base(d1, d2)
  { }

  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm salg)
  {
    Ret sv, pv;
    double pt = this->run_parallel_test(palg, pv, I);
    double st = this->run_sequential_test(salg, sv, I);

    bool b1 = this->compare_containers();
    bool b2 = this->check_return(sv, pv);
    return set_result(result, st, pt, b1 && b2);
  }

  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm2 salg)
  {
    Ret sv, pv;
    double pt = this->run_parallel_test(palg, pv, I);
    double st = this->run_sequential_test_4arg(salg, sv, I);

    bool b1 = this->compare_containers();
    bool b2 = this->check_return(sv, pv);
    return set_result(result, st, pt, b1 && b2);
  }
};

template<>
struct test_two_view<void>
  : public test_two_view_base
{
  using PAlgorithm = boost::function<void (view_type&, view_type&)>;

  using SAlgorithm =
    boost::function<void (iterator_type, iterator_type, iterator_type)>;

  using SAlgorithm2 = boost::function<void (iterator_type, iterator_type)>;

  template <typename Generator1, typename Generator2>
  test_two_view(data_descriptor<pcontainer_type,view_type,Generator1> const& d1,
                data_descriptor<pcontainer_type,view_type,Generator2> const& d2)
    : test_two_view_base(d1, d2)
  { }

  bool operator()(timed_test_result& result, const int I,
                  PAlgorithm palg, SAlgorithm salg)
  {
    double pt = this->run_parallel_test(palg, I);
    double st = this->run_sequential_test(salg, I);
    bool res  = this->compare_containers();
    return set_result(result, st, pt, res);
  }

  bool operator()(timed_test_result& result, const int I,
                  PAlgorithm palg, SAlgorithm2 salg)
  {
    double pt = this->run_parallel_test(palg, I);
    double st = this->run_sequential_test_2arg(salg, I);
    bool res  = this->compare_containers();
    return set_result(result, st, pt, res);
  }
};


template<>
struct test_two_view<view_type::reference>
  : public test_two_view_base
{
  typedef view_type::reference view_reference_type;

  typedef boost::function<view_reference_type (view_type&, view_type&)>
            PAlgorithm;
  typedef boost::function<iterator_type (iterator_type, iterator_type,
                                         iterator_type)>             SAlgorithm;
  typedef boost::function<iterator_type (iterator_type, iterator_type,
                                         iterator_type, iterator_type)>
            SAlgorithm2;

  template <typename Generator1, typename Generator2>
  test_two_view(
    data_descriptor<pcontainer_type, view_type, Generator1> const& d1,
    data_descriptor<pcontainer_type, view_type, Generator2> const& d2)
    : test_two_view_base(d1, d2)
  {}

  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm2 salg)
  {
    iterator_type sv;
    double pt;

    view_reference_type pv =
      this->run_parallel_test<PAlgorithm, view_reference_type>(palg, I, pt);
    double st = this->run_sequential_test_4arg(salg, sv, I);

    bool b1 = this->compare_containers();
    bool b2 = this->check_return(sv, pv);
    stapl::rmi_fence();
    bool b3 = this->check_distance(pv,sv);
    return set_result(result, st, pt, b1 && b2 && b3);
  }
};

template <>
struct test_two_view<view_type>
  : public test_two_view_base
{
  using PAlgorithm = boost::function<view_type (view_type&, view_type&)>;

  using SAlgorithm =
    boost::function<iterator_type (iterator_type, iterator_type,
                                   iterator_type)>;

  using SAlgorithm2 =
    boost::function<iterator_type (iterator_type, iterator_type,
                                   iterator_type, iterator_type)>;

  using Validator2 =
    boost::function<bool (view_type&, view_type&, iterator_type,
                          iterator_type, iterator_type, iterator_type)>;

  template<typename Generator1, typename Generator2>
  test_two_view(
    data_descriptor<pcontainer_type, view_type, Generator1> const& d1,
    data_descriptor<pcontainer_type, view_type, Generator2> const& d2)
    : test_two_view_base(d1, d2)
  { }

  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm salg)
  {
    view_type* pv(nullptr);
    iterator_type sv;
    double pt = this->run_parallel_test(palg, &pv, I);
    double st = this->run_sequential_test(salg, sv, I);

    bool b1 = true;
    if (pv != nullptr)
    {
      b1 = this->check_container(pv->begin(), pv->end(), this->v1[1].begin());
      delete pv;
    }
    return set_result(result, st, pt, b1);
  }

  bool operator()(timed_test_result& result, const int I,
                  PAlgorithm palg, SAlgorithm salg, Validator2 validator)
  {
    view_type* pv(nullptr);
    iterator_type sv;
    double pt = this->run_parallel_test(palg, &pv, I);
    if (pv != nullptr)
      delete pv;
    double st = this->run_sequential_test(salg, sv, I);

    bool b = stapl::do_once(validator,
                            this->v0[0], this->v0[1],
                            this->v1[0].begin(), this->v1[0].end(),
                            this->v1[1].begin(), this->v1[1].end());

    return set_result(result, st, pt, b);
  }

  bool operator()(timed_test_result& result, const int I,
                  PAlgorithm palg, SAlgorithm2 salg, Validator2 validator)
  {
    view_type* pv(nullptr);
    iterator_type sv;
    double pt = this->run_parallel_test(palg, &pv, I);
    if (pv != nullptr)
      delete pv;
    double st = this->run_sequential_test_4arg(salg, sv, I);

    bool b = stapl::do_once(validator,
                            this->v0[0], this->v0[1],
                            this->v1[0].begin(), this->v1[0].end(),
                            this->v1[1].begin(), this->v1[1].end());

    return set_result(result, st, pt, b);
  }
};

//specialization for mismatch
template<>
struct test_two_view<std::pair<view_type::reference, view_type::reference> >
  : public test_two_view_base
{
  typedef view_type::reference view_iterator_type;

  typedef boost::function<std::pair<view_iterator_type, view_iterator_type>
                          (view_type&, view_type&)>
            PAlgorithm;
  typedef boost::function<std::pair<iterator_type, iterator_type>
                          (iterator_type, iterator_type, iterator_type)>
            SAlgorithm;

  template <typename Generator1, typename Generator2>
  test_two_view(
    data_descriptor<pcontainer_type, view_type, Generator1> const& d1,
    data_descriptor<pcontainer_type, view_type, Generator2> const& d2)
    : test_two_view_base(d1, d2)
  {}

  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm salg)
  {
    // Can not default construct a pair of references, so we use a
    // different run_parallel_test and this double.
    double pt;
    std::pair<iterator_type, iterator_type>           sv;

    std::pair<view_type::reference, view_type::reference> pv =
      this->run_parallel_test<PAlgorithm,
        std::pair<view_type::reference, view_type::reference>>(palg, I, pt);
    double st = this->run_sequential_test(salg, sv, I);

    bool b1 = this->compare_containers();
    bool b2 = false;
    if (stapl::is_null_reference(pv.first) && sv.first == v1[0].end() )
      b2 = true;
    else
      b2 =  std::distance(v1[0].begin(),sv.first) ==
            (signed)stapl::index_of(pv.first) - (signed)v0[0].begin().index();
    bool b3 = false;
    if (stapl::is_null_reference(pv.second) && sv.second == v1[1].end() )
      b3 = true;
    else
      b3 =  std::distance(v1[1].begin(),sv.second) ==
            (signed)stapl::index_of(pv.second) - (signed)v0[1].begin().index();

    return set_result(result, st, pt, b1 && b2 && b3);
  }
};


// test_three_view specialization for weighted_inner_product test
template<typename Ret>
struct test_three_view
  : public test_three_view_base
{
  typedef boost::function<Ret (view_type&, view_type&, view_type&)>
            PAlgorithm;
  typedef boost::function<Ret (iterator_type, iterator_type, iterator_type,
                               iterator_type)>
            SAlgorithm;

  template <typename Generator1, typename Generator2>
  test_three_view(
                const data_descriptor<pcontainer_type,view_type,Generator1>& d1,
                const data_descriptor<pcontainer_type,view_type,Generator1>& d2,
                const data_descriptor<pcontainer_type,view_type,Generator2>& d3)
    : test_three_view_base(d1, d2, d3)
  {}

  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm salg)
  {
    Ret sv, pv;
    double pt = this->run_parallel_test(palg, pv, I);
    double st = this->run_sequential_test(salg, sv, I);

    bool b1 = this->compare_containers();
    bool b2 = this->check_return(sv, pv);
    return set_result(result, st, pt, b1 && b2);
  }

};


template<>
struct test_three_view<void>
  : public test_three_view_base
{
  typedef boost::function<void (view_type&, view_type&, view_type&)>
            PAlgorithm;
  typedef boost::function<iterator_type (iterator_type, iterator_type,
                                         iterator_type, iterator_type)>
            SAlgorithm;

  template <typename Generator1, typename Generator2, typename Generator3>
  test_three_view(
    data_descriptor<pcontainer_type, view_type, Generator1> const& d1,
    data_descriptor<pcontainer_type, view_type, Generator2> const& d2,
    data_descriptor<pcontainer_type, view_type, Generator3> const& d3)
    : test_three_view_base(d1, d2, d3)
  {}

  // Used for palgorithms that need 3 views with different sizes and return
  // iterator.
  // When calling the std algorithm, we pass 5 iterators
  // (v1.begin, v1.end, v2.begin, v2.end, v3.begin)
  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm salg)
  {
    iterator_type sv;
    double pt = this->run_parallel_test(palg, I);
    double st = this->run_sequential_test(salg, sv, I);

    bool b1 = this->compare_containers();
    return set_result(result, st, pt, b1);
  }
};

// This specialization is for the 5 argument STL function that merge needs.
template<>
struct test_three_view<iterator_type>
  : public test_three_view_base
{
  typedef boost::function<void (view_type const&, view_type const&, view_type&)>
            PAlgorithm;
  typedef boost::function<iterator_type (iterator_type, iterator_type,
            iterator_type, iterator_type, iterator_type)>
            SAlgorithm;

  template <typename Generator1, typename Generator2, typename Generator3>
  test_three_view(
    data_descriptor<pcontainer_type, view_type, Generator1> const& d1,
    data_descriptor<pcontainer_type, view_type, Generator2> const& d2,
    data_descriptor<pcontainer_type, view_type, Generator3> const& d3)
    : test_three_view_base(d1, d2, d3)
  {}

  bool operator()(timed_test_result& result, const int I, PAlgorithm palg,
                  SAlgorithm salg)
  {
    iterator_type sv;
    double pt = this->run_parallel_test_const(palg, I);
    double st = this->run_sequential_test_5arg(salg, sv, I);

    bool b1 = this->compare_containers();
   return set_result(result, st, pt, b1);
  }
};
#endif
