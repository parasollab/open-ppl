/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_NESTED_PARALLELISM_COMMON_ALGORITHMS_HPP
#define STAPL_BENCHMARKS_NESTED_PARALLELISM_COMMON_ALGORITHMS_HPP

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/counting_view.hpp>
#include "../runtime_base_profiler.hpp"
#include <iostream>


//////////////////////////////////////////////////////////////////////
/// @brief Work function to increase the given object by 1.
//////////////////////////////////////////////////////////////////////
struct increase_elem
{
  template<typename Reference>
  void operator()(Reference elem) const
  { elem = elem + 1; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to increase all the elements of the given object by 1.
//////////////////////////////////////////////////////////////////////
struct increase_wf
{
  typedef void result_type;

  template<typename T>
  void operator()(T t) const
  { stapl::for_each(stapl::array_view<T>(t), increase_elem()); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to increase all the elements of the given view by 1.
//////////////////////////////////////////////////////////////////////
template<typename View>
void increment_all(View vw)
{ stapl::for_each(vw, increase_wf()); }


//////////////////////////////////////////////////////////////////////
/// @brief Profiler that benchmarks @ref increment_all.
//////////////////////////////////////////////////////////////////////
template<typename View>
class increment_all_prof
: public runtime_base_profiler<>
{
private:
  View m_vw;

public:
  increment_all_prof(View vw, int argc = 0, char** argv = 0)
  : runtime_base_profiler<>("increment_all                ", argc, argv),
    m_vw(vw)
  { }

  void run(void)
  { increment_all(m_vw); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Finds the minimum element in the given view of a container.
///
/// The minimum element is stored in the second argument of the function
/// operator.
//////////////////////////////////////////////////////////////////////
struct min_row_impl
{
  typedef void result_type;

  template <typename T1, typename T2>
  void operator()(T1 t1, T2 t2) const
  { t2 = stapl::min_value(stapl::array_view<T1>(t1)); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Finds the minimum element in the given view of a container of
///        containers and stores it in @p vw2.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2>
void min_row(View1 vw1, View2 vw2)
{ stapl::map_func(min_row_impl(), vw1, vw2); }


//////////////////////////////////////////////////////////////////////
/// @brief Profiler that benchmarks @ref min_row.
//////////////////////////////////////////////////////////////////////
template<typename InputView, typename OutputView>
class min_row_prof
: public runtime_base_profiler<>
{
private:
  InputView  m_ivw;
  OutputView m_ovw;

public:
  min_row_prof(InputView ivw, OutputView ovw, int argc = 0, char** argv = 0)
  : runtime_base_profiler<>("min_row                      ", argc, argv),
    m_ivw(ivw),
    m_ovw(ovw)
  { }

  void run(void)
  { min_row(m_ivw, m_ovw); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Finds the minimum element in the given view of a container.
//////////////////////////////////////////////////////////////////////
struct min_elem_wf
{
  template <typename T>
  typename T::value_type operator()(T t) const
  { return stapl::min_value(stapl::array_view<T>(t)); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Finds the minimum element in the given view of a container of
///        containers.
//////////////////////////////////////////////////////////////////////
template<typename T, typename View>
T min_elem(View vw)
{ return stapl::map_reduce(min_elem_wf(), stapl::min<T>(), vw); }


//////////////////////////////////////////////////////////////////////
/// @brief Profiler that benchmarks @ref min_elem.
//////////////////////////////////////////////////////////////////////
template<typename View, typename T>
class min_elem_prof
: public runtime_base_profiler<>
{
private:
  View m_ivw;
  T&   m_t;

public:
  min_elem_prof(View ivw, T& t, int argc = 0, char** argv = 0)
  : runtime_base_profiler<>("min_elem                     ", argc, argv),
    m_ivw(ivw),
    m_t(t)
  { }

  void run(void)
  { m_t = min_elem<T>(m_ivw); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Copies a @ref counting_view to the passed view.
//////////////////////////////////////////////////////////////////////
struct set_val
{
  typedef void result_type;

  template <typename T>
  void operator()(T t) const
  {
    typedef typename T::value_type value_type;
    stapl::copy(stapl::counting_view<value_type>(t.size(), 10),
                stapl::array_view<T>(t));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Prints one element to @c std::cout.
//////////////////////////////////////////////////////////////////////
struct print_element
{
  typedef void result_type;

  template<typename T>
  void operator()(T t) const
  { std::cout << std::setw(4) << t; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Prints all the elements of a container to @c std::cout.
//////////////////////////////////////////////////////////////////////
struct print_pc
{
  typedef void result_type;

  template<typename T>
  void operator()(T t) const
  {
    stapl::map_func(print_element(), stapl::array_view<T>(t));
    std::cout << std::endl;
    std::cout.flush();
  }
};

#endif
