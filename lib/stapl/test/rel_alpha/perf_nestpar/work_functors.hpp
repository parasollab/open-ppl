/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef NESTPAR_WORKFUNCTORS
#define NESTPAR_WORKFUNCTORS

#include "nestpar_utilities.hpp"
#include <sstream>
#include <stapl/views/counting_view.hpp>




// -------------
// -- Fill --
// -------------
template <typename T>
struct insert_uniformly_wf
{

  typedef void result_type;
  size_t m_size;
  T m_min ;
  T m_max ;
  T ratio ;
  insert_uniformly_wf(size_t sz,
                       T min = 1,
                       T max = std::numeric_limits<T>::max() - 1 )
    : m_size(sz),
      m_min(min),
      m_max(max)
  {
    ratio = m_max/m_size;
    ratio -= m_min/m_size;
  }


  template <typename View>
  void operator()(T i,View& vw)
  {
    T to_insert = m_min +i;
    to_insert *= ratio;
    vw.insert(to_insert);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_size);
  }

};


struct nestpar_inner_set_fill_wf
{

  typedef void result_type;
  typedef typename stapl::result_of::counting_view<int>::type cv_type;

  size_t m_size;
  cv_type m_cv;

  nestpar_inner_set_fill_wf(size_t sz)
    : m_size(sz), m_cv(stapl::counting_view<int>(sz))
  {
        stapl::rmi_fence();
  }

  template <typename View1>
  void operator()(View1 const& vw1)
  {

    insert_uniformly_wf <typename View1::value_type>insertwf(m_size);
    stapl::map_func(insertwf,
                    m_cv,
                    stapl::make_repeat_view(vw1));

  }

  template <typename View1, typename T>
  void operator()(View1&& vw1, T min, T max)
  {
    insert_uniformly_wf <typename View1::value_type>insertwf(m_size, min, max);
    stapl::map_func(insertwf,
                    m_cv,
                    stapl::make_repeat_view(vw1));
    stapl::rmi_fence();
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_size);
    t.member(m_cv);
  }

};


template<int Percent>
struct set_fill_Xpercent
{

  typedef void result_type;

  size_t m_size;
  size_t m_percentage;

  set_fill_Xpercent(size_t sz)
    : m_size(sz), m_percentage(Percent)
  {}

  template <typename View1>
  void operator()(View1& vw1)
  {
    size_t nb_locations = stapl::get_num_locations();
    size_t my_loc = stapl::get_location_id();
    size_t block_sz = m_size/nb_locations;
    // Local is from [my_loc*block_sz,my_loc*block_sz+block_sz]
    for (size_t i = my_loc*block_sz; i < my_loc*block_sz+block_sz; ++i)
    {
      int to_insert = (int)i;
      // Shift so that m_percentage% are not local.
      to_insert=(to_insert+ (int)(block_sz*m_percentage/100) ) % m_size;
      vw1.insert(to_insert);
    }
    stapl::rmi_fence();
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_size);
    t.member(m_percentage);
  }

};

// 2 lvl :

//Simple map_func wrapper:
template <typename FillWF>
struct nestpar_outer_fill_wf
{

  typedef void result_type;

  size_t m_size;

  nestpar_outer_fill_wf(size_t sz)
    : m_size(sz)
  { }

  template <typename View1>
  void operator()(View1&& vw1)
  {
    stapl::map_func(FillWF(m_size), vw1);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_size);
  }

};


// -------------
// -- Process --
// -------------

struct nestpar_map_reduce_process_wf
{
  typedef int result_type;

  template <typename View1>
  int operator()(View1&& vw1)
  {
    return stapl::map_reduce( id_int_wf(), min_int_wf(), vw1);
  }
};


// 2-lvl
struct nestpar_map_reduce_2lvl_wf
{
  typedef int result_type;

  template <typename View1>
  int operator()(View1&& vw1)
  {
    return stapl::map_reduce(nestpar_map_reduce_process_wf(),
                             min_int_wf(),
                             vw1);
  }
};

#endif
