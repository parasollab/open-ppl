/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_TEST_CONTAINERS_VIEWBASED_DIST_FUNCS_HPP
#define STAPL_TEST_CONTAINERS_VIEWBASED_DIST_FUNCS_HPP

#include <stapl/algorithm.hpp>
#include <stapl/numeric.hpp>

struct all_locs_equal
  : public stapl::p_object
{
private:
  long int m_value;

public:
  long int get_value(void)
  { return m_value; }

  bool operator()(long int value)
  {
    m_value = value;
    stapl::future<long int> r =
      stapl::allreduce_rmi(stapl::plus<long int>(),
                           this->get_rmi_handle(),
                           &all_locs_equal::get_value);
    long int result = r.get();
    return result == value*this->get_num_locations() ? true : false;
  }
};


struct loc_provider
  : public stapl::p_object
{ };


struct uneven_blocks
{
public:
  typedef unsigned long int gid_type;
  typedef unsigned long int index_type;

  unsigned long int operator()(unsigned long int i)
  {
    // Two adjacent partitions will contain 256 elements.
    // The first contains 192, the second contains 64.
    unsigned long int prev_part_pairs = i/512;
    unsigned long int rem = i - prev_part_pairs*512;
    return prev_part_pairs*2 + (rem < 384 ? 0 : 1);
  }

  void
  update(std::vector<
           std::tuple<std::pair<gid_type,gid_type>, index_type,
           stapl::location_type>> const&,
         size_t)
  { stapl::abort("mapping_base::update called."); }
};


struct reverse_parts
{
private:
  unsigned long int    m_nparts;
  stapl::location_type m_nlocs;

public:
  typedef unsigned long int gid_type;
  typedef stapl::location_type index_type;

  reverse_parts(unsigned long int nparts, stapl::location_type nlocs)
    : m_nparts(nparts), m_nlocs(nlocs)
  { }

  stapl::location_type operator()(unsigned long int i)
  { return (m_nlocs != 1) ? m_nparts - 1 - i : 0; }

  void define_type(stapl::typer& t)
  { t.member(m_nparts); }

  void
  update(std::vector<
           std::tuple<std::pair<gid_type,gid_type>, unsigned long int,
           stapl::location_type>> const&,
         size_t)
  { stapl::abort("mapping_base::update called."); }
};


struct check_parts
{
private:
  loc_provider* m_lp;
public:
  typedef bool result_type;

  check_parts(loc_provider* lp)
    : m_lp(lp)
  { }

  template <typename Partition>
  result_type operator()(Partition p)
  {
    unsigned long int sum  = std::accumulate(p.begin(), p.end(), 0);
    unsigned long int dist = std::distance(p.begin(), p.end());
    unsigned long int inc_sum = dist*(dist-1)/2;

    // Partitions are mapped to locations in reverse order.
    unsigned long int part_id =
      m_lp->get_num_locations() - 1 - m_lp->get_location_id();

    bool passed = sum == (*p.begin())*dist + inc_sum;

    if (part_id % 2 == 0)
    {
      // partition has 384 elements in it.
      return passed && dist == 384;
    } else {
      // partition has 128 elements in it.
      return passed && dist == 128;
    }
  }

  void define_type(stapl::typer& t)
  { t.member(m_lp); }
};


struct mid_init
{
  unsigned int m_outer_size;
  unsigned int m_mid_size;
  unsigned int m_inner_size;
  long int     m_outer_index;

  typedef void result_type;

  mid_init(unsigned int outer_size, unsigned int mid_size,
           unsigned int inner_size, long int outer_index)
    : m_outer_size(outer_size), m_mid_size(mid_size), m_inner_size(inner_size),
      m_outer_index(outer_index)
  { }

  template <typename View, typename Index>
  void operator()(View vw, Index i)
  {
    long int start = m_outer_index*m_mid_size*m_inner_size + i*m_inner_size;
    stapl::generate(vw, stapl::sequence<long int>(start, 1));
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_outer_size);
    t.member(m_mid_size);
    t.member(m_inner_size);
    t.member(m_outer_index);
  }
};


struct outer_init
{
  unsigned int m_outer_size;
  unsigned int m_mid_size;
  unsigned int m_inner_size;

  typedef void result_type;

  outer_init(unsigned int outer_size, unsigned int mid_size,
             unsigned int inner_size)
    : m_outer_size(outer_size), m_mid_size(mid_size), m_inner_size(inner_size)
  { }

  template <typename View, typename Index>
  void operator()(View vw, Index i)
  {
    map_func(mid_init(m_outer_size, m_mid_size, m_inner_size, i), vw,
             stapl::counting_view<unsigned int>(m_mid_size));
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_outer_size);
    t.member(m_mid_size);
    t.member(m_inner_size);
  }
};

struct mid_sum
{
  typedef long int result_type;

  template <typename View>
  result_type operator()(View vw)
  {
    long int zero = 0;
    return stapl::accumulate(vw, zero);
  }
};


struct outer_sum
{
  typedef long int result_type;

  template <typename View>
  result_type operator()(View vw)
  {
    return stapl::map_reduce(mid_sum(), stapl::plus<long int>(), vw);
  }
};
#endif
