/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <boost/lexical_cast.hpp>
#include <stapl/vector.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/containers/distribution/composed_specification.hpp>
#include <stapl/views/system_view.hpp>
#include <stapl/algorithms/numeric.hpp>
#include <stapl/runtime/counter/default_counters.hpp>
#include "../../confint.hpp"

typedef stapl::counter<stapl::default_timer> counter_t;

struct inner_check_wf
{
private:
  long unsigned int m_inner_size;
  unsigned int      m_nlocs;

  // version == 0 -> balance, 1 -> cyclic
  unsigned int m_version;

  size_t m_outer_index;
  size_t m_mid_index;

public:
  using result_type = bool;

  inner_check_wf(long unsigned int inner_size, unsigned int nlocs,
                 unsigned int version, size_t oindex, size_t mindex)
    : m_inner_size(inner_size), m_nlocs(nlocs), m_version(version),
      m_outer_index(oindex), m_mid_index(mindex)
  { }

  template<typename View, typename Index>
  result_type operator()(View&& vw, Index&& i)
  {
    size_t index = i;

    unsigned int loc = stapl::get_affinity().tag;

    if (m_version == 0)
      return (index / (m_inner_size / m_nlocs)) == loc;
    else
      return index % m_nlocs == loc;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_inner_size);
    t.member(m_nlocs);
    t.member(m_version);
    t.member(m_outer_index);
    t.member(m_mid_index);
  }
};


struct mid_check_wf
{
private:
  long unsigned int m_mid_size;
  long unsigned int m_inner_size;
  unsigned int      m_nlocs;

  // version == 0 -> cyclic, 1 -> balance
  int m_version;

  size_t            m_outer_index;

public:
  using result_type = bool;

  mid_check_wf(long unsigned int mid_size, long unsigned int inner_size,
               unsigned int nlocs, unsigned int version, size_t oidx)
    : m_mid_size(mid_size), m_inner_size(inner_size), m_nlocs(nlocs),
      m_version(version), m_outer_index(oidx)
  { }

  template<typename View, typename Index>
  result_type operator()(View&& vw, Index&& i)
  {
    size_t index = i;

    bool res =
      stapl::map_reduce(inner_check_wf(m_inner_size, m_nlocs,
                                       m_version, m_outer_index, index),
                        stapl::logical_and<bool>(),
                        vw, stapl::counting_view(vw.size(), 0));

    unsigned int loc = stapl::get_affinity().tag;

    if (m_version == 0)
      return res && index % m_nlocs == loc;
    else
      return res && (index / (m_mid_size / m_nlocs)) == loc;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_mid_size);
    t.member(m_inner_size);
    t.member(m_nlocs);
    t.member(m_version);
    t.member(m_outer_index);
  }
};


struct outer_check_wf
{
private:
  long unsigned int m_outer_size;
  long unsigned int m_mid_size;
  long unsigned int m_inner_size;
  unsigned int      m_nlocs;

  // version == 0 -> balance, 1 -> cyclic
  unsigned int m_version;

public:
  using result_type = bool;

  outer_check_wf(long unsigned int osize, long unsigned int msize,
                 long unsigned int isize, unsigned int nlocs,
                 unsigned int version)
    : m_outer_size(osize), m_mid_size(msize), m_inner_size(isize),
      m_nlocs(nlocs), m_version(version)
  { }


  template<typename View, typename Index>
  result_type operator()(View&& vw, Index&& i)
  {
    size_t index = i;
    unsigned int loc = stapl::get_affinity().tag;

    auto res =
      stapl::map_reduce(mid_check_wf(m_mid_size, m_inner_size, m_nlocs,
                                     m_version, index),
                        stapl::logical_and<bool>(),
                        vw, stapl::counting_view(vw.size(), 0));

    if (m_version == 0)
      return res && (index / (m_outer_size / m_nlocs)) == loc;
    else
      return res && index % m_nlocs == loc;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_outer_size);
    t.member(m_mid_size);
    t.member(m_inner_size);
    t.member(m_nlocs);
    t.member(m_version);
  }
};


template<typename Container>
bool check_affinity(Container& c, long unsigned int osize,
                    long unsigned int msize, long unsigned int isize,
                    unsigned int nlocs, unsigned int version)
{
  stapl::vector_view<Container> cv(c);

  return stapl::map_reduce(outer_check_wf(osize, msize, isize, nlocs, version),
                           stapl::logical_and<bool>(),
                           cv, stapl::counting_view(c.size(), 0));
}


stapl::exit_code stapl_main(int argc, char** argv)
{
  counter_t timer;
  double ctor_time(0.), redist_time(0.);

  long unsigned int outer_size, mid_size, inner_size;

  unsigned int nlocs = stapl::get_num_locations();

  if(argc == 4)
  {
    outer_size = boost::lexical_cast<long unsigned int>(argv[1]);
    mid_size = boost::lexical_cast<long unsigned int>(argv[2]);
    inner_size = boost::lexical_cast<long unsigned int>(argv[3]);
  }
  else
  {
    outer_size = 2 * nlocs;
    mid_size   = 2 * outer_size;
    inner_size = 2 * outer_size;
  }

  using dist_spec_tp = stapl::distribution_spec<>;
  using vb_partition = stapl::view_based_partition<dist_spec_tp>;
  using vb_mapper    = stapl::view_based_mapper<dist_spec_tp>;

  typedef stapl::vector<double, vb_partition, vb_mapper>        inner_ct_t;
  typedef stapl::vector<inner_ct_t, vb_partition, vb_mapper>    mid_ct_t;
  typedef stapl::vector<mid_ct_t, vb_partition, vb_mapper>      outer_ct_t;

  auto initial_dist =
    stapl::make_composed_dist_spec(
      // index is the multidimensional index of the container being constructed.
      [&](std::vector<size_t> const& index)
      {
        // If index is empty, the outer container is being constructed.
        if (index.empty())
          return stapl::balance(outer_size);

        // middle containers will start with cyclic distribution
        else if (index.size() == 1)
          return stapl::cyclic(mid_size);

        // leaf containers will match outer container distribution
        else
          return stapl::balance(inner_size);
      }
    );

  timer.reset();
  timer.start();

  // Construct the composed container
  outer_ct_t ct(initial_dist);

  ctor_time = timer.stop();

  bool ctor_passed =
    check_affinity(ct, outer_size, mid_size, inner_size, nlocs, 0);

  // The new distribution replaces balance distributions with cyclic,
  // and vice-versa
  auto new_dist =
    stapl::make_composed_dist_spec(
      // index is the multidimensional index of the container being constructed.
      [&](std::vector<size_t> const& index)
      {
        // If index is empty, the outer container is being constructed.
        if (index.empty())
          return stapl::cyclic(outer_size);

        // middle containers will be redistributed to balanced distributions
        else if (index.size() == 1)
          return stapl::balance(mid_size);

        // leaf containers will match outer container distribution with a
        // cyclic distribution
        else
          return stapl::cyclic(inner_size);
      }
    );

  timer.reset();
  timer.start();

  ct.redistribute(new_dist);

  redist_time = timer.stop();

  bool redist_passed =
    check_affinity(ct, outer_size, mid_size, inner_size, nlocs, 1);

  stapl::do_once([&]() {
    std::cout << "Test: composed_vector_construction\n"
              << "Status: ";
    if (ctor_passed)
      std::cout << "PASSED\n";
    else
      std::cout<< "FAILED\n";
    std::cout << "Version: STAPL\n"
              << "Time: " << ctor_time << "\n"
              << "Notes: size = ("
              << outer_size << ", " << mid_size << ", " << inner_size << ")\n";

    std::cout << "Test: composed_vector_redistribution\n"
              << "Status: ";
    if (redist_passed)
      std::cout << "PASSED\n";
    else
      std::cout<< "FAILED\n";
    std::cout << "Version: STAPL\n"
              << "Time: " << redist_time << "\n"
              << "Notes: size = ("
              << outer_size << ", " << mid_size << ", " << inner_size << ")\n";
  });

  return EXIT_SUCCESS;
}
