/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <boost/lexical_cast.hpp>
#include <stapl/multiarray.hpp>
#include <stapl/array.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/containers/distribution/composed_specification.hpp>
#include <stapl/views/system_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/algorithms/numeric.hpp>
#include <stapl/runtime/counter/default_counters.hpp>

typedef stapl::counter<stapl::default_timer> counter_t;

struct logical_and_update
{
private:
  bool m_local_result;

public:

  logical_and_update(bool local_result)
    : m_local_result(local_result)
  { }

  template<typename Element>
  void operator()(Element&& e) const
  { e = e && m_local_result; }

  void define_type(stapl::typer& t)
  { t.member(m_local_result); }
};

struct inner_check_wf
{
private:
  using size_type =
    stapl::tuple<long unsigned int, long unsigned int, long unsigned int>;

  unsigned int m_version;
  unsigned int m_block_size;
  stapl::rmi_handle::reference m_handle;

public:
  typedef void result_type;

  inner_check_wf(unsigned int version, unsigned int block_size,
                 stapl::rmi_handle::reference handle)
    : m_version(version), m_block_size(block_size), m_handle(handle)
  { }

  template<typename Ref, typename Index>
  result_type operator()(Ref&& r, Index&& i) {
    stapl::array<bool>* correct =
      stapl::resolve_handle<stapl::array<bool>>(m_handle);

    size_type index = i;
    unsigned int loc = stapl::get_affinity().tag;

    if (m_version == 0)
    {
      // Container is distributed across z-dimension
      correct->apply_set(loc,
        logical_and_update(loc == stapl::get<2>(index) / m_block_size));
    }
    else
    {
      // Container is distributed across x-dimension
      correct->apply_set(loc,
        logical_and_update(loc == stapl::get<0>(index) / m_block_size));
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_version);
    t.member(m_block_size);
    t.member(m_handle);
  }
};


struct outer_check_wf
{
private:
  using size_type =
    stapl::tuple<long unsigned int, long unsigned int, long unsigned int>;

  unsigned int m_version;
  unsigned int m_outer_block_size;
  unsigned int m_inner_block_size;

public:
  typedef void result_type;

  outer_check_wf(unsigned int version, unsigned int outer_block_size,
                 unsigned int inner_block_size)
    : m_version(version), m_outer_block_size(outer_block_size),
      m_inner_block_size(inner_block_size)
  { }

  template <typename View, typename Index, typename Correct>
  result_type operator()(View&& vw, Index&& i, Correct&& correct)
  {
    size_type index = i;
    unsigned int loc = stapl::get_affinity().tag;

    if (m_version == 0)
    {
      // Container is distributed across z-dimension
      correct.apply_set(loc,
        logical_and_update(loc == stapl::get<2>(index) / m_outer_block_size));
    }
    else
    {
      // Container is distributed across x-dimension
      correct.apply_set(loc,
        logical_and_update(loc == stapl::get<0>(index) / m_outer_block_size));
    }

    stapl::map_func(inner_check_wf(m_version, m_inner_block_size,
                                   correct.container().get_rmi_handle()),
      vw, stapl::counting_view_nd<3>(vw.dimensions(), size_type(0,0,0)));
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_version);
    t.member(m_outer_block_size);
    t.member(m_inner_block_size);
  }
};



template<typename Container, typename Size>
bool check_affinity(Container& c, Size inner_dims,
                    unsigned int version, unsigned int nlocs)
{
  // Container of results that will be reduced to get the overall result
  stapl::array<bool> elem_correct(nlocs, true);
  stapl::array_view<stapl::array<bool>> elem_correct_view(elem_correct);

  stapl::multiarray_view<Container> cv(c);

  unsigned int outer_block_size, inner_block_size;
  if (version == 0)
  {
    outer_block_size = stapl::get<2>(c.dimensions()) / nlocs;
    inner_block_size = stapl::get<2>(inner_dims) / nlocs;
  }
  else
  {
    outer_block_size = stapl::get<0>(c.dimensions()) / nlocs;
    inner_block_size = stapl::get<0>(inner_dims) / nlocs;
  }

  stapl::map_func(outer_check_wf(version, outer_block_size, inner_block_size),
    cv,
    stapl::counting_view_nd<3>(c.dimensions(),
                               typename Container::index_type(0,0,0)),
    stapl::make_repeat_view_nd<3>(elem_correct_view));

  // fence required to ensure RMI traffic generated in work functions is
  // processed.
  stapl::rmi_fence();

  return stapl::map_reduce(stapl::identity<bool>(), stapl::logical_and<bool>(),
                           elem_correct_view);

  return outer_block_size == inner_block_size;
}


template<typename Spec, typename Traversal>
class get_spec_wf
{
private:
  using gid_type = typename Spec::gid_type;

  gid_type m_outer_sz;
  gid_type m_inner_sz;
  gid_type m_loc_dims;

public:
  get_spec_wf(gid_type outer, gid_type inner, gid_type loc_dims)
    : m_outer_sz(outer), m_inner_sz(inner), m_loc_dims(loc_dims)
  { }

  // Query for Top Level Container
  Spec operator()(void) const
  { return stapl::balanced_nd<Traversal>(m_outer_sz, m_loc_dims); }

  // Query for Inner Containers
  Spec operator()(gid_type gid) const
  { return stapl::balanced_nd<Traversal>(m_inner_sz, m_loc_dims); }
};


stapl::exit_code stapl_main(int argc, char** argv)
{
  long unsigned int outer_size, inner_size;

  if(argc == 3) {
    outer_size = boost::lexical_cast<long unsigned int>(argv[1]);
    inner_size = boost::lexical_cast<long unsigned int>(argv[2]);
  }
  else {
    outer_size = 2 * stapl::get_num_locations();
    inner_size = 2 * outer_size;
  }


  typedef stapl::index_sequence<2, 1, 0>                         D3_traversal;
  typedef stapl::md_distribution_spec<D3_traversal>::type        D3_spec;

  typedef stapl::multiarray<3, double, D3_spec>                  inner_ct_t;
  typedef stapl::multiarray<3, inner_ct_t, D3_spec>              outer_ct_t;

  typedef get_spec_wf<D3_spec, D3_traversal>                     spec_gen_t;

  unsigned int nlocs = stapl::get_num_locations();

  stapl::tuple<size_t, size_t, size_t> loc_dims(1, 1, nlocs);

  stapl::tuple<unsigned long int, unsigned long int, unsigned long int>
    outer_dims(outer_size, outer_size, outer_size),
    inner_dims(inner_size, inner_size, inner_size);

  auto initial_dist =
    stapl::make_heterogeneous_composed_dist_spec<spec_gen_t, D3_spec, D3_spec>(
      spec_gen_t(outer_dims, inner_dims, loc_dims));

  counter_t timer;
  timer.reset();
  timer.start();

  outer_ct_t ct(initial_dist);

  double ctor_time = timer.stop();

  bool ctor_passed = check_affinity(ct, inner_dims, 0, nlocs);

  stapl::do_once([&]() {
    std::cout << "Test: composed_multiarray_construction\n"
              << "Status: ";
    if (ctor_passed)
      std::cout << "PASSED\n";
    else
      std::cout<< "FAILED\n";
    std::cout << "Version: STAPL\n"
              << "Time: " << ctor_time << "\n"
              << "Notes: "
              << "outer_size = (" << outer_size << ", " << outer_size << ", "
              << outer_size << "), "
              << "inner_size = (" << inner_size << ", " << inner_size << ", "
              << inner_size << ")\n";
  });

  // change dimension across which locations are spread.
  stapl::tuple<size_t, size_t, size_t> loc_dims2(nlocs, 1, 1);

  auto new_dist =
    stapl::make_heterogeneous_composed_dist_spec<spec_gen_t, D3_spec, D3_spec>(
      spec_gen_t(outer_dims, inner_dims, loc_dims2));

  timer.reset();
  timer.start();

  ct.redistribute(new_dist);

  double redist_time = timer.stop();

  bool redist_passed = check_affinity(ct, inner_dims, 1, nlocs);

  stapl::do_once([&]() {
    std::cout << "Test: composed_multiarray_redistribution\n"
              << "Status: ";
    if (redist_passed)
      std::cout << "PASSED\n";
    else
      std::cout<< "FAILED\n";
    std::cout << "Version: STAPL\n"
              << "Time: " << redist_time << "\n"
              << "Notes: "
              << "outer_size = (" << outer_size << ", " << outer_size << ", "
              << outer_size << "), "
              << "inner_size = (" << inner_size << ", " << inner_size << ", "
              << inner_size << ")\n";
  });

  return EXIT_SUCCESS;
}
