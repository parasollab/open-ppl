/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <fstream>

#include <vector>
#include <utility>

#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/pack_ops.hpp>
#include <stapl/utility/tuple.hpp>

#include <test/algorithms/test_utils.h>

#include <stapl/array.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>

#include <stapl/containers/distribution/specifications_fwd.hpp>
#include <stapl/containers/distribution/specifications.hpp>

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/skeletons/functional/wavefront.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/reduce.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/filters.hpp>
#include <stapl/skeletons/transformations/nest.hpp>

#include <stapl/runtime/concurrency/thread_local_storage.hpp>

#include "../../test_report.hpp"

using namespace stapl;
using namespace skeletons;
using namespace wavefront_utils;

using value_type = std::vector<int>;


value_type initial_value(size_t flow_width)
{
  return value_type(flow_width, 0);
}

constexpr size_t ndims   = CUR_DIM;
constexpr size_t nlevels = 2;

// Quick type metafunction to create a reversed index_sequence to feed to
// md_distribution_spec and related types.
template<std::size_t N, typename = make_index_sequence<N>>
struct reverse_index;


template<std::size_t N, std::size_t... Indices>
struct reverse_index<N, index_sequence<Indices...>>
{
  using type = index_sequence<((ndims-1) - Indices)...>;
};


using md_spec_traversal_t = reverse_index<ndims>::type;
using spec_type           = md_distribution_spec<md_spec_traversal_t>::type;
using tuple_t             = homogeneous_tuple_type<ndims, size_t>::type;
using flow_type           = std::vector<int>;
using timer_type          = counter<default_timer>;
using grind_vals_t        = std::vector<std::vector<int>>;
using pg_dims_vec_t       = std::vector<tuple_t>;
using traversal_t         = typename default_traversal<ndims>::type;
using partition_type      = multiarray_impl::block_partition<traversal_t>;
using gid_type            = typename spec_type::gid_type;
using linear_t            = nd_linearize<gid_type, traversal_t>;
using domain_t            = indexed_domain<size_t, ndims, traversal_t>;
using corner_t            = std::array<skeletons::position, ndims>;

static STAPL_RUNTIME_THREAD_LOCAL(grind_vals_t, grind_vals)

//
// Nested Container Creation Functions
//
template<size_t NDims, typename = make_index_sequence<NDims>>
class get_spec_wf;


template<size_t NDims, size_t ...Indices>
class get_spec_wf<NDims, index_sequence<Indices...>>
{
private:
  using system_ct_t       = dist_view_impl::system_container;
  using location_groups_t = std::vector<system_ct_t>;

  /// @brief Each element corresponds to a given level of the
  /// nested execution and describes the number of paragraphs
  /// we want working in each dimensions of the virtual, multi-dimensional
  /// set of locations allocated to the problem.  Hence, this is how
  /// many elements we spawn in this location in non-leaf containers.
  ///
  /// For example an element of (2,2,2) means 8 paragraph will be
  /// spawned, one on each element of the container we're creating.
  pg_dims_vec_t                                   m_pg_dims;

  /// @brief Used in the leaf container as a multiplier with the number
  /// of locations to determine its size.
  size_t                                          m_elements_per_affinity;

  /// @brief Each eleemnt corresponds to a given level of the
  /// nested execution and describes the number of locations that should
  /// be used in each dimensions to volumetrically parition elements,
  /// spawn PARAGRAPHS, etc.
  std::vector<gid_type>                           m_loc_dims;

  /// @brief A collection of partition of the number of locations available
  /// at each level of the nesting.
  std::shared_ptr<std::vector<location_groups_t>> m_loc_groups_ptr;


  //////////////////////////////////////////////////////////////////////
  /// @brief Populate the given set of location groups by creating a
  /// partition of the given multidimensional set of location by the
  /// paragraph dimensions, and linearizing the indices into sets of
  /// location identifiers.
  //////////////////////////////////////////////////////////////////////
  void initialize_loc_groups(location_groups_t& loc_groups,
                             gid_type const& loc_dims,
                             gid_type const& pg_dims)
  {
#ifndef STAPL_NDEBUG
    auto t = stapl::dist_spec_impl::modulo(loc_dims, pg_dims);
    bool b = stapl::vs_map_reduce([](unsigned int val) { return val == 0; },
                                  stapl::logical_and<bool>(), true, t);

    stapl_assert(b, "Non Conformable location / cellset dims");
#endif

    linear_t linearizer(loc_dims);

    partition_type part(domain_t(loc_dims), pg_dims);

    auto group_domain = part.domain();

    for (auto group_idx = group_domain.first();
         group_idx <= group_domain.last();
         group_idx = group_domain.advance(group_idx, 1))
    {
      auto loc_domain = part[group_idx];

      std::vector<location_type> locs;

      for (auto loc_idx = loc_domain.first();
           loc_idx <= loc_domain.last();
           loc_idx = loc_domain.advance(loc_idx, 1))
        locs.push_back(linearizer(loc_idx));

      loc_groups.emplace_back(std::move(locs));
    }
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Properly initialize loc_dims by multiplying lower levels of
  /// pg_dims.  Also initialize partition of linear locations for each
  /// level of the hierarchy.
  //////////////////////////////////////////////////////////////////////
  get_spec_wf(pg_dims_vec_t pg_dims, size_t elements_per_affinity)
    : m_pg_dims(std::move(pg_dims)),
      m_elements_per_affinity(elements_per_affinity),
      m_loc_groups_ptr(std::make_shared<std::vector<location_groups_t>>())
  {
    for (size_t level = 0; level < m_pg_dims.size(); ++level)
    {
      m_loc_dims.push_back(m_pg_dims[level]);

      for (size_t idx = level + 1; idx < m_pg_dims.size(); ++idx)
        m_loc_dims[level] =
          tuple_t(
            (get<Indices>(m_loc_dims[level])*get<Indices>(m_pg_dims[idx]))...);

      m_loc_groups_ptr->push_back(location_groups_t());
      initialize_loc_groups(m_loc_groups_ptr->back(),
                            m_loc_dims[level],
                            m_pg_dims[level]);
    }
  }

  ~get_spec_wf()
  {
    // Have to teardown the system containers in a reverse order so that
    // child gangs are destroyed before their parents.
    if (m_loc_groups_ptr.unique())
    {
      while (!m_loc_groups_ptr->empty())
        m_loc_groups_ptr->pop_back();
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Provides distribution spec for top level container.  Use all
  /// locations (viewed multi-dimensionally in m_loc_dims[0]), with
  /// pg_dim elemens (in each dimension).
  //////////////////////////////////////////////////////////////////////
  /// @brief Provides distribution spec for top level container.  Use all
  spec_type operator()(void) const
  {
    // Use level == current_level interface
    return uniform_nd<md_spec_traversal_t>(m_pg_dims[0], m_loc_dims[0]);
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Handles all other distribution specs besides top level.
  /// Steady state is uniform_nd splitting locations as define by the
  /// system container for that level.  Bottom level is a balanced
  /// distribution.
  //////////////////////////////////////////////////////////////////////
  template<typename... GIDS>
  spec_type operator()(GIDS... gids) const
  {
    constexpr size_t level = sizeof...(GIDS);

    using linearize_t = nd_linearize<gid_type, default_traversal<ndims>::type>;

    using tuple_ops::back;

    const unsigned int loc_group_id =
      linearize_t(m_pg_dims[level-1])(back(tuple<GIDS const&...>(gids...)));

    if (level <= nlevels-2)
      return uniform_nd<md_spec_traversal_t>(
        m_pg_dims[level],
        m_loc_dims[level],
        (*m_loc_groups_ptr)[level-1][loc_group_id]);

    auto num_elements =
      tuple_ops::transform(
        m_pg_dims[nlevels-1],
        [&](size_t dim) { return dim * m_elements_per_affinity; }
      );

    // else (bottom level)
    return balanced_nd<md_spec_traversal_t>(
      num_elements,
      m_pg_dims[nlevels-1],
      (*m_loc_groups_ptr)[level-1][loc_group_id]);
  }
}; // class get_spec_wf


template<size_t NLevels,
         typename = typename homogeneous_tuple_type<NLevels, spec_type>::type>
struct make_composed_spec;


template<size_t NLevels, typename ...Args>
struct make_composed_spec<NLevels, tuple<Args...>>
{
  template<typename WFParam>
  static auto apply(WFParam&& wf_param)
  STAPL_AUTO_RETURN((
    make_heterogeneous_composed_dist_spec<
      typename std::decay<WFParam>::type, Args...
    >(std::forward<WFParam>(wf_param))
  ))
};


template<size_t Iter, size_t Limit>
struct nd_iterate
{
  template<typename ...Args>
  static void
  apply(tuple_t const& dimensions,
        std::function<void (tuple_t const&)> f,
        Args... args)
  {
    const size_t extent = get<Iter>(dimensions);

    for (size_t idx = 0; idx < extent; ++idx)
      nd_iterate<Iter+1, Limit>::apply(dimensions, f, args..., idx);
  }
};


template<size_t N>
struct nd_iterate<N, N>
{
  template<typename ...Args>
  static void
  apply(tuple_t const& dimensions,
        std::function<void (tuple_t const&)> f,
        Args... args)
  {
    f(tuple_t(args...));
  }
};


template<size_t level>
struct make_container
{
  using type =
    multiarray<ndims, typename make_container<level-1>::type, spec_type>;

  template<typename View>
  static void initialize(View const& view, size_t flow_width)
  {
    nd_iterate<0, ndims>::apply(
      view.dimensions(),
      [&view, flow_width](tuple_t const& idx)
        {  make_container<level-1>::initialize(view[idx], flow_width); }
    );
  }
};


template<>
struct make_container<1>
{
  using type = multiarray<ndims, value_type, spec_type>;

  template<typename View>
  static void initialize(View const& view, size_t flow_width)
  {
    nd_iterate<0, ndims>::apply(
      view.dimensions(),
      [&view, flow_width](tuple_t const& idx)
        { view[idx] = initial_value(flow_width); }
    );
  }
};


//
// Nested Skeleton Creation Functions
//
using namespace wavefront_utils;

using skeletons::no_filter;


class grind_wf
{
private:
  tuple_t   m_index;
  size_t    m_pipe_depth;

public:
  using result_type = flow_type;

  grind_wf(tuple_t const& index, size_t pipe_depth)
    : m_index(index), m_pipe_depth(pipe_depth)
  { }

  template<typename Vals, typename Return>
  void impl(Vals& vals, Return& ret) const
  {
    for (size_t i = 0; i < m_pipe_depth; ++i)
      ret += vals[0][i];
  }

  template<typename Vals, typename Return,
           typename Boundary0, typename ...Boundaries>
  void impl(Vals& vals, Return& ret,
            Boundary0&& boundary0, Boundaries&&... boundaries) const
  {
    const auto v_size = boundary0.size();

    for (size_t i = 0; i < v_size; ++i)
    {
      ret += boundary0[i];
      impl(vals, ret, std::forward<Boundaries>(boundaries)...);
    }
  }

  template<typename Center, typename Boundary0, typename ...Boundaries>
  result_type
  operator()(Center&&, Boundary0&& boundary0, Boundaries&&... boundaries) const
  {
    static_assert(
      sizeof...(Boundaries) + 1 == ndims, "Invalid grind_wf invocation");

    timer_type grind_timer;
    grind_timer.start();

    auto&& vals = grind_vals.get();

    result_type ret(vals.size());

    const auto v_size = vals.size();

    for (size_t i = 0; i < v_size; ++i)
    {
      ret[i] = boundary0[i];
      impl(vals, ret[i], std::forward<Boundaries>(boundaries)...);
    }

    return ret;
  }

  void define_type(typer& t)
  {
    t.member(m_index);
    t.member(m_pipe_depth);
  }
}; // class grind_wf


template<size_t level>
struct make_skeleton
{
  static auto apply(corner_t const& corner, size_t pipe_depth)
  STAPL_AUTO_RETURN((
    wavefront(
      make_skeleton<level-1>::apply(corner, pipe_depth), corner,
      skeleton_traits<stapl::use_default, !(level == nlevels)>())))
};


template<>
struct make_skeleton<1>
{
  static auto apply(corner_t const& corner, size_t pipe_depth)
  STAPL_AUTO_RETURN((
    wavefront(
      grind_wf(tuple_t(), pipe_depth), corner,
      skeleton_traits<stapl::use_default, true>())))
};


//
// Invokes the given skeleton with the given view, using the input view
// as the boundary condition for each of the n dimensions as well.
//
template<size_t NDims, typename = make_index_sequence<NDims>>
struct execute_pg;


template<size_t NDims, size_t ...Indices>
struct execute_pg<NDims, index_sequence<Indices...>>
{
  template<size_t N, typename View>
  static View& reflect(View& view)
  { return view; }

  template<typename Skeleton, typename View>
  static void apply(Skeleton& skel, View& view)
  {
    skeletons::execute(
      skeletons::default_execution_params(),
      skel,
      view, reflect<Indices>(view)...);
  }
};


double test_nd(pg_dims_vec_t pg_dims, size_t flow_width,
               size_t pipe_depth, size_t overpartition_factor)
{
  //
  // Make the container and give ownership to a view over it.
  // Initialize a single value in the container.
  //
  using container_t  = make_container<nlevels>::type;
  using view_t       = multiarray_view<container_t>;

  auto composed_spec =
    make_composed_spec<nlevels>::apply(
      get_spec_wf<ndims>(std::move(pg_dims), overpartition_factor));

  view_t in_view_nd(new container_t(composed_spec));

  do_once([&in_view_nd, flow_width]() {
    make_container<nlevels>::initialize(in_view_nd, flow_width);
  });

  //
  // Define the starting corner
  //
  std::array<skeletons::position, ndims> corner;

  for (auto&& elem : corner)
    elem = position::first;

  //
  // Create the Nested Skeleton
  //
  auto skel = nest(make_skeleton<nlevels>::apply(corner, pipe_depth));

  //
  // Execute the algorithm and return measured time.
  //
  timer_type timer;
  timer.start();

  execute_pg<ndims>::apply(skel, in_view_nd);

  return timer.stop();
}


template<std::size_t NDims, typename = make_index_sequence<NDims>>
struct estimate_grind;


template<std::size_t NDims, std::size_t... Indices>
struct estimate_grind<NDims, index_sequence<Indices...>>
{
  template<typename WF>
  static void
  apply(WF wf, size_t local_width, size_t overpartition_factor)
  {
    do_once([wf, local_width, overpartition_factor]
    {
      //
      // Generate random data for NDims boundaries.
      //
      std::vector<std::vector<int>> boundaries(NDims);

      for (auto&& boundary : boundaries)
      {
        boundary.resize(local_width);

        for (auto&& elem : boundary)
          elem = rand() % 50000;
      }

      //
      // Run multiple iterations and compute average.
      //
      timer_type grind_timer;
      grind_timer.start();

      size_t n_iters = 5;

      // Pass first boundary as center (ignored in grind_wf for now).
      for (size_t iter = 0; iter < n_iters; ++iter)
        for (size_t i = 0; i < overpartition_factor; ++i)
          wf(boundaries[0], boundaries[Indices]...);

      const double grind_elapsed =
        grind_timer.stop() / static_cast<double>(n_iters);

      std::cout << "Estimated grind time per core = " << grind_elapsed << "\n";
    });
  }
}; // struct estimate_grind


template<std::size_t ...Indices>
tuple_t
vec_to_tuple(std::vector<size_t> const& dims, index_sequence<Indices...>)
{
  return tuple_t(dims[Indices]...);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc != 4 + nlevels)
    abort(
     "./pg_p2p_data_flow_2level num_elem factor pipe_depth pg_dims0 ..."
    );

  const size_t nlocs                = get_num_locations();
  const size_t num_elems            = atoi(argv[1]);
  const size_t overpartition_factor = atoi(argv[2]);
  const size_t pipe_depth           = atoi(argv[3]);

  std::vector<std::vector<size_t>> input_dims;

  for (int i = 4; i < argc; ++i)
  {
    input_dims.push_back(std::vector<size_t>());

    std::string s = argv[i];

    auto match_comma = [](char c) { return c == ','; };

    const size_t current_ndims =
      std::count_if(s.begin(), s.end(), match_comma);

    if (ndims-1 != current_ndims)
      abort("Invalid pg_dims specification");

    auto iter = s.begin();

    for (size_t i=0; i<ndims; ++i)
    {
      auto end_iter = std::find_if(iter, s.end(), match_comma);

      std::string q(iter, end_iter);

      input_dims.back().push_back(atoi(q.c_str()));

      if (end_iter != s.end())
        iter = ++end_iter;
    }
  }

   pg_dims_vec_t pg_dims;

   for (auto&& v : input_dims)
   { pg_dims.push_back(vec_to_tuple(v, make_index_sequence<ndims>())); }

   size_t computed_locs = 1;
   for (auto&& dims : pg_dims)
     computed_locs *= tuple_ops::fold(dims, 1, stapl::multiplies<size_t>());

   if (nlocs != computed_locs)
     abort("Invalid nesting specification");

   size_t dim_size = 1;
   for (auto&& dims : pg_dims)
     dim_size *= get<0>(dims);

   const size_t flow_width = num_elems / dim_size / overpartition_factor;

   do_once([&]()
   {
     std::cout << ndims
               << "D Wavefront Skeleton Composition, processor dimensions = ";

     for (auto&& t : pg_dims)
     {
       std::cout << "(";
       print_tuple(std::cout, t);
       std::cout << ") ";
     }

     std::cout << "\nFirst Overall Procs Dimension = " << dim_size
               << "\nLocal store size is " << flow_width
               << ", pipe_depth pipe = " << pipe_depth << "\n";
   });

  //
  // Generate
  //
  auto&& vals = grind_vals.get();

  vals.resize(flow_width);

  auto value_gen = []{ return rand() % 50000; };

  for (auto&& v : vals)
  {
    v.resize(pipe_depth);
    for (auto&& elem : v)
      elem = value_gen();
  }

  do_once([]{ std::cout << "Number Generation Done.\n"; });

  estimate_grind<ndims>::apply(
    grind_wf(tuple_t(), pipe_depth), flow_width, overpartition_factor);

  for (int i = 0; i < 32; ++i)
  {
    timer_type benchmark_timer;
    benchmark_timer.start();

    test_nd(pg_dims, flow_width, pipe_depth, overpartition_factor);

    const double benchmark_elapsed = benchmark_timer.stop();

    array<double> times_ct(get_num_locations());
    times_ct[get_location_id()] = benchmark_elapsed;
    array_view<array<double>> times_vw(times_ct);
    double result = reduce(times_vw, max<double>());

    do_once([i, result]()
    {
      std::cout << "Iteration " << i
                << " time=" << result << "\n";
    });
  }

  return EXIT_SUCCESS;
}
