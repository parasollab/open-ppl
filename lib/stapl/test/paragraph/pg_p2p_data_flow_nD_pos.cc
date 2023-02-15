/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/utility/pack_ops.hpp>
#include <stapl/containers/partitions/block_partition.hpp>
#include <test/algorithms/test_utils.h>
#include <stapl/runtime/concurrency/thread_local_storage.hpp>

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/skeletons/reduce.hpp>
#include <stapl/skeletons/spans/balanced.hpp>

using namespace stapl;

constexpr size_t ndims = 2;

using tuple_t          = homogeneous_tuple_type<ndims, size_t>::type;
using traversal_t      = typename default_traversal<ndims>::type;
using problem_domain_t = indexed_domain<size_t, ndims, traversal_t>;
using linear_t         = nd_linearize<tuple_t, traversal_t>;
using rev_linear_t     = nd_reverse_linearize<tuple_t, traversal_t>;
using flow_type        = std::vector<int>;
using timer_type       = counter<default_timer>;
using grind_vals_t     = std::vector<std::vector<int>>;
using pg_dims_vec_t    = std::vector<tuple_t>;

static STAPL_RUNTIME_THREAD_LOCAL(grind_vals_t, grind_vals)


class pg_placement_strategy
{
private:
  using domain_type     = indexed_domain<std::size_t>;
  using partition_type  = balanced_partition<domain_type>;

  size_t m_num_sections;

  using static_local_placement = std::true_type;

public:
  pg_placement_strategy(size_t num_sections)
    : m_num_sections(num_sections)
  { }

  template<typename WF, typename ...Views>
  typename std::enable_if<!is_factory<WF>::value, locality_info>::type
  execution_location(WF const& wf, Views const&... views)
  { return default_task_placement().execution_location(wf, views...); }

  template<typename WF, typename ...Views>
  typename std::enable_if<
    is_factory<WF>::value,
    std::pair<rmi_handle::reference, std::vector<unsigned int>>
  >::type
  execution_location(WF const& wf, Views const&...)
  {
    partition_type sections(domain_type(get_num_locations()), m_num_sections);

    auto locations = sections[wf.index()];

    std::vector<unsigned int> v;
    v.reserve(locations.size());

    for (auto idx = locations.first(); idx <= locations.last(); ++idx)
      v.push_back(idx);

    return std::make_pair(rmi_handle::reference(), std::move(v));
  }

  default_info get_sched_info(void) noexcept
  { return default_info(); }

  void notify_finished(void) noexcept
  { }

  void define_type(typer &t)
  { t.member(m_num_sections); }
}; // class pg_placement_strategy


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

  template<typename Boundary0, typename ...Boundaries>
  result_type
  operator()(Boundary0&& boundary0, Boundaries&&... boundaries) const
  {
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
  { t.member(m_index); }
}; // class grind_wf


class boundary_wf
{
private:
  size_t m_size;

public:
  using result_type = flow_type;

  boundary_wf(size_t size)
    : m_size(size)
  { }

  result_type operator()(void) const
  {
    flow_type f(m_size);

    std::fill(f.begin(), f.end(), 0);

    return f;
  }

  void define_type(typer& t)
  { t.member(m_size); }
};


template<typename Partition, typename TaskIdMapper, typename LocationMapper>
class task_mapper
{
private:
  Partition      m_partition;
  TaskIdMapper   m_task_id_mapper;
  LocationMapper m_location_mapper;

public:
  task_mapper(Partition      partition,
              TaskIdMapper   task_id_mapper,
              LocationMapper location_mapper)
    : m_partition(std::move(partition)),
      m_task_id_mapper(std::move(task_id_mapper)),
      m_location_mapper(std::move(location_mapper))
  { }

  std::pair<location_type, loc_qual>
  operator()(size_t task_id) const
  {
    const auto problem_point = m_task_id_mapper(task_id);
    const auto ndim_loc      = m_partition.find(problem_point);
    const auto loc           = m_location_mapper(ndim_loc);

    return std::pair<location_type, loc_qual>(loc, LQ_CERTAIN);
  }

  constexpr bool is_perfect_mapper(void) const
  { return true; }
};


template<typename Iterator>
location_type
recursive_map(Iterator begin_iter, Iterator end_iter,
              tuple_t point, problem_domain_t const& loc_domain)
{
  using partition_t      = multiarray_impl::block_partition<traversal_t>;
  partition_t parts(loc_domain, *begin_iter);

  linear_t lin(*begin_iter);

  auto part   = parts.find(point);
  auto domain = parts[part];
  auto offset = lin(part) * domain.size();

  if (++begin_iter == end_iter)
   return offset;

  return offset + recursive_map(begin_iter, end_iter, point, domain);
}


template<size_t NDims, typename = make_index_sequence<NDims>>
class pin_mapper;


template<size_t NDims, size_t ...Indices>
class pin_mapper<NDims, index_sequence<Indices...>>
{
private:
  using traversal_t = typename default_traversal<NDims>::type;
  using partition_t = multiarray_impl::block_partition<traversal_t>;
  using pair_t      = std::pair<location_type, loc_qual>;

  rev_linear_t   m_rev_linearizer;
  pg_dims_vec_t  m_pg_dims;
  partition_t    m_partition;

public:
  pin_mapper(rev_linear_t  rev_linearizer,
             pg_dims_vec_t pg_dims,
             partition_t   partition)
    : m_rev_linearizer(std::move(rev_linearizer)),
      m_pg_dims(std::move(pg_dims)),
      m_partition(std::move(partition))
  {
    stapl_assert(m_pg_dims.size() > 0, "Invalid pg_dims size");
  }


  pair_t operator()(size_t key) const
  {
    auto problem_point     = m_rev_linearizer(key);
    tuple_t ndim_loc       = m_partition.find(problem_point);

    tuple_t child_loc_dims = m_pg_dims[0];

    for (size_t i=1; i< m_pg_dims.size(); ++i)
     child_loc_dims =
       tuple_t((get<Indices>(child_loc_dims) * get<Indices>(m_pg_dims[i]))...);

    location_type loc =
      recursive_map(m_pg_dims.rbegin(), m_pg_dims.rend(),
                    ndim_loc, problem_domain_t(child_loc_dims));

    return pair_t(loc, LQ_CERTAIN);
  }


  constexpr bool is_perfect_mapper(void) const
  { return true; }


  void set_num_locations(size_t nlocs)
  { }

  void define_type(typer& t)
  {
    t.member(m_rev_linearizer);
    t.member(m_pg_dims);
    t.member(m_partition);
  }
}; // class pin_mapper


class boundary_factory
  : public factory_wf
{
private:
  size_t           m_child_index;
  problem_domain_t m_boundary_domain;
  size_t           m_flow_width;
  linear_t         m_linearizer;
  rev_linear_t     m_rev_linearizer;

public:
  using result_type       = flow_type;

  boundary_factory(size_t child_index,
                   problem_domain_t boundary_domain,
                   size_t flow_width,
                   linear_t linearizer,
                   rev_linear_t rev_linearizer)
    : m_child_index(child_index),
      m_boundary_domain(boundary_domain),
      m_flow_width(flow_width),
      m_linearizer(linearizer),
      m_rev_linearizer(rev_linearizer)
  { }

  using scheduler_type      = default_scheduler;
  using coarsener_type      = null_coarsener;
  using domain_type         = indexed_domain<size_t, 1>;
  using partition_type      = balanced_partition<domain_type>;

  using loc_linearizer      = identity<size_t>;

  class boundary_rev_linearizer
  {
  private:
    using index_type = problem_domain_t::index_type;

    problem_domain_t m_boundary_domain;
    rev_linear_t     m_rev_linearizer;
    index_type       m_first;

  public:
    boundary_rev_linearizer(problem_domain_t boundary_domain,
                            rev_linear_t rev_linearizer,
                            index_type first)
      : m_boundary_domain(boundary_domain),
        m_rev_linearizer(rev_linearizer),
        m_first(first)
    { }

    size_t operator()(size_t tid) const
    {
      auto idx = m_rev_linearizer(tid);

      size_t steps = 0;

      for (auto val = m_first; val != idx; )
      {
        val = m_boundary_domain.advance(val, 1);
        ++steps;
      }

      return steps;
    }
  };

  using task_id_mapper_type =
    task_mapper<partition_type, boundary_rev_linearizer, loc_linearizer>;

  template<typename ...Views>
  task_id_mapper_type
  get_task_id_mapper(Views const&...) const
  {
    const size_t nlocs      = get_num_locations();
    const size_t num_ports  = m_boundary_domain.size();

    return task_id_mapper_type(
      partition_type(domain_type(num_ports), nlocs),
      boundary_rev_linearizer(
        m_boundary_domain, m_rev_linearizer, m_boundary_domain.first()),
      loc_linearizer());
  }

  scheduler_type get_scheduler(void) const
  { return scheduler_type(); }

  coarsener_type get_coarsener(void) const
  { return null_coarsener(); }

  size_t index(void) const
  { return m_child_index; }

  template<typename PGV>
  void operator()(PGV const& pgv) const
  {
    const auto my_id        = pgv.graph().get_location_id();
    const size_t nlocs      = pgv.graph().get_num_locations();
    const size_t num_ports  = m_boundary_domain.size();

    partition_type partition(domain_type(num_ports), nlocs);

    auto local_domain = partition[my_id];

    if (local_domain.empty())
      return;

    for (auto idx = local_domain.first();
         idx <= local_domain.last();
         idx = local_domain.advance(idx, 1))
    {
      auto val_idx = m_boundary_domain.first();
      val_idx      = m_boundary_domain.advance(val_idx, idx);

      const size_t tid = m_linearizer(val_idx);

      pgv.add_task(tid, boundary_wf(m_flow_width), 1);
      pgv.set_result(tid, tid);
    }
  }

  void define_type(typer& t)
  {
    t.member(m_child_index);
    t.member(m_boundary_domain);
    t.member(m_flow_width);
    t.member(m_linearizer);
    t.member(m_rev_linearizer);
  }
}; // class boundary_factory


template<typename Placement>
struct no_td_fifo_scheduler
  : fifo_scheduler<Placement>
{
  using task_placement_all_local = void;

  template<typename ...Args>
  no_td_fifo_scheduler(Args&&... args)
    : fifo_scheduler<Placement>(std::forward<Args>(args)...)
  { }
};


class wavefront_factory_base
  : public factory_wf
{
protected:
  size_t           m_child_index;
  tuple_t          m_id;
  size_t           m_level;
  problem_domain_t m_problem_domain;
  linear_t         m_linearizer;
  rev_linear_t     m_rev_linearizer;
  pg_dims_vec_t    m_pg_dims_vec;
  tuple_t          m_pg_dims;
  size_t           m_flow_width;
  size_t           m_pipe_depth;

  size_t num_sections(void) const
  { return tuple_ops::fold(m_pg_dims, 1, stapl::multiplies<size_t>()); }

public:
  using scheduler_type = no_td_fifo_scheduler<pg_placement_strategy>;
  using coarsener_type = null_coarsener;

  scheduler_type get_scheduler(void) const
  { return scheduler_type(this->num_sections()); }

  coarsener_type get_coarsener(void) const
  { return null_coarsener(); }

  size_t index(void) const
  { return m_child_index; }

  tuple_t const& pg_dims(void) const
  {
    return m_pg_dims;
  }

  wavefront_factory_base(size_t child_index,
                         tuple_t id,
                         size_t level,
                         problem_domain_t problem_domain,
                         linear_t linearizer,
                         rev_linear_t rev_linearizer,
                         pg_dims_vec_t pg_dims,
                         size_t flow_width,
                         size_t pipe_depth)
    : m_child_index(child_index),
      m_id(id),
      m_level(level),
      m_problem_domain(problem_domain),
      m_linearizer(linearizer),
      m_rev_linearizer(rev_linearizer),
      m_pg_dims_vec(std::move(pg_dims)),
      m_pg_dims(m_pg_dims_vec.back()),
      m_flow_width(flow_width),
      m_pipe_depth(pipe_depth)
  {
    if (m_pg_dims_vec.empty())
      abort("empty vector");

    m_pg_dims_vec.pop_back();
  }

  void define_type(typer& t)
  {
    t.member(m_id);
    t.member(m_level);
    t.member(m_problem_domain);
    t.member(m_linearizer);
    t.member(m_rev_linearizer);
    t.member(m_pg_dims_vec);
    t.member(m_pg_dims);
  }
};


template<size_t Iter, size_t Limit>
struct leaf_task_creator
{
  template<typename PGV, typename ProblemDomain,
           typename Boundaries, typename ...Parameters>
  static void
  apply(PGV& pgv,
        tuple_t const& index,
        ProblemDomain const& problem_domain,
        Boundaries const& boundaries,
        linear_t const& linearizer,
        Parameters&&... params)
  {
    // This is the point in the problem domain I'm consuming from on this
    // boundary.  Its linearization is either the port_id for an input port
    // of the parent paragraph or the task id of a sibling to consume from.
    auto predecessor_idx = index;
    --get<Iter>(predecessor_idx);
    const size_t predecessor_id = linearizer(predecessor_idx);

    // If I'm on the boundary of the problem subdomain assigned to the paragraph
    // spawning this task, read from the appropriate input boundary port.
    if (get<Iter>(index) == get<Iter>(problem_domain.first()))
    {
      get<Iter>(boundaries).assert_perfect_placement(predecessor_id);

      // Each port in each of the input boundary ports is connected to
      // exactly one task.
      get<Iter>(boundaries).set_num_succs(predecessor_id, 1);

      leaf_task_creator<Iter+1, Limit>::apply(
        pgv, index, problem_domain, boundaries, linearizer,
        std::forward<Parameters>(params)...,
        consume<flow_type>(get<Iter>(boundaries), predecessor_id));

      return;
    }

    // I'm reading from another task in current paragraph, set up dataflow
    // using intra-paragraph ports (i.e., pgv)
    leaf_task_creator<Iter+1, Limit>::apply(
      pgv, index, problem_domain, boundaries, linearizer,
      std::forward<Parameters>(params)...,
      consume<flow_type>(pgv, predecessor_id));
  }
}; // struct leaf_task_creator


template<size_t N>
struct leaf_task_creator<N, N>
{
  template<typename PGV, typename ProblemDomain,
           typename Boundaries, typename ...Parameters>
  static void
  apply(PGV& pgv, tuple_t const&, ProblemDomain const&,
        Boundaries const&, linear_t const&,
        Parameters&&... params)
  { pgv.add_task(std::forward<Parameters>(params)...); }
};


template<size_t NDims, typename = make_index_sequence<NDims>>
class leaf_factory;


template<size_t NDims, size_t ...Indices>
class leaf_factory<NDims, index_sequence<Indices...>>
  : public wavefront_factory_base
{
private:
  using traversal_t    = typename default_traversal<NDims>::type;
  using partition_type = multiarray_impl::block_partition<traversal_t>;

public:
  using result_type         = flow_type;
  using domain_type         = problem_domain_t;
  using task_id_mapper_type =
    task_mapper<partition_type, rev_linear_t, linear_t>;

  using wavefront_factory_base::wavefront_factory_base;

  template<typename ...Views>
  task_id_mapper_type
  get_task_id_mapper(Views const&...) const
  {
    return task_id_mapper_type(
      partition_type(m_problem_domain, m_pg_dims),
      m_rev_linearizer,
      linear_t(m_pg_dims));
  }

  template<typename PGV, typename ...Views>
  void operator()(PGV const& pgv, Views const&... views) const
  {
    rev_linear_t loc_rev_linearizer(m_pg_dims);

    const tuple_t loc_idx =
      loc_rev_linearizer(pgv.graph().get_location_id());

    partition_type partition(m_problem_domain, m_pg_dims);

    auto local_domain = partition[loc_idx];

    for (auto idx = local_domain.first();
         idx <= local_domain.last();
         idx = local_domain.advance(idx, 1))
    {
      const size_t linear_tid = m_linearizer(idx);

      const size_t edges_on_out_boundary =
        pack_ops::functional::plus_(
          ((get<Indices>(idx)
           == (get<Indices>(m_problem_domain.last()))) ? 1 : 0)...);

      // Basic case is one successor in each dimension. If this task has more
      // than one edge on the problem boundary, only count as one successor as
      // it just feeds into output port of this paragraph.
      size_t num_succs = NDims;

      if (edges_on_out_boundary > 1)
        num_succs -= edges_on_out_boundary - 1;

      leaf_task_creator<0, NDims>::apply(
        pgv, idx,
        m_problem_domain, tuple<Views const&...>(views...),
        m_linearizer, linear_tid, grind_wf(idx, m_pipe_depth), num_succs);

      // If this task is compute results on the problem domain,
      // wire it into this paragraph's out ports.
      if (edges_on_out_boundary > 0)
        pgv.set_result(linear_tid, linear_tid);
    }
  }
}; // class leaf_factory


class boundary_filter
{
private:
  problem_domain_t m_domain;
  rev_linear_t     m_rev_linearizer;

public:
  boundary_filter(problem_domain_t domain,
                  rev_linear_t rev_linearizer)
    : m_domain(std::move(domain)),
      m_rev_linearizer(rev_linearizer)
  { }

  bool operator()(size_t linear_index) const
  {
    auto index = m_rev_linearizer(linear_index);
    return m_domain.contains(index);
  }

  void define_type(typer& t)
  {
    t.member(m_domain);
    t.member(m_rev_linearizer);
  }
}; // class boundary_filter



template<size_t Iter, size_t Limit>
struct task_creator
{
  template<typename PGV, typename ProblemDomain, typename Boundaries,
           typename Factory, typename ...Parameters>
  static size_t
  apply(PGV& pgv, tuple_t const& idx, ProblemDomain const& problem_domain,
        Boundaries const& boundaries,
        linear_t const& linearizer,
        rev_linear_t const& rev_linearizer,
        tuple_t const& loc_dims, pg_dims_vec_t const& pg_dims,
        size_t tid, Factory const& factory, Parameters&&... params)
  {
    // Define the domain of the boundary condition
    auto first       = problem_domain.first();
    --get<Iter>(first);

    auto last        = problem_domain.last();
    get<Iter>(last)  = get<Iter>(first);

    ProblemDomain boundary_domain(first, last);

    using traversal_t    = typename default_traversal<Limit>::type;
    using partition_type = multiarray_impl::block_partition<traversal_t>;

    auto plane_loc_dims = loc_dims;
    get<Iter>(plane_loc_dims) = 1;

    pin_mapper<Limit> mapper(
      rev_linearizer, pg_dims, partition_type(boundary_domain, plane_loc_dims));

    // If I'm on the boundary of the problem, for this paragraph, read input
    // from the appropriate input port...
    if (get<Iter>(idx) == 0)
    {
      std::vector<size_t> boundary_ids;
      boundary_ids.reserve(boundary_domain.size());

      for (auto boundary_idx = boundary_domain.first();
           boundary_idx <= boundary_domain.last();
           boundary_idx = boundary_domain.advance(boundary_idx, 1))
      {
        boundary_ids.push_back(linearizer(boundary_idx));
      }

      // Add a consume call for this predecessor for the task being created.
      return task_creator<Iter+1, Limit>::apply(
        pgv, idx, problem_domain, boundaries,
        linearizer, rev_linearizer, loc_dims, pg_dims,
        tid, factory,
        std::forward<Parameters>(params)...,
        consume_pg<flow_type>(
          get<Iter>(boundaries),
          std::move(boundary_ids),
          use_default(),
          std::move(mapper))
      );
    }
    else
    {
      // Set predecessor task_id for consume_pg() statement below with the
      // paragraph executing adjacent piece in the problem domain in the
      // given dimension.
      auto predecessor_idx = idx;
      --get<Iter>(predecessor_idx);

      const size_t predecessor_task_id = linearizer(predecessor_idx);

      // Add a consume call for this predecessor for the task being created.
      return task_creator<Iter+1, Limit>::apply(
        pgv, idx, problem_domain, boundaries,
        linearizer, rev_linearizer, loc_dims, pg_dims,
        tid, factory,
        std::forward<Parameters>(params)...,
        consume_pg<flow_type>(
          pgv, predecessor_task_id,
          boundary_filter(boundary_domain, rev_linearizer),
          use_default(),
          pin_mapper<Limit>(rev_linearizer, pg_dims,
                            partition_type(boundary_domain, plane_loc_dims)))
      );
    }
  }
}; // class task_creator


template<size_t N>
struct task_creator<N, N>
{
  template<typename PGV, typename ProblemDomain, typename Boundaries,
           typename Factory, typename ...Parameters>
  static size_t
  apply(PGV& pgv, tuple_t const&, ProblemDomain const&, Boundaries const&,
        linear_t const&, rev_linear_t const&, tuple_t const&,
        pg_dims_vec_t const&, size_t tid, Factory const& factory,
        Parameters&&... params)
  {
    pgv.add_task(tid, factory, std::forward<Parameters>(params)...);
    return 1;
  }
};



template<size_t Iter, size_t Limit>
struct set_num_succ_ifier
{
  template<typename PGV, typename Partition, typename Boundaries,
           typename PGDims>
  static void
  apply(PGV& pgv,
        Partition const& problem_loc_partition,
        Boundaries const& boundaries,
        linear_t const& linearizer,
        PGDims const& pg_dims,
        location_type my_id)
  {
    using domain_t = decltype(problem_loc_partition.domain());

    auto loc_domain = problem_loc_partition.domain();
    auto first      = problem_loc_partition.domain().first();
    auto last       = problem_loc_partition.domain().last();
    get<Iter>(last) = 0;

    domain_t boundary_loc_domain(first, last);

    for (auto loc_idx = boundary_loc_domain.first();
         loc_idx <= boundary_loc_domain.last();
         loc_idx = boundary_loc_domain.advance(loc_idx, 1))
    {
      auto computed_loc =
        recursive_map(pg_dims.rbegin(), pg_dims.rend(), loc_idx, loc_domain);

      if (my_id == computed_loc)
      {
        auto problem_domain = problem_loc_partition[loc_idx];
        auto first          = problem_domain.first();
        --get<Iter>(first);

        auto last           = problem_domain.last();
        get<Iter>(last)     = get<Iter>(first);

        decltype(problem_domain) boundary_domain(first, last);

        using traversal_t    = typename default_traversal<Limit>::type;
        using partition_type = multiarray_impl::block_partition<traversal_t>;

        stapl_assert(get<Iter>(loc_idx) == 0, "What is that");

        for (auto boundary_idx = boundary_domain.first();
             boundary_idx <= boundary_domain.last();
             boundary_idx = boundary_domain.advance(boundary_idx, 1))
        {
          const size_t linear_boundary_id = linearizer(boundary_idx);

          get<Iter>(boundaries).assert_perfect_placement(linear_boundary_id);

          get<Iter>(boundaries).set_num_succs(linear_boundary_id, 1);
        }
      }
    }

    set_num_succ_ifier<Iter + 1, Limit>::apply(
      pgv, problem_loc_partition, boundaries, linearizer, pg_dims, my_id);
  }
}; // class set_num_succ_ifier


template<size_t N>
struct set_num_succ_ifier<N, N>
{
  template<typename PGV, typename Partition, typename Boundaries,
           typename PGDims>
  static void
  apply(PGV& pgv, Partition const&, Boundaries const&,
        linear_t const&, PGDims const&, location_type)
  { }
};


template<size_t NDims, typename = make_index_sequence<NDims>>
class result_mapper;


template<size_t NDims, size_t ...Indices>
class result_mapper<NDims, index_sequence<Indices...>>
{
private:
  problem_domain_t   m_problem_domain;
  rev_linear_t       m_rev_linearizer;

public:
  result_mapper(problem_domain_t problem_domain,
                rev_linear_t rev_linearizer)
    : m_problem_domain(problem_domain),
      m_rev_linearizer(rev_linearizer)
  { }

  bool should_flow(size_t idx) const
  {
    return pack_ops::functional::plus_(
      (get<Indices>(m_problem_domain.last())
       == get<Indices>(m_rev_linearizer(idx)))...);
  }

  size_t operator()(size_t idx) const
  { return idx; }

  void define_type(typer& t)
  {
    t.member(m_problem_domain);
    t.member(m_rev_linearizer);
  }
}; // class result_mapper


template<size_t NDims, typename = make_index_sequence<NDims>>
class recursive_factory;


template<size_t NDims, size_t ...Indices>
class recursive_factory<NDims, index_sequence<Indices...>>
  : public wavefront_factory_base
{
private:
  using traversal_t       = typename default_traversal<NDims>::type;
  using partition_type    = multiarray_impl::block_partition<traversal_t>;
  using part_domain_t     = typename partition_type::domain_type;
  using part_index_t      = typename partition_type::index_type;

  class ident
  {
  public:
    part_index_t find(part_index_t const& idx) const
    { return idx; }
  };

  class loc_linearizer
  {
  private:
    size_t          m_section_size;
    part_domain_t   m_part_domain;

  public:
    loc_linearizer(size_t section_size, part_domain_t part_domain)
      : m_section_size(section_size), m_part_domain(part_domain)
    { }

    size_t operator()(part_index_t const& idx) const
    {
      auto idx2            = m_part_domain.first();
      size_t child_index   = 0;

      for ( ; idx2 != idx; idx2 = m_part_domain.advance(idx2, 1), ++child_index)
      { }

      return child_index * m_section_size;
    }
  };

public:
  using result_type       = flow_type;
  using domain_type       = problem_domain_t;

  using wavefront_factory_base::wavefront_factory_base;

  using task_id_mapper_type =
    task_mapper<ident, rev_linear_t, loc_linearizer>;

  template<typename ...Views>
  task_id_mapper_type
  get_task_id_mapper(Views const&...) const
  {
    const size_t nlocs       = get_num_locations();
    const size_t child_nlocs = nlocs / this->num_sections();

    partition_type p(m_problem_domain, m_pg_dims);

    return task_id_mapper_type(
      ident(), m_rev_linearizer, loc_linearizer(child_nlocs, p.domain()));
  }


  template<typename PGV, typename ...Views>
  void operator()(PGV const& pgv, Views const&... views) const
  {
    const auto my_id         = pgv.graph().get_location_id();
    const size_t nlocs       = pgv.graph().get_num_locations();
    const size_t child_nlocs = nlocs / this->num_sections();

    tuple_t child_loc_dims = m_pg_dims_vec[0];

    for (size_t i=1; i<m_pg_dims_vec.size(); ++i)
      child_loc_dims =
        tuple_t((get<Indices>(child_loc_dims)
                 * get<Indices>(m_pg_dims_vec[i]))...);

    //
    // Call set_num_succs on the appropriate location...
    //
    tuple_t current_loc_dims((get<Indices>(child_loc_dims)
                              * get<Indices>(m_pg_dims))...);

    // The Problem Domain in the "virtual" aka this level plane coordinates
    partition_type loc_partition(m_problem_domain, current_loc_dims);

    auto pg_dims_tmp = m_pg_dims_vec;
    pg_dims_tmp.push_back(m_pg_dims);

    set_num_succ_ifier<0, NDims>::apply(
      pgv, loc_partition, tuple<Views const&...>(views...),
      m_linearizer, pg_dims_tmp, my_id);

    //
    //  Spawn Nested PGs.
    //
    partition_type data_partition(m_problem_domain, m_pg_dims);

    auto&& part_domain = data_partition.domain();

    size_t child_index      = 0;

    size_t num_pgs_created = 0;

    for (auto idx = part_domain.first();
         idx <= part_domain.last();
         idx = part_domain.advance(idx, 1), ++child_index)
    {
      if (my_id == child_index * child_nlocs)
      {
        auto child_problem_domain = data_partition[idx];

        //
        // See how many of my out edges are on the boundary of this nested
        // section (i.e., how many of faces actually feed into this parent's
        // output ports.
        //
        const size_t edges_on_out_boundary =
          pack_ops::functional::plus_(
            ((get<Indices>(child_problem_domain.last())
             == (get<Indices>(m_problem_domain.last()))) ? 1:0)...);

        size_t num_succs = NDims;

        if (edges_on_out_boundary > 1)
          num_succs -= edges_on_out_boundary-1;

        //
        // Call another instance of this factory with restricted problem domain
        // unless the next level is a leaf in the nested paragraph
        // specification (for which we use a leaf_factory).
        //
        if (m_pg_dims_vec.size() > 1)
        {
          recursive_factory f(child_index, idx, m_level+1, child_problem_domain,
                              m_linearizer, m_rev_linearizer, m_pg_dims_vec,
                              m_flow_width, m_pipe_depth);

          num_pgs_created =
            task_creator<0, NDims>::apply(pgv, idx, child_problem_domain,
                                          tuple<Views const&...>(views...),
                                          m_linearizer, m_rev_linearizer,
                                          child_loc_dims, m_pg_dims_vec,
                                          m_linearizer(idx), f, num_succs);
        }
        else
        {
          using leaf_factory_t = leaf_factory<NDims>;

          leaf_factory_t f(child_index, idx, m_level+1, child_problem_domain,
                           m_linearizer, m_rev_linearizer, m_pg_dims_vec,
                           m_flow_width, m_pipe_depth);

          num_pgs_created =
            task_creator<0, NDims>::apply(pgv, idx, child_problem_domain,
                                          tuple<Views const&...>(views...),
                                          m_linearizer, m_rev_linearizer,
                                          child_loc_dims, m_pg_dims_vec,
                                          m_linearizer(idx), f, num_succs);
        }

        if (edges_on_out_boundary > 0)
          pgv.set_result_pg(
            m_linearizer(idx),
            result_mapper<NDims>(m_problem_domain, m_rev_linearizer));
      }
    }

    pgv.add_expected_tasks(1 - num_pgs_created);
  }
}; // class recursive_factory


template<size_t Iter, size_t Limit>
struct task_creator_with_boundary
{
  template<typename PGV, typename Idx, typename ProblemDomain, typename Factory,
           typename ...Parameters>
  static size_t
  apply(PGV& pgv, Idx idx, size_t child_idx,
        ProblemDomain const& problem_domain,
        Factory&& factory,
        linear_t linearizer, rev_linear_t rev_linearizer,
        size_t flow_width, tuple_t pg_dims, tuple_t loc_dims,
        pg_dims_vec_t const& pg_dims_vec, size_t num_succs,
        Parameters&&... params)
  {
    // For the outer factory, if I'm on the outgoing face of the problem domain,
    // don't increment num_succs.
    if (get<Iter>(idx) != (get<Iter>(pg_dims)))
      ++num_succs;

    size_t predecessor_task_id;

    // Define the domain of the boundary condition
    auto first       = problem_domain.first();
    --get<Iter>(first);

    auto last        = problem_domain.last();
    get<Iter>(last)  = get<Iter>(first);

    ProblemDomain boundary_domain(first, last);

    using traversal_t    = typename default_traversal<Limit>::type;
    using partition_type = multiarray_impl::block_partition<traversal_t>;

    auto plane_loc_dims = loc_dims;
    get<Iter>(plane_loc_dims) = 1;


    // If I'm on the boundary of the problem, spawn a simple paragraph for the
    // boundary condition so that we have uniform treatment for dataflow.
    if (get<Iter>(idx) == 1)
    {
      // Adopt policy that TID increment pg_dims in the boundary dimension
      // and then linearizes in the problem domain.
      auto boundary_idx = idx;
      --get<Iter>(boundary_idx);

      const size_t boundary_task_id = linearizer(boundary_idx);

      // Factory is given linear task id (child_idx) of the paragraph that
      // will consume it, so that paragraph placement policy will collocate it.
      // (its index() member reflects this value).
      boundary_factory boundary_factory(child_idx,  boundary_domain,
                                        flow_width, linearizer,
                                        rev_linearizer);

      pgv.add_task(boundary_task_id, std::move(boundary_factory), 1);

      // Set predecessor task_id for consume_pg() statement below
      predecessor_task_id = boundary_task_id;

      // Add a consume call for this predecessor for the task being created.
      return 1 + task_creator_with_boundary<Iter+1, Limit>::apply(
        pgv, idx, child_idx, problem_domain,
        std::forward<Factory>(factory),
        linearizer, rev_linearizer, flow_width, pg_dims, loc_dims, pg_dims_vec,
        num_succs,
        std::forward<Parameters>(params)...,
        consume_pg<flow_type>(
          pgv, predecessor_task_id,
          unconditional_flow(),
          use_default(),
          pin_mapper<Limit>(rev_linearizer, pg_dims_vec,
                            partition_type(boundary_domain, plane_loc_dims)))
      );
    }
    else
    {
      // Set predecessor task_id for consume_pg() statement below with the
      // paragraph executing adjacent piece in the problem domain in the
      // given dimension.
      auto predecessor_idx = idx;
      --get<Iter>(predecessor_idx);

      predecessor_task_id = linearizer(predecessor_idx);

      // Add a consume call for this predecessor for the task being created.
      return task_creator_with_boundary<Iter+1, Limit>::apply(
        pgv, idx, child_idx, problem_domain,
        std::forward<Factory>(factory),
        linearizer, rev_linearizer, flow_width, pg_dims, loc_dims, pg_dims_vec,
        num_succs,
        std::forward<Parameters>(params)...,
        consume_pg<flow_type>(
          pgv, predecessor_task_id,
          boundary_filter(boundary_domain, rev_linearizer),
          use_default(),
          pin_mapper<Limit>(rev_linearizer, pg_dims_vec,
                            partition_type(boundary_domain, plane_loc_dims)))
      );
    }
  }
};


template<size_t N>
struct task_creator_with_boundary<N, N>
{
  template<typename PGV, typename Idx, typename ProblemDomain, typename Factory,
           typename ...Parameters>
  static size_t
  apply(PGV& pgv, Idx idx, size_t child_idx,
        ProblemDomain const& problem_domain,
        Factory&& factory,
        linear_t linearizer, rev_linear_t rev_linearizer,
        size_t flow_width, tuple_t, tuple_t, pg_dims_vec_t const&,
        size_t num_succs,
        Parameters&&... params)
  {
    pgv.add_task(linearizer(idx),
                 std::forward<Factory>(factory),
                 num_succs,
                 std::forward<Parameters>(params)...);
    return 1;
  }
};


template<size_t NDims, typename = make_index_sequence<NDims>>
class outer_factory;


template<size_t NDims, size_t ...Indices>
class outer_factory<NDims, index_sequence<Indices...>>
  : public wavefront_factory_base
{
private:
  using traversal_t    = typename default_traversal<NDims>::type;
  using partition_type = multiarray_impl::block_partition<traversal_t>;
  using part_domain_t  = typename partition_type::domain_type;
  using part_index_t   = typename partition_type::index_type;

  class ident
  {
  public:
    part_index_t find(part_index_t const& idx) const
    { return idx; }
  };

  class loc_linearizer
  {
  private:
    size_t          m_section_size;
    part_domain_t   m_part_domain;

  public:
    loc_linearizer(size_t section_size, part_domain_t part_domain)
      : m_section_size(section_size), m_part_domain(part_domain)
    { }

    size_t operator()(part_index_t const& idx) const
    {
      stapl_assert(
        pack_ops::functional::plus_((get<Indices>(idx) == 0 ? 1 : 0)...) <= 1,
        "unexpected boundary index");

      part_index_t adj_idx(
        (get<Indices>(idx) == 0 ? 0 : (get<Indices>(idx) - 1))...);

      auto idx2            = m_part_domain.first();
      size_t child_index   = 0;

      for ( ; idx2 != adj_idx; idx2 = m_part_domain.advance(idx2, 1),
           ++child_index)
      { }

      return child_index * m_section_size;
    }
  };

public:
  using wavefront_factory_base::wavefront_factory_base;
  using result_type = void;

  using task_id_mapper_type =
    task_mapper<ident, rev_linear_t, loc_linearizer>;

  template<typename ...Views>
  task_id_mapper_type
  get_task_id_mapper(Views const&...) const
  {
    const size_t nlocs       = get_num_locations();
    const size_t child_nlocs = nlocs / this->num_sections();

    partition_type p(m_problem_domain, m_pg_dims);

    return task_id_mapper_type(
      ident(), m_rev_linearizer, loc_linearizer(child_nlocs, p.domain()));
  }

  template<typename PGV>
  void operator()(PGV const& pgv) const
  {
    tuple_t child_loc_dims = m_pg_dims_vec[0];

    for (size_t i=1; i<m_pg_dims_vec.size(); ++i)
      child_loc_dims =
        tuple_t((get<Indices>(child_loc_dims)
                 * get<Indices>(m_pg_dims_vec[i]))...);

    const auto my_id         = pgv.graph().get_location_id();
    const size_t nlocs       = pgv.graph().get_num_locations();

    const size_t child_nlocs = nlocs / this->num_sections();

    partition_type data_partition(m_problem_domain, m_pg_dims);

    auto&& part_domain = data_partition.domain();

    size_t child_index = 0;

    for (auto idx = part_domain.first();
         idx <= part_domain.last();
         idx = part_domain.advance(idx, 1), ++child_index)
    {
      if (my_id == child_index * child_nlocs)
      {
        auto child_problem_domain = data_partition[idx];

        if (m_pg_dims_vec.size() > 1)
        {
          using factory_t = recursive_factory<NDims>;

          factory_t f(child_index, idx, m_level+1, child_problem_domain,
                      m_linearizer, m_rev_linearizer, m_pg_dims_vec,
                      m_flow_width, m_pipe_depth);

          task_creator_with_boundary<0, NDims>::apply(
            pgv, tuple_t((get<Indices>(idx) + 1)...), child_index,
            child_problem_domain, f,
            m_linearizer, m_rev_linearizer, m_flow_width, m_pg_dims,
            child_loc_dims, m_pg_dims_vec, 0);
        }
        else
        {
          using leaf_factory_t = leaf_factory<NDims>;

          leaf_factory_t f(child_index, idx, m_level+1, child_problem_domain,
                           m_linearizer, m_rev_linearizer, m_pg_dims_vec,
                           m_flow_width, m_pipe_depth);

          task_creator_with_boundary<0, NDims>::apply(
            pgv, tuple_t((get<Indices>(idx) + 1)...), child_index,
            child_problem_domain, f,
            m_linearizer, m_rev_linearizer, m_flow_width, m_pg_dims,
            child_loc_dims,  m_pg_dims_vec, 0);
        }
      }

      if (my_id > child_index * child_nlocs
          && my_id < (child_index+1)*child_nlocs)
      {
        size_t cnt =
          1 + pack_ops::functional::plus_((get<Indices>(idx) == 0 ? 1 : 0)...);

        pgv.add_expected_tasks(cnt);
      }
    }
  }
};


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

      size_t n_iters = 32;

      for (size_t iter = 0; iter < n_iters; ++iter)
        for (size_t i = 0; i < overpartition_factor; ++i)
          wf(boundaries[Indices]...);

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
  if (argc < 5)
    abort(
      "./pg_p2p_data_flow_nD num_elem factor pipe_depth pg_dims0 [pg_dims1] ..."
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

  const auto problem_dims =
    tuple_ops::transform(
      tuple_t(), [&](size_t) { return dim_size * overpartition_factor; });

  do_once([&]()
  {
    std::cout << ndims << "D Wavefront, processor dimensions = ";

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

  const size_t num_sections =
    tuple_ops::fold(pg_dims.back(), 1, stapl::multiplies<size_t>());

  auto first_problem_idx =
    tuple_ops::transform(tuple_t(), [](size_t) { return 1; });

  auto linear_dims =
    tuple_ops::transform(problem_dims, [](size_t v) { return v + 1; });

  linear_t     linearizer(linear_dims);
  rev_linear_t rev_linearizer(linear_dims);

  problem_domain_t problem_domain(first_problem_idx, problem_dims);

  using factory_t = outer_factory<ndims>;

  flow_type boundary;

  using scheduler_t = no_td_fifo_scheduler<pg_placement_strategy>;
  using paragraph_t = paragraph<scheduler_t, factory_t>;

  for (int i = 0; i < 32; ++i)
  {
    timer_type benchmark_timer;
    benchmark_timer.start();
    factory_t f(0, tuple_t(), 0, problem_domain,
                linearizer, rev_linearizer, pg_dims,
                flow_width, pipe_depth);

    scheduler_t sched(num_sections);
    paragraph_t pg(f, sched);
    pg();

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

