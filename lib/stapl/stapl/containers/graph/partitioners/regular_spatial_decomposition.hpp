/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_PARTITIONERS_MESH_REGULAR_SPATIAL_DECOMPOSITION_HPP
#define STAPL_CONTAINERS_GRAPH_PARTITIONERS_MESH_REGULAR_SPATIAL_DECOMPOSITION_HPP

#include <stapl/containers/graph/mesh/implicit_regular_mesh_domain.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>

#include <stapl/containers/graph/partitioners/gpartition.h>
#include <stapl/containers/graph/partitioners/graph_partitioner_utils.hpp>

#include <stapl/utility/tuple.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor checking if a gid tuple is invalid.
/// @ingroup pgraphPartitioner
/// @tparam N last index of a gid tuple.
//////////////////////////////////////////////////////////////////////
template<int N>
struct is_invalid_gid
{
  typedef size_t gid_type;

  //////////////////////////////////////////////////////////////////////
  /// @param t gid tuple.
  /// @return true if invalid
  //////////////////////////////////////////////////////////////////////
  template<class TUPLE>
  bool operator()(TUPLE const& t) const
  {
    return ( (get<N>(t) == index_bounds<gid_type>::invalid()) ||
             is_invalid_gid<N-1>()(t) );
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor checking if a gid tuple is invalid.
/// @ingroup pgraphPartitioner
/// @tparam N last index of a gid tuple.
///
/// Specialization to stop recursion when tuple index is -1
//////////////////////////////////////////////////////////////////////
template<>
struct is_invalid_gid<-1>
{
  //////////////////////////////////////////////////////////////////////
  /// @param t gid tuple.
  /// @return true if invalid
  //////////////////////////////////////////////////////////////////////
  template<class TUPLE>
  bool operator()(TUPLE const& t) const
  {
    return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Partition a domain representing a regular mesh.
/// @ingroup pgraphPartitioner
/// @tparam Dom regular mesh domain type.
//////////////////////////////////////////////////////////////////////
template <typename Dom>
struct spatial_regular_domain_decompose
{
  typedef Dom                                   domain_type;
  typedef typename domain_type::gid_type        gid_type;
  typedef typename domain_type::tuple_type      tuple_type;

private:
  domain_type  m_domain;
  tuple_type   m_nparts;      // number of partitions in each dimension

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the first gid of the ith partition of
  ///        the balanced partitioning of a contiguous indexed domain.
  //////////////////////////////////////////////////////////////////////
  struct spatial_1d_decompose_first_elt
  {
    typedef gid_type result_type;

    //////////////////////////////////////////////////////////////////////
    /// @param triple (size, num partitions, partition id).
    /// @return gid
    //////////////////////////////////////////////////////////////////////
    template<class TUPLE>
    gid_type operator()(TUPLE const& triple) const
    {
      typedef indexed_domain<gid_type> indexed_domain_type;

      const size_t size     = get<0>(triple);
      const size_t num_part = get<1>(triple);
      const size_t part_id  = get<2>(triple);

      return balanced_partition<indexed_domain_type>(indexed_domain_type(size),
                                                     num_part)[part_id].first();
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the last gid of the ith partition of
  ///        the balanced partitioning of a contiguous indexed domain.
  //////////////////////////////////////////////////////////////////////
  struct spatial_1d_decompose_last_elt
  {
    typedef gid_type result_type;

    //////////////////////////////////////////////////////////////////////
    /// @param triple (size, num partitions, partition id).
    /// @return gid
    //////////////////////////////////////////////////////////////////////
    template<class TUPLE>
    gid_type operator()(TUPLE const& triple) const
    {
      typedef indexed_domain<gid_type> indexed_domain_type;
      size_t size = get<0>(triple);
      size_t num_part = get<1>(triple);
      size_t part_id = get<2>(triple);

      return balanced_partition<indexed_domain_type>(indexed_domain_type(size),
                                                     num_part)[part_id].last();
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the minimum size of two sizes
  //////////////////////////////////////////////////////////////////////
  struct calculate_real_size
  {
    typedef gid_type result_type;
    gid_type operator()(gid_type const& size, gid_type const& num_part) const
    {
      return (size < num_part) ? size : num_part;
    }
  };

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Default constructor.
  /// @param dom regular mesh domain to partition.
  /// @param num_partitions number of partitions in each dimension.
  //////////////////////////////////////////////////////////////////////
  spatial_regular_domain_decompose(domain_type const& dom,
                                   tuple_type const& num_partitions)
   : m_domain(dom), m_nparts(num_partitions)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Take a partition index and return the corresponding
  ///        partitioned domain
  /// @param idx partition index
  /// @return domain representing the partition idx
  //////////////////////////////////////////////////////////////////////
  domain_type operator[](size_t idx) const
  {
    stapl_assert(idx < tuple_ops::fold(m_nparts, 1, multiplies<gid_type>()),
                 "partition index out of bound\n");

    implicit_regular_mesh_domain<tuple_type> partition_dom(m_nparts);
    tuple_type part_id = partition_dom.reverse_linearize(
                            partition_dom.advance(partition_dom.first(), idx));
    tuple_type first, last;

    first = tuple_ops::transform(
      tuple_ops::zip(m_domain.get_local_sizes(), m_nparts, part_id),
      spatial_1d_decompose_first_elt()
    );

    if (is_invalid_gid<tuple_size<tuple_type>::value-1>()(first))
      return domain_type();

    last = tuple_ops::transform(
      tuple_ops::zip(m_domain.get_local_sizes(), m_nparts, part_id),
      spatial_1d_decompose_last_elt()
    );

    return domain_type(first, last, m_domain);
  }

  size_t size() const
  {
    tuple_type sizes = tuple_ops::transform(
      m_domain.get_local_sizes(), m_nparts, calculate_real_size()
    );

    return tuple_ops::fold(sizes, 1, multiplies<gid_type>());
  }
};


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor returning the element passed in constructor.
/// @ingroup pgraphPartitioner
/// @todo Use general identity
//////////////////////////////////////////////////////////////////////
template<typename T>
class kba_identity
{
private:
  T const& m_t;

public:
  typedef T result_type;

  kba_identity(T const& t)
   : m_t(t)
  { }

  template<typename Q>
  T const& operator()(Q const&) const
  {
    return m_t;
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Create the KBA partition layout using the number of locations.
/// @ingroup pgraphPartitioner
/// @return KBA partition layout.
//////////////////////////////////////////////////////////////////////
template<int Dim>
typename size_tuple<Dim>::type KBA()
{
  const size_t n             = get_num_locations();
  const double root_degree   = Dim - 1;
  const size_t decomp_value  = std::pow(n, 1.0 / root_degree);

  typedef typename size_tuple<Dim>::type tuple_t;

  tuple_t decomp = tuple_ops::transform(
    tuple_t(), detail::kba_identity<size_t>(decomp_value)
  );

  get<Dim-1>(decomp) = 1;

  return decomp;
}


//////////////////////////////////////////////////////////////////////
/// @brief Create the KBA partition layout using @p num_x partitions
///        on the x axis.
/// @ingroup pgraphPartitioner
/// @param num_x number of partitions on the x axis.
/// @return KBA partition layout.
//////////////////////////////////////////////////////////////////////
size_tuple<2>::type KBA(size_t const& num_x)
{
  return make_tuple(num_x, 1);
}


//////////////////////////////////////////////////////////////////////
/// @brief Create the KBA partition layout using @p num_x partitions
///        on the x axis and @p num_y partitions on the y axis.
/// @ingroup pgraphPartitioner
/// @param num_x number of partitions on the x axis.
/// @param num_y number of partitions on the y axis.
/// @return KBA partition layout.
//////////////////////////////////////////////////////////////////////
size_tuple<3>::type KBA(size_t const& num_x, size_t const& num_y)
{
  return make_tuple(num_x, num_y, 1);
}


//////////////////////////////////////////////////////////////////////
/// @brief Functor setting the children domain of a super-vertex of a
///        hierarchical view.
/// @ingroup pgraphPartitioner
//////////////////////////////////////////////////////////////////////
template<typename Dom>
struct set_supervertex_domain
{
  typedef void result_type;

private:
  Dom m_dom;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param d super-vertex children domain.
  //////////////////////////////////////////////////////////////////////
  set_supervertex_domain(Dom const& d)
   : m_dom(d)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param sv super-vertex reference.
  //////////////////////////////////////////////////////////////////////
  template<typename Property>
  void operator()(Property& sv) const
  {
    sv.set_domain(m_dom);
  }

  void define_type(typer& t)
  {
    t.member(m_dom);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Collapser decomposing a regular mesh into @p decomp partitions.
/// @ingroup pgraphPartitioner
//////////////////////////////////////////////////////////////////////
template<int Dim>
struct spatial_decomposition
{
  typedef typename size_tuple<Dim>::type tuple_type;

private:
  tuple_type m_decomposition;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param decomp number of partitions in each dimension.
  //////////////////////////////////////////////////////////////////////
  spatial_decomposition(tuple_type const& decomp)
   : m_decomposition(decomp)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param graph_vw regular mesh view.
  /// @return partition object.
  //////////////////////////////////////////////////////////////////////
  template<class GView>
  gpartition<GView> operator()(GView const& graph_vw) const
  {
    typedef typename GView::domain_type                domain_type;
    typedef typename partition_view_type<GView>::type  partition_view_type;

    spatial_regular_domain_decompose<domain_type>
      decomposed_doms(graph_vw.domain(), m_decomposition);

    size_t num_partitions = decomposed_doms.size();
    size_t num_locs       = get_num_locations();
    size_t myid           = get_location_id();

    partition_view_type partition_view =
      create_partition_view(graph_vw, num_partitions);

    balanced_partition<indexed_domain<size_t> >
      loc_doms(indexed_domain<size_t>(num_partitions), num_locs);
    indexed_domain<size_t> mydom = loc_doms[myid];

    for (size_t i=0; i<mydom.size(); ++i)
    {
      size_t partition_id = mydom.advance(mydom.first(), i);

      partition_view.vp_apply_async(
        partition_id,
        set_supervertex_domain<domain_type>(decomposed_doms[partition_id])
      );
    }

    rmi_fence(); // for vp_apply_asyncs

    return gpartition<GView>(partition_view);
  }

  void define_type(typer &t)
  {
    t.member(m_decomposition);
  }
};

} //namespace stapl

#endif /* STAPL_CONTAINERS_GRAPH_PARTITIONERS_MESH_REGULAR_\
          SPATIAL_DECOMPOSITION_HPP */
