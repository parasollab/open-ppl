/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_PARTITIONERS_ARBITRARY_MESH_DECOMPOSITION_HPP
#define STAPL_CONTAINERS_GRAPH_PARTITIONERS_ARBITRARY_MESH_DECOMPOSITION_HPP

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/partitioners/gpartition.h>
#include <stapl/containers/graph/mesh/geom_vector.hpp>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/balance_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/sorting.hpp>
#include <stapl/utility/tuple.hpp>
#include <utility>
#include <tools/libstdc++/proxy/pair.h>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to project a cell centroid on an axis.
/// @tparam Dim dimension of the mesh.
/// @tparam Precision type of centroid coordinates.
//////////////////////////////////////////////////////////////////////
template <int Dim, typename Precision = double>
struct project_cells
{
private:
  geom_vector<Dim, Precision> m_axis;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param axis axis used in the projection.
  //////////////////////////////////////////////////////////////////////
  project_cells(geom_vector<Dim, Precision> const& axis)
    : m_axis(axis)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param cell mesh cell.
  /// @param proj pair storing the computed projection and the associated
  /// cell id.
  //////////////////////////////////////////////////////////////////////
  template <typename Cell, typename Proj>
  void operator()(Cell cell, Proj proj) const
  {
    proj = std::make_pair(m_axis.dot(cell.property().get_centroid()),
                          cell.descriptor());
  }

  void define_type(typer &t)
  {
    t.member(m_axis);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compute the cell domains of mesh partitions.
//////////////////////////////////////////////////////////////////////
struct get_partition_domains
{
  typedef std::vector<std::vector<size_t> > result_type;
private:
  size_t m_num_parts;
  size_t m_mesh_size;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param num_parts number of partitions.
  /// @param mesh_size number of cells in the mesh.
  //////////////////////////////////////////////////////////////////////
  get_partition_domains(size_t const& num_parts, size_t const& mesh_size)
    : m_num_parts(num_parts), m_mesh_size(mesh_size)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param part a mesh partitions.
  /// @param index_part indices of the cell partitions.
  /// @return the cell domains of the mesh partitions.
  //////////////////////////////////////////////////////////////////////
  template <typename Partition1, typename Partition2>
  result_type operator()(Partition1 part, Partition2 index_part) const
  {
    result_type doms(m_num_parts);
    size_t rem      = m_mesh_size % m_num_parts;
    size_t bszl     = m_mesh_size / m_num_parts;
    size_t bszg     = bszl + (rem > 0 ? 1 : 0);

    typename Partition2::iterator index_it = index_part.begin();
    typename Partition1::iterator it = part.begin(),
                                 end_it = part.end();
    for (; it != end_it; ++it, ++index_it)
    {
      size_t index = *index_it;
      size_t i = (index < rem*bszg) ? index/bszg :
                                      rem + (index - rem * bszg)/bszl;
      doms[i].push_back((*it).second);
    }
    return doms;
  }

  void define_type(typer &t)
  {
    t.member(m_num_parts);
    t.member(m_mesh_size);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Convert a vector to a domset1D domain.
/// @param dom vector storing identifiers.
/// @return domset1D domain of the vector.
//////////////////////////////////////////////////////////////////////
template <typename T>
domset1D<T> convert_to_domset1d(std::vector<T>& dom)
{
  domset1D<T> result;
  std::sort(dom.begin(), dom.end());
  size_t dom_size = dom.size();
  size_t start = dom[0];
  for (size_t j=1; j < dom_size; ++j)
  {
    if (dom[j] != dom[j-1] + 1)
    {
      result += domset1D<T>(start, dom[j-1]);
      start = dom[j];
    }
  }
  result += domset1D<T>(start, dom[dom_size-1]);
  return result;
}


//////////////////////////////////////////////////////////////////////
/// @brief Partition a mesh along one dimension.
/// @tparam Dim mesh dimension.
/// @tparam GView type of mesh view.
/// @tparam Precision type of mesh vertex coordinates.
//////////////////////////////////////////////////////////////////////
template<int Dim, typename GView, typename Precision = double>
struct mesh_partition_1d
{
  typedef std::vector<size_t> domain_type;
  typedef std::vector<domain_type> result_type;
  typedef typename geom_vector<Dim, Precision>::element_type element_type;
  typedef std::pair<element_type, size_t> projection_pair_type;

private:
  GView m_graph_vw;

  //////////////////////////////////////////////////////////////////////
  /// @brief Reduce partition domains.
  /// @tparam T type of cell ids.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  struct reduce_vector_wf
  {
    typedef std::vector<std::vector<T> > result_type;
    //////////////////////////////////////////////////////////////////////
    /// @param elt1 first partition domain.
    /// @param elt2 second partition domain.
    //////////////////////////////////////////////////////////////////////
    template<typename Element1, typename Element2>
    result_type operator()(Element1 const& elt1, Element2 const& elt2)
    {
      result_type result = elt1;
      typename result_type::iterator res_it = result.begin(),
                                     end_rest_it = result.end();
      typename Element2::const_iterator it = elt2.begin();
      for (; res_it != end_rest_it; ++res_it, ++it)
      {
        std::vector<T> dom = (*it);
        std::copy(dom.begin(), dom.end(), std::back_inserter(*res_it));
      }
      return result;
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Projection comparator. Projection values are sorted in
  /// increasing order.
  //////////////////////////////////////////////////////////////////////
  struct projection_comparator
  {
    //////////////////////////////////////////////////////////////////////
    /// @param p1 first projection value.
    /// @param p2 second projection value.
    //////////////////////////////////////////////////////////////////////
    bool operator()(projection_pair_type const& p1,
                    projection_pair_type const& p2) const
    {
      return p1.first < p2.first;
    }
  };

public:
  //////////////////////////////////////////////////////////////////////
  /// @param graph_vw mesh view to be partitioned.
  //////////////////////////////////////////////////////////////////////
  mesh_partition_1d(GView const& graph_vw)
    : m_graph_vw(graph_vw)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param num_partitions number of partitions.
  /// @param axis axis for projecting cells on one dimension.
  //////////////////////////////////////////////////////////////////////
  result_type operator()(size_t const& num_partitions,
                         geom_vector<Dim, Precision> const& axis) const
  {
    typedef static_array<projection_pair_type> array_type;
    typedef array_view<array_type> array_view_type;

    size_t num_locs = get_num_locations();
    size_t num_cells = m_graph_vw.size();
    array_type projection_array(num_cells);
    array_view_type projection_view(projection_array);

    //Projection of the centroids onto one specific axis
    map_func(project_cells<Dim, Precision>(axis), m_graph_vw, projection_view);

    //sort array by increasing centroids' projection
    sort(projection_view, projection_comparator());

    //Compute the partition domains
    std::vector<std::vector<size_t> >
       domains = map_reduce(get_partition_domains(num_partitions, num_cells),
                            reduce_vector_wf<size_t>(),
                            native_view(projection_view),
                            balance_view(counting_view<size_t>(num_cells),
                            num_locs));
     return domains;


  }

  void define_type(typer &t)
  {
    t.member(m_graph_vw);
  }

};


//////////////////////////////////////////////////////////////////////
/// @brief Specify types of tuple variables used in the decomposition.
///        The type depends on the number of projections.
/// @tparam Dim mesh dimension.
/// @tparam NumProjection number of projections for partitioning.
/// @tparam Precision type of mesh vertex coordinates.
//////////////////////////////////////////////////////////////////////
template<int Dim, int NumProjection, typename Precision = double>
struct decomposition_tuple_type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specify types of tuple variables used in the decomposition.
///        The type depends on the number of projections.
/// @tparam Dim mesh dimension.
/// @tparam Precision type of mesh vertex coordinates.
//////////////////////////////////////////////////////////////////////
template<int Dim, typename Precision>
struct decomposition_tuple_type<Dim, 1, Precision>
{
  typedef tuple<size_t> num_part_type;
  typedef tuple<geom_vector<Dim, Precision> > axis_type;
  typedef std::vector<std::vector<size_t> > domain_sequence;
  typedef tuple<domain_sequence> domain_tuple_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specify types of tuple variables used in the decomposition.
///        The type depends on the number of projections.
/// @tparam Dim mesh dimension.
/// @tparam Precision type of mesh vertex coordinates.
//////////////////////////////////////////////////////////////////////
template<int Dim, typename Precision>
struct decomposition_tuple_type<Dim, 2, Precision>
{
  typedef tuple<size_t, size_t> num_part_type;
  typedef tuple<geom_vector<Dim, Precision>,
                geom_vector<Dim, Precision> > axis_type;
  typedef std::vector<std::vector<size_t> > domain_sequence;
  typedef tuple<domain_sequence, domain_sequence> domain_tuple_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specify types of tuple variables used in the decomposition.
///        The type depends on the number of projections.
/// @tparam Dim mesh dimension.
/// @tparam Precision type of mesh vertex coordinates.
//////////////////////////////////////////////////////////////////////
template<int Dim, typename Precision>
struct decomposition_tuple_type<Dim, 3, Precision>
{
  typedef tuple<size_t, size_t, size_t> num_part_type;
  typedef tuple<geom_vector<Dim, Precision>,
                geom_vector<Dim, Precision>,
                geom_vector<Dim, Precision> > axis_type;
  typedef std::vector<std::vector<size_t> > domain_sequence;
  typedef tuple<domain_sequence, domain_sequence,
                domain_sequence> domain_tuple_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to compute a final partition after the partitioning
/// of each dimension.
/// @tparam Dim mesh dimension.
/// @tparam NumProjection number of projections for partitioning.
/// @tparam Precision type of mesh vertex coordinates.
//////////////////////////////////////////////////////////////////////
template<int Dim, int NumProjection, typename Precision = double>
struct set_kba_like_partition_domains
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Functor to compute a final partition after the partitioning
/// of each dimension.
/// @tparam Dim mesh dimension.
/// @tparam Precision type of mesh vertex coordinates.
//////////////////////////////////////////////////////////////////////
template<int Dim, typename Precision>
struct set_kba_like_partition_domains<Dim, 1, Precision>
{
  typedef void result_type;
  typedef typename decomposition_tuple_type<
                   Dim,1,Precision>::num_part_type num_part_type;
  typedef typename decomposition_tuple_type<
                   Dim,1, Precision>::domain_tuple_type domain_tuple_type;
private:
  num_part_type     m_num_parts;
  domain_tuple_type m_doms;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param num_parts number of partitions.
  /// @param doms domains resulting from the partitioning of the dimensions.
  //////////////////////////////////////////////////////////////////////
  set_kba_like_partition_domains(num_part_type const& num_parts,
                                 domain_tuple_type const& doms)
    : m_num_parts(num_parts), m_doms(doms)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param svertex storing a mesh partition.
  //////////////////////////////////////////////////////////////////////
  template <typename Svertex>
  void operator()(Svertex svertex)
  {
    svertex.property().set_domain(
      convert_to_domset1d(get<0>(m_doms)[svertex.descriptor()]));
  }

  void define_type(typer &t)
  {
    t.member(m_num_parts);
    t.member(m_doms);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to compute a final partition after the partitioning
/// of each dimension.
/// @tparam Dim mesh dimension.
/// @tparam Precision type of mesh vertex coordinates.
//////////////////////////////////////////////////////////////////////
template<int Dim, typename Precision>
struct set_kba_like_partition_domains<Dim, 2, Precision>
{
  typedef void result_type;
  typedef typename decomposition_tuple_type<
                   Dim,2, Precision>::num_part_type num_part_type;
  typedef typename decomposition_tuple_type<
                   Dim,2, Precision>::domain_tuple_type domain_tuple_type;
private:
  num_part_type     m_num_parts;
  domain_tuple_type m_doms;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param num_parts number of partitions.
  /// @param doms domains resulting from the partitioning of the dimensions.
  //////////////////////////////////////////////////////////////////////
  set_kba_like_partition_domains(num_part_type const& num_parts,
                                 domain_tuple_type const& doms)
    : m_num_parts(num_parts), m_doms(doms)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param svertex storing a mesh partition.
  //////////////////////////////////////////////////////////////////////
  template <typename Svertex>
  void operator()(Svertex svertex)
  {
    size_t partition_id = svertex.descriptor();
    size_t x_axis_index = partition_id % get<0>(m_num_parts);
    size_t y_axis_index = partition_id / get<0>(m_num_parts);
    svertex.property().set_domain(
      convert_to_domset1d(get<0>(m_doms)[x_axis_index]) &
      convert_to_domset1d(get<1>(m_doms)[y_axis_index]));
  }

  void define_type(typer &t)
  {
    t.member(m_num_parts);
    t.member(m_doms);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to compute a final partition after the partitioning
/// of each dimension.
/// @tparam Dim mesh dimension.
/// @tparam Precision type of mesh vertex coordinates.
//////////////////////////////////////////////////////////////////////
template<int Dim, typename Precision>
struct set_kba_like_partition_domains<Dim, 3, Precision>
{
  typedef void result_type;
  typedef typename decomposition_tuple_type<
                   Dim,3, Precision>::num_part_type num_part_type;
  typedef typename decomposition_tuple_type<
                   Dim,3, Precision>::domain_tuple_type domain_tuple_type;
private:
  num_part_type     m_num_parts;
  domain_tuple_type m_doms;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param num_parts number of partitions.
  /// @param doms domains resulting from the partitioning of the dimensions.
  //////////////////////////////////////////////////////////////////////
  set_kba_like_partition_domains(num_part_type const& num_parts,
                                 domain_tuple_type const& doms)
    : m_num_parts(num_parts), m_doms(doms)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param svertex storing a mesh partition.
  //////////////////////////////////////////////////////////////////////
  template <typename Svertex>
  void operator()(Svertex svertex)
  {
    size_t partition_id = svertex.descriptor();
    size_t x_axis_index = (partition_id %
                          (get<0>(m_num_parts)*get<1>(m_num_parts))) %
                          get<0>(m_num_parts);
    size_t y_axis_index = (partition_id %
                          (get<0>(m_num_parts)*get<1>(m_num_parts))) /
                          get<0>(m_num_parts);
    size_t z_axis_index = partition_id /
                          (get<0>(m_num_parts)*get<1>(m_num_parts));
    svertex.property().set_domain(
                          convert_to_domset1d(get<0>(m_doms)[x_axis_index]) &
                          convert_to_domset1d(get<1>(m_doms)[y_axis_index]) &
                          convert_to_domset1d(get<2>(m_doms)[z_axis_index]));
  }

  void define_type(typer &t)
  {
    t.member(m_num_parts);
    t.member(m_doms);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor partitioning an arbitrary mesh by projecting its cells
/// on axes and then calling a 1D balanced partitioner.
/// @tparam Dim mesh dimension.
/// @tparam NumProjection number of projections for partitioning.
/// @tparam Precision type of mesh vertex coordinates.
/// @bug The implementation of this partitioner needs to be changed. The
/// current implementation can lead to unbalanced partitions. After
/// partitioning the mesh along an axis, each created partition needed to be
/// recursively partitioned along the other axes to get the final partitions.
//////////////////////////////////////////////////////////////////////
template<int Dim, int NumProjection, typename Precision = double>
class arbitrary_mesh_collapser
{
public:
  typedef typename decomposition_tuple_type<
                   Dim, NumProjection, Precision>::num_part_type num_part_type;
  typedef typename decomposition_tuple_type<
                   Dim, NumProjection, Precision>::axis_type axis_type;
  typedef typename decomposition_tuple_type<
                   Dim, NumProjection, Precision>::domain_tuple_type
                   domain_tuple_type;
private:
  num_part_type m_num_parts;
  axis_type m_axis;

public:
  typedef domset1D<size_t> domain_type;
  typedef typename geom_vector<Dim, Precision>::element_type element_type;
  typedef std::pair<element_type, size_t> projection_pair_type;

  //////////////////////////////////////////////////////////////////////
  /// @param num_partitions tuple of numbers of partitions in each dimension.
  /// @param axis tuple of axes for projections (one axis per dimension).
  //////////////////////////////////////////////////////////////////////
  arbitrary_mesh_collapser(num_part_type const& num_partitions,
                           axis_type const& axis)
    : m_num_parts(num_partitions), m_axis(axis)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @param graph_vw mesh view to be partitioned.
  //////////////////////////////////////////////////////////////////////
  template<class GView>
  gpartition<GView> operator()(GView const& graph_vw) const
  {
    typedef static_array<projection_pair_type> array_type;
    typedef array_view<array_type> array_view_type;
    typedef typename gpartition<GView>::partition_view_t Hview;

    size_t total_partitions = tuple_ops::fold(m_num_parts, 1,
                                              multiplies<size_t>());
    Hview hview = create_partition_view(graph_vw, total_partitions);

    domain_tuple_type doms;
    doms = tuple_ops::transform(m_num_parts, m_axis, mesh_partition_1d<
     Dim, GView, Precision>(graph_vw));

    map_func(set_kba_like_partition_domains<
     Dim, NumProjection, Precision>(m_num_parts, doms), hview);

    return gpartition<GView>(hview);
  }

  void define_type(typer &t)
  {
    t.member(m_num_parts);
    t.member(m_axis);
  }
};



} //namespace stapl

#endif
