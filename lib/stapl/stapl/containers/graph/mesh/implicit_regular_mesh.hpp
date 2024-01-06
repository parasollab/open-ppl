/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_MESH_IMPLICIT_REGULAR_MESH_HPP
#define STAPL_CONTAINERS_GRAPH_MESH_IMPLICIT_REGULAR_MESH_HPP

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/partitioners/regular_spatial_decomposition.hpp>
#include <stapl/containers/sequential/graph/graph_util.h>
#include <stapl/containers/sequential/graph/adj_list_vertex_edge.h>
#include <stapl/utility/tuple.hpp>
#include <utility>
#include <stapl/containers/graph/mesh/implicit_regular_mesh_domain.hpp>
#include <stapl/containers/graph/mesh/geom_vector.hpp>
#include <boost/iterator/counting_iterator.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/views/metadata/extraction/generic.hpp>
#include <stapl/containers/iterators/container_iterator.hpp>
#include <stapl/containers/iterators/container_accessor.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Cell face of a regular mesh.
/// @tparam Dim mesh dimension.
/// @todo put in detail namespace.
//////////////////////////////////////////////////////////////////////
template<int Dim>
class regular_face_property
{
public:
  typedef geom_vector<Dim, double>   gvector_t;
  typedef gvector_t                  normal_type;

private:
  normal_type m_normal;

public:
  regular_face_property(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @param norm cell face normal.
  //////////////////////////////////////////////////////////////////////
  regular_face_property(normal_type const& norm)
    : m_normal(norm)
  { }

  normal_type get_normal(void)
  { return m_normal; }

  void define_type(typer& t)
  { t.member(m_normal); }

}; // class regular_face_property


//////////////////////////////////////////////////////////////////////
/// @brief Functor that computes the index of a neighboring cell and
///        the direction in which it lies from the cell provided.
/// @tparam DOM mesh domain type.
/// @tparam N mesh dimension.
/// @todo put in detail namespace.
//////////////////////////////////////////////////////////////////////
template <typename DOM,int N>
struct regular_mesh_cell_neighbor
{
  typedef typename DOM::tuple_type tuple_type;
  typedef typename regular_face_property
      <tuple_size<tuple_type>::value>::normal_type   normal_type;
  typedef std::pair<tuple_type, normal_type> result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param dom mesh domain.
  /// @param t1 cell index.
  /// @param idx neighbor index
  /// @return pair with neighboring cell position and normal.
  //////////////////////////////////////////////////////////////////////
  result_type operator()(DOM const& dom,
                         tuple_type const& t1, size_t idx) const
  {
    normal_type normal;
    tuple_type tuple_tmp(t1);
    /*Try positive direction of axis N*/
    get<N>(tuple_tmp) = get<N>(tuple_tmp)+1;
    if (idx==0)
    { //if it is the good neighbor to return
      normal.template set<N>(1);
      if (dom.contains(tuple_tmp))
      //if neighbor exists, return it
        return result_type(tuple_tmp, normal);
      else
      //if neighbor is boundary, return myself
        return result_type(t1, normal);
    }
    else
      idx--;

    /*Try negative direction of axis N*/
    if (idx==0)
    { //if it is the good neighbor to return
      normal.template set<N>(-1);
      if (static_cast<int>(get<N>(tuple_tmp))-2<0)
      //if neighbor is boundary (negative position), return myself
        return result_type(t1, normal);
      get<N>(tuple_tmp) = get<N>(tuple_tmp)-2;
      if (dom.contains(tuple_tmp))
      //if neighbor exists, return it
        return result_type(tuple_tmp, normal);
      else
      //if neighbor is boundary, return myself
        return result_type(t1, normal);
    }
    else
      idx--;
    return regular_mesh_cell_neighbor<DOM,N-1>()(dom,t1,idx);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor getting a cell neighbor.
/// @tparam DOM mesh domain type.
/// @todo put in detail namespace.
///
/// Template specialization to stop the recursion in finding the cell
/// neighbor.
//////////////////////////////////////////////////////////////////////
template <typename DOM>
struct regular_mesh_cell_neighbor<DOM,-1>
{
  typedef typename DOM::tuple_type tuple_type;
  typedef typename regular_face_property
      <tuple_size<tuple_type>::value>::normal_type   normal_type;
  typedef std::pair<tuple_type, normal_type> result_type;
  result_type operator()(const DOM &dom,
                         tuple_type const& t1, size_t idx) const
  {
    return result_type();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Regular mesh edge functor creating the edges of a cell on-the-fly.
/// @tparam Dom mesh domain type.
/// @todo put in detail namespace.
//////////////////////////////////////////////////////////////////////
template<typename DOM>
struct regular_mesh_edge_func
{
public:
  typedef typename DOM::tuple_type tuple_type;
  typedef size_t                                             vertex_descriptor;
  typedef edge_descriptor_impl<vertex_descriptor>              edge_descriptor;
  typedef regular_face_property<tuple_size<tuple_type>::value> edge_property;
  typedef sequential::graph_property_edge<vertex_descriptor,
                                          edge_property>       edge_type;
  typedef edge_type result_type;

private:
  DOM m_dom;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param domain mesh domain.
  //////////////////////////////////////////////////////////////////////
  regular_mesh_edge_func(DOM const& domain)
    : m_dom(domain)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns number of edges of a cell.
  /// @return number of edges.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return 2*tuple_size<tuple_type>::value;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Build a cell edge.
  /// @param parent index of source cell of the edge.
  /// @param idx neighbor index.
  /// @return cell edge.
  //////////////////////////////////////////////////////////////////////
  edge_type operator() (size_t const& parent, size_t const& idx) const
  {
    typedef regular_mesh_cell_neighbor<DOM,tuple_size<tuple_type>::value-1>
                                                  regular_mesh_cell_neighbor_t;
    typedef typename regular_mesh_cell_neighbor_t::result_type result_t;
    tuple_type parent_pos = m_dom.reverse_linearize(parent);
    result_t result = regular_mesh_cell_neighbor<
                                        DOM, tuple_size<tuple_type>::value-1>()
                                                      (m_dom, parent_pos, idx);

    return edge_type(edge_descriptor(parent, m_dom.linearize(result.first)),
                                                 edge_property(result.second));
  }

  void define_type(typer& t)
  { t.member(m_dom); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Implicit mesh cell built on-the-fly.
/// @tparam EdgeGenFunc edge generator functor type.
/// @todo put in detail namespace.
//////////////////////////////////////////////////////////////////////
template<typename EdgeGenFunc>
class implicit_vertex
{
public:
  typedef EdgeGenFunc                             edge_functor;
  typedef size_t                                  vertex_descriptor;
  typedef properties::no_property                 property_type;
  typedef property_type                           property_reference;
  typedef typename edge_functor::edge_property    edge_property;
  typedef typename edge_functor::edge_descriptor  edge_descriptor;
  typedef typename edge_functor::edge_type        edge_type;
  typedef boost::transform_iterator<
            boost::function<
               edge_type (size_t const&)>,
            boost::counting_iterator<
               vertex_descriptor> >               adj_edge_iterator;
  typedef const adj_edge_iterator                 const_adj_edge_iterator;

private:
  vertex_descriptor m_descriptor;
  EdgeGenFunc       m_f;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param d vertex descriptor.
  /// @param f edge generator functor.
  //////////////////////////////////////////////////////////////////////
  implicit_vertex(size_t const& d, EdgeGenFunc f)
    : m_descriptor(d), m_f(f)
  { }

  vertex_descriptor descriptor(void) const
  {
    return m_descriptor;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge iterator on the first edge of the vertex.
  /// @return edge iterator on the first edge of the vertex.
  //////////////////////////////////////////////////////////////////////
  adj_edge_iterator begin(void)
  {
    return boost::make_transform_iterator(boost::counting_iterator<
                                            vertex_descriptor>(0),
                                          boost::bind(m_f, m_descriptor, _1));
  }
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an edge iterator after the last edge of the vertex.
  /// @return edge iterator after the last edge of the vertex.
  //////////////////////////////////////////////////////////////////////
  adj_edge_iterator end(void)
  {
    return boost::make_transform_iterator(boost::counting_iterator<
                                            vertex_descriptor>(m_f.size()),
                                          boost::bind(m_f, m_descriptor, _1));
  }

  const_adj_edge_iterator begin(void) const
  {
    return boost::make_transform_iterator(boost::counting_iterator<
                                            vertex_descriptor>(0),
                                          boost::bind(m_f, m_descriptor, _1));
  }

  const_adj_edge_iterator end(void) const
  {
    return boost::make_transform_iterator(boost::counting_iterator<
                                            vertex_descriptor>(m_f.size()),
                                          boost::bind(m_f, m_descriptor, _1));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of edges of the vertex.
  /// @return number of edges of the vertex.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return m_f.size();
  }

  void define_type(typer& t)
  {
    t.member(m_descriptor);
    t.member(m_f);
  }
};


template<typename EdgeGenFunc, typename Accessor>
class proxy<implicit_vertex<EdgeGenFunc>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef implicit_vertex<EdgeGenFunc> target_t;

public:
  typedef typename target_t::edge_functor        edge_functor;
  typedef typename target_t::vertex_descriptor   vertex_descriptor;
  typedef typename target_t::property_type       property_type;
  typedef typename target_t::property_reference  property_reference;
  typedef typename target_t::edge_property       edge_property;
  typedef typename target_t::edge_descriptor     edge_descriptor;
  typedef typename target_t::edge_type           edge_type;
  typedef typename target_t::adj_edge_iterator   adj_edge_iterator;
  typedef typename target_t::const_adj_edge_iterator const_adj_edge_iterator;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  operator target_t(void) const
  { return Accessor::read(); }

  vertex_descriptor descriptor(void) const
  { return Accessor::read().descriptor(); }

  adj_edge_iterator begin(void)
  { return Accessor::read().begin(); }

  adj_edge_iterator end(void)
  { return Accessor::read().end(); }

  const_adj_edge_iterator begin(void) const
  { return Accessor::read().begin(); }

  const_adj_edge_iterator end(void) const
  { return Accessor::read().end(); }

  size_t size(void) const
  { return Accessor::read().size();}
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to set up the mesh decomposition, partitioning the
///        mesh along the dimension with the highest number of cells.
/// @tparam N mesh dimension-1.
/// @todo put in detail namespace.
//////////////////////////////////////////////////////////////////////
template<int N>
struct decompose_largest_dim
{
  //////////////////////////////////////////////////////////////////////
  /// @param sizes size of the mesh in each dimension.
  /// @param max_value maximum number of cells among the dimensions.
  /// @param num_parts number of partitions for decomposition.
  /// @param decomp number of partitions in each dimension.
  //////////////////////////////////////////////////////////////////////
  template<typename Tuple1, typename Tuple2>
  void operator()(Tuple1 const& sizes,
                  size_t const& max_value,
                  size_t const& num_parts,
                  Tuple2& decomp)
  {
    if (get<N>(sizes) == max_value) {
      get<N>(decomp) = num_parts;
      return;
    }
    decompose_largest_dim<N-1>()(sizes, max_value, num_parts, decomp);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor to set up the mesh decomposition, partitioning the
///        mesh along the dimension with the highest number of cells.
/// @todo put in detail namespace.
///
/// Template specialization to stop the recursion
//////////////////////////////////////////////////////////////////////
template<>
struct decompose_largest_dim<-1>
{
  template<typename Tuple1, typename Tuple2>
  void operator()(Tuple1 const& sizes,
                  size_t const& max_value,
                  size_t const& num_parts,
                  Tuple2& decomp)
  { }
};

template<typename DI>
class flat_container;


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor returning the element passed in constructor.
/// @todo Use general identity
//////////////////////////////////////////////////////////////////////
template<typename T>
class mesh_identity
{
private:
  T const& m_t;

public:
  typedef T result_type;

  mesh_identity(T const& t)
   : m_t(t)
  { }

  template<typename Q>
  T const& operator()(Q const&) const
  { return m_t; }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Functor returning the locality metadata information of the
///        mesh container.
/// @tparam C container type.
/// @todo There should probably be a custom domain extractor
///       for regular meshes
/// @todo put in detail namespace.
//////////////////////////////////////////////////////////////////////
template<typename C>
struct implicit_regular_mesh_metadata
{
  typedef C                                                    container_type;
  typedef detail::extract_domain<C>                            get_dom_t;
  typedef typename get_dom_t::result_type                      domain_type;
  typedef typename domain_type::index_type                     index_type;
  typedef typename container_type::tuple_type                  tuple_type;
  typedef C                                                    component_type;

  typedef metadata_entry<domain_type, component_type*, size_t> dom_info_type;

  typedef metadata::flat_container<dom_info_type>              md_cont_type;
  typedef std::pair<bool, md_cont_type*>                       return_type;

  //////////////////////////////////////////////////////////////////////
  /// @param c pointer to the container.
  ///
  /// @return a pair indicating if the metadata container is static and
  ///   balance distributed and a pointer to the metadata container.
  //////////////////////////////////////////////////////////////////////
  return_type operator()(C* c) const
  {
    const size_t n = c->get_num_locations();

    tuple_type decomp = tuple_ops::transform(
      tuple_type(), detail::mesh_identity<size_t>(1)
    );

    tuple_type local_sizes = get_dom_t()(c).get_local_sizes();

    size_t max_value = tuple_ops::fold(local_sizes,size_t(0),max<size_t>());

    decompose_largest_dim<tuple_size<tuple_type>::value-1>()
      (get_dom_t()(c).get_local_sizes(), max_value, n, decomp);

    spatial_regular_domain_decompose<domain_type>
      npart(get_dom_t()(c), decomp);

    size_t num_part = npart.size();

    md_cont_type* out_part = new md_cont_type(num_part);

    if (num_part>0)
    {
      if (out_part->get_location_id() < num_part)
      {
        size_t id = out_part->get_location_id();

        (*out_part)[id] =
          dom_info_type(
            id, npart[id], NULL,
            LQ_CERTAIN, get_affinity(),
            c->get_rmi_handle(), c->get_location_id()
          );
      }
    }

    return std::make_pair(true, out_part);
  }
}; // struct implicit_regular_mesh_metadata


template <typename Container>
struct implicit_regular_mesh_distribution
  : public p_object
{
private:
  typedef detail::extract_domain<Container>                   get_dom_t;

  typename get_dom_t::result_type&                            m_domain;

public:
  typedef typename get_dom_t::result_type::tuple_type         tuple_type;
  typedef typename get_dom_t::result_type                     domain_type;
  typedef Container                                           component_type;
  typedef metadata_entry<domain_type, component_type*, size_t>   dom_info_type;

  // partition_type defined only to satisfy is_fixed_size_md
  typedef balanced_partition<domain_type>                     partition_type;

  implicit_regular_mesh_distribution(domain_type& domain)
    : m_domain(domain)
  { }

  domain_type const& domain(void) const
  {
    return m_domain;
  }

  future<dom_info_type> metadata_at(size_t gid)
  {
    const size_t n  = this->get_num_locations();
    const size_t id = this->get_location_id();

    tuple_type decomp = tuple_ops::transform(
      tuple_type(), detail::mesh_identity<size_t>(1)
    );

    tuple_type local_sizes = m_domain.get_local_sizes();

    size_t max_value = tuple_ops::fold(local_sizes, size_t(0), max<size_t>());

    decompose_largest_dim<tuple_size<tuple_type>::value-1>()
                             (m_domain.get_local_sizes(),max_value, n, decomp);

    spatial_regular_domain_decompose<domain_type> npart(m_domain, decomp);

    size_t num_part = npart.size();

    for (size_t i = 0; i != num_part; ++i)
    {
      domain_type ndom(npart[i]);

      if (ndom.contains(gid))
      {
        return make_ready_future(dom_info_type(
                 id, ndom, nullptr,
                 LQ_CERTAIN, get_affinity(),
                 this->get_rmi_handle(), this->get_location_id()));
      }
    }

    abort("GID not found in distributed implicit regular mesh domain");

    return make_ready_future(dom_info_type(
             id, npart[0], nullptr,
             LQ_CERTAIN, get_affinity(),
             this->get_rmi_handle(), this->get_location_id()));
  }
}; // struct implicit_regular_mesh_distribution


//////////////////////////////////////////////////////////////////////
/// @brief Implicit container representing a regular mesh.
/// @tparam N mesh dimension.
//////////////////////////////////////////////////////////////////////
template<int N>
struct implicit_regular_mesh_container
  : public p_object
{
  typedef implicit_regular_mesh_container<N>            this_type;
  typedef implicit_regular_mesh_distribution<this_type> distribution_type;
  typedef implicit_regular_mesh_metadata<this_type>     loc_dist_metadata;
  typedef typename size_tuple<N>::type                  tuple_type;
  typedef implicit_regular_mesh_domain<tuple_type>      domain_type;
  typedef domain_type                                   closed_domain_type;
  typedef regular_mesh_edge_func<domain_type>           edge_gen_func_type;
  typedef implicit_vertex< edge_gen_func_type >         value_type;
  typedef value_type                                    reference;
  typedef const value_type                              const_reference;
  typedef reference                                     vertex_reference;
  typedef typename vertex_reference::adj_edge_iterator  adj_edge_iterator;
  typedef typename vertex_reference::const_adj_edge_iterator
                                                        const_adj_edge_iterator;
  typedef typename vertex_reference::property_type      vertex_property;
  typedef typename vertex_reference::edge_property      edge_property;
  typedef typename vertex_reference::edge_descriptor    edge_descriptor;
  typedef typename vertex_reference::vertex_descriptor  vertex_descriptor;
  typedef size_t                                        gid_type;
  typedef container_accessor<this_type>                 accessor_type;
  typedef container_iterator<this_type, accessor_type,
                      std::random_access_iterator_tag>  iterator;

private:
  domain_type       m_dom;
  distribution_type m_dist;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param size size of the mesh in each dimension.
  //////////////////////////////////////////////////////////////////////
  implicit_regular_mesh_container(tuple_type const& size)
    : m_dom(size), m_dist(m_dom)
  { }

  distribution_type& distribution(void)
  {
    return m_dist;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns cell whose id is @ref idx.
  /// @param idx cell id.
  /// @return cell.
  //////////////////////////////////////////////////////////////////////
  value_type operator[](gid_type const& idx) const
  {
    return value_type(idx, edge_gen_func_type(m_dom));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns cell located @ref n positions after cell id
  ///        @ref idx in the domain.
  /// @param idx cell id.
  /// @param n number of positions.
  /// @return cell.
  //////////////////////////////////////////////////////////////////////
  value_type advance(gid_type const& idx, long const& n)
  {
    return value_type(m_dom.advance(idx, n), edge_gen_func_type(m_dom));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a cell iterator on the first cell of the mesh.
  /// @return cell iterator on the first cell of the mesh.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return iterator(this, m_dom.first());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a cell iterator after the last cell of the mesh.
  /// @return cell iterator after the last cell of the mesh.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return iterator(this, m_dom.open_last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a cell iterator on cell whose id is @ref gid.
  /// @param gid mesh cell id.
  /// @return cell iterator on cell whose id is @ref gid.
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return iterator(this, gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an iterator to the specified gid, which is restricted
  /// to the provided domain.
  /// @param domain The domain to which the iterator will be restricted.
  /// @param gid The gid of the vertex to which the iterator will point.
  /// @return The iterator pointing to the specified gid, restricted to the
  /// given domain.
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(domain_type const& domain, gid_type const& gid)
  {
    return iterator(this,domain,gid);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns cell whose id is @ref idx.
  /// @param idx cell id.
  /// @return cell.
  //////////////////////////////////////////////////////////////////////
  value_type get_element(gid_type const& idx) const
  {
    return this->operator[](idx);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the version of the container.
  /// @return Version number of 0
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of the vertices of the mesh.
  /// @return number of the vertices of the mesh.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return m_dom.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the cell domain.
  //////////////////////////////////////////////////////////////////////
  domain_type domain(void) const
  {
    return m_dom;
  }
}; // struct implicit_regular_mesh_container


//////////////////////////////////////////////////////////////////////
/// @brief Helper struct to get the type of a graph view over a mesh.
//////////////////////////////////////////////////////////////////////
template<int N>
struct implicit_regular_mesh_view_type
{
  typedef graph_view<implicit_regular_mesh_container<N> > type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Function creating an implicit regular mesh.
/// @tparam N mesh dimension.
/// @param size size of the mesh in each dimension.
/// @return graph view over the created mesh.
//////////////////////////////////////////////////////////////////////
template<int N>
typename implicit_regular_mesh_view_type<N>::type
make_implicit_regular_mesh_view
  (typename implicit_regular_mesh_container<N>::tuple_type const& size)
{
  return typename implicit_regular_mesh_view_type<N>::type
                                (new implicit_regular_mesh_container<N>(size));
}

} // namespace stapl

#endif // STAPL_CONTAINERS_GRAPH_MESH_IMPLICIT_REGULAR_MESH_HPP
