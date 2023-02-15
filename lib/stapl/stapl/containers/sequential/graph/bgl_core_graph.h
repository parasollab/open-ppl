/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_BGL_CORE_GRAPH_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_BGL_CORE_GRAPH_HPP

#include "graph_util.h"
#if ((STAPL__GNUC__ == 4) && (STAPL__GNUC_MINOR__ == 6))
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#endif
#if ((STAPL__GNUC__ == 4) && (STAPL__GNUC_MINOR__ > 6))
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
#include <boost/graph/adjacency_list.hpp>
#if ((STAPL__GNUC__ == 4) && (STAPL__GNUC_MINOR__ >= 6))
#pragma GCC diagnostic pop
#endif

namespace boost{
  enum vertex_user_property_t {vertex_user_property};
  BOOST_INSTALL_PROPERTY(vertex, user_property);
}

namespace stapl{
namespace sequential{

template<class G> class bgl_graph;

//////////////////////////////////////////////////////////////////////
/// @brief STAPL adaptor for the BGL edge descriptor.
/// @ingroup graphInterop
/// @tparam BGLGraph Type of the boost graph.
//////////////////////////////////////////////////////////////////////
template <class BGLGraph>
class bgl_edge_descriptor_adaptor
{
  typedef bgl_edge_descriptor_adaptor<BGLGraph>           this_type;
  typedef typename boost::graph_traits < BGLGraph >::edge_descriptor
                                                          bgl_edge_descriptor;
  typedef typename boost::graph_traits < BGLGraph >::vertex_descriptor
                                                          bgl_vertex_descriptor;
  typedef typename boost::property_map<BGLGraph,
    boost::edge_weight_t>::type::value_type               edge_property;
  template<class G> friend class bgl_graph;
  //private data
  bgl_edge_descriptor m_ed;
  BGLGraph* m_graph;
 public:
  typedef typename boost::property_map<BGLGraph,
    boost::vertex_name_t>::type::value_type vertex_descriptor;
  typedef bgl_edge_descriptor_adaptor<BGLGraph>    edge_descriptor;

  bgl_edge_descriptor_adaptor()
    : m_graph(NULL)
  {}
  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes the edge descriptor adaptor with a source and a target.
  /// @param _s,_t The source and target vertices.
  //////////////////////////////////////////////////////////////////////
  bgl_edge_descriptor_adaptor(vertex_descriptor _s, vertex_descriptor _t) {
    m_ed = bgl_edge_descriptor(_s,_t,NULL); //no property on desc
    m_graph = NULL;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes the edge descriptor adaptor with a source and a target.
  /// @param _s,_t The source and target vertices.
  /// @todo How is it different from the constructor above ?
  //////////////////////////////////////////////////////////////////////
  bgl_edge_descriptor_adaptor(vertex_descriptor _s, vertex_descriptor _t,
                              size_t) {
    m_ed = bgl_edge_descriptor(_s,_t,NULL);
    m_graph = NULL;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes the edge descriptor adaptor with a BGL edge descriptor,
  ///   an optional edge id and a graph.
  /// @param _ed A BGL edge descriptor.
  /// @param _eid An edge ID, by default it is an INVALID_VALUE
  /// @param _g A pointer to the graph. By default points to NULL.
  //////////////////////////////////////////////////////////////////////
  bgl_edge_descriptor_adaptor(const bgl_edge_descriptor& _ed,
                              size_t _eid=INVALID_VALUE,
                              BGLGraph* _g=NULL)
    :m_ed(_ed), m_graph(_g) {
    if (m_graph != NULL && _eid != INVALID_VALUE)
      boost::put(boost::edge_name, *m_graph, m_ed, _eid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the edge descriptor of this adaptor.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor descriptor(void) const {
    return edge_descriptor(*(this->base_reference()), this->id(), m_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the source vertex of the edge descriptor of this adaptor.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor   source(void)  const { return m_ed.m_source;}

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the target vertex of the edge descriptor of this adaptor.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor   target(void)  const { return m_ed.m_target;}

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the ID of the edge descriptor of this adaptor.
  //////////////////////////////////////////////////////////////////////
  size_t id(void) const {
    if (m_graph == NULL) return INVALID_VALUE;
    else return boost::get(boost::edge_name, *m_graph, m_ed);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Prints informations about the edge descriptor.
  //////////////////////////////////////////////////////////////////////
  void display() const {
    std::cout<<this->source()<<"->"<<this->target()<<":"<<this->id()<<" ";
  }

 private:
  //internal for bgl_core class
  bgl_edge_descriptor bgl_descriptor(void) const { return m_ed;}
};

//////////////////////////////////////////////////////////////////////
/// @brief BGL edge reference wrapper
/// @ingroup graphInterop
/// @tparam BaseReference Type of the BGL edge.
/// @tparam BGLGraph Type of the boost graph.
//////////////////////////////////////////////////////////////////////
template <class BaseReference, class BGLGraph>
class bgl_edge_reference
{
  BaseReference m_ref;
  BGLGraph* m_graph;
 public:
  typedef typename boost::property_map<BGLGraph,
    boost::vertex_name_t>::type::value_type        vertex_descriptor;
  typedef bgl_edge_descriptor_adaptor<BGLGraph>    edge_descriptor;
  typedef typename boost::property_map<BGLGraph,
    boost::edge_weight_t>::type::value_type        edge_property;

  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes an edge.
  /// @param _ref The BGL edge to be wrapped.
  /// @param _graph The graph which contains the edge.
  //////////////////////////////////////////////////////////////////////
  bgl_edge_reference(BaseReference _ref, BGLGraph* _graph)
    : m_ref(_ref), m_graph(_graph) {}

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the edge descriptor of this edge.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor descriptor() const {
    return edge_descriptor(*(this->m_ref), this->id(), m_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the source vertex of this edge.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor source(void) const {
    return boost::get(boost::vertex_name, *m_graph,
                      boost::source(*(this->m_ref), *m_graph));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the target vertex of this edge.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor target(void) const {
    return boost::get(boost::vertex_name, *m_graph,
                      boost::target(*(this->m_ref), *m_graph));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the ID of this edge.
  //////////////////////////////////////////////////////////////////////
  size_t id() const {
    return boost::get(boost::edge_name, *m_graph, *(this->m_ref) );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the edge property.
  //////////////////////////////////////////////////////////////////////
  edge_property& property() {
    return boost::get(boost::edge_weight, *m_graph)[*(this->m_ref)];
  }
};


template<typename BaseIterator,typename BGLGraph>
class const_bgl_edge_iterator_adaptor;

//////////////////////////////////////////////////////////////////////
/// @brief STAPL adaptor for the BGL edge iterator.
/// @ingroup graphInterop
/// @tparam BaseReference Type of the BGL edge.
/// @tparam BGLGraph Type of the boost graph.
//////////////////////////////////////////////////////////////////////
template<typename BaseIterator,typename BGLGraph>
class bgl_edge_iterator_adaptor
    : public boost::iterator_adaptor<bgl_edge_iterator_adaptor<BaseIterator,
                                                               BGLGraph>,
                                     BaseIterator>
{
 private:
  friend class const_bgl_edge_iterator_adaptor<BaseIterator,BGLGraph>;
  typedef boost::iterator_adaptor<bgl_edge_iterator_adaptor<BaseIterator,
                                                            BGLGraph>,
                                  BaseIterator>            base_type;
  typedef bgl_edge_iterator_adaptor<BaseIterator,BGLGraph> this_type;
  typedef typename boost::graph_traits < BGLGraph >::vertex_descriptor
                                                          bgl_vertex_descriptor;
  typedef typename boost::property_map<BGLGraph,
    boost::edge_weight_t>::type::value_type  edge_property;
  BGLGraph*                      m_graph;

 public:
  typedef typename boost::property_map<BGLGraph,
    boost::vertex_name_t>::type::value_type               vertex_descriptor;
  typedef bgl_edge_descriptor_adaptor<BGLGraph>           edge_descriptor;
  typedef bgl_edge_reference<BaseIterator, BGLGraph>      value_type;

  bgl_edge_iterator_adaptor()
    : m_graph(NULL)
  {}

  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes an edge iterator adaptor.
  /// @param _iterator The BGL edge iterator to be wrapped.
  /// @param _graph Pointer to the graph where the edge is.
  //////////////////////////////////////////////////////////////////////
  bgl_edge_iterator_adaptor(BaseIterator _iterator, BGLGraph* _graph)
    : base_type(_iterator), m_graph(_graph) {}

  value_type operator*() const {
    return value_type(this->base_reference(), m_graph);
  }
};// class edge_iterator_adaptor


//////////////////////////////////////////////////////////////////////
/// @brief View on the vertices. The vertices are accessible with iterators.
/// @ingroup graphInterop
/// @tparam Iter iterator type.
//////////////////////////////////////////////////////////////////////
template <class Iter>
class bgl_ve_view
{
  Iter m_cref_first;
  Iter m_cref_second;
 public:
  typedef Iter       iterator;
  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes a view with 2 iterators : The begin and the end.
  /// @param _cref1, _cref2 The begin and end iterators.
  //////////////////////////////////////////////////////////////////////
  bgl_ve_view(Iter& _cref1, Iter& _cref2)
    : m_cref_first(_cref1), m_cref_second(_cref2) {}
  iterator begin() const { return iterator(m_cref_first);}
  iterator   end() const { return iterator(m_cref_second);}
};


//////////////////////////////////////////////////////////////////////
/// @brief BGL vertex reference wrapper
/// @ingroup graphInterop
/// @tparam BaseReference Type of the BGL vertex.
/// @tparam BGLGraph Type of the boost graph.
//////////////////////////////////////////////////////////////////////
template <typename BaseReference, typename BGLGraph>
class bgl_vertex_reference
{
private:
  typedef typename boost::graph_traits < BGLGraph >::out_edge_iterator
    bgl_out_edge_iterator;
  typedef bgl_edge_iterator_adaptor<bgl_out_edge_iterator, BGLGraph>
    out_edge_iterator;

 protected:
  BaseReference m_ref;
  BGLGraph*     m_graph;

 public:
  typedef typename boost::property_map<BGLGraph,
    boost::vertex_name_t>::type::value_type          vertex_descriptor;
  typedef typename std::iterator_traits<BaseReference>::value_type
                                                     value_type;
  typedef typename boost::property_map<BGLGraph,
    boost::vertex_user_property_t>::type::value_type property_type;
  typedef bgl_ve_view<out_edge_iterator>             adj_edge_view;

  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes a vertex.
  /// @param _ref The BGL vertex to be wrapped.
  /// @param _graph The graph which contains the vertex.
  //////////////////////////////////////////////////////////////////////
  bgl_vertex_reference(BaseReference _ref, BGLGraph* _graph)
    : m_ref(_ref) , m_graph(_graph) {}

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the vertex descriptor of this edge.
  //////////////////////////////////////////////////////////////////////
  size_t descriptor()   const {
    return boost::get(boost::vertex_name, *m_graph, *(this->m_ref) );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the vertex property.
  //////////////////////////////////////////////////////////////////////
  property_type& property()   {
    return boost::get(boost::vertex_user_property, *m_graph)[*(this->m_ref)];
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator pointing to the begin of all the out edges of
  ///   this vertex.
  //////////////////////////////////////////////////////////////////////
  out_edge_iterator begin()   {
    return out_edge_iterator( boost::out_edges(*(this->m_ref),
                                               *m_graph).first,m_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator pointing to the end of all the out edges of
  ///   this vertex.
  //////////////////////////////////////////////////////////////////////
  out_edge_iterator end()     {
    return out_edge_iterator( boost::out_edges(*(this->m_ref),
                                               *m_graph).second,m_graph);
  }
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the out degree of this vertex.
  //////////////////////////////////////////////////////////////////////
  size_t size(void)     const {
    return boost::out_degree(*(this->m_ref), *m_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a view on the edges of this vertex.
  //////////////////////////////////////////////////////////////////////
  adj_edge_view edges() const {
    return adj_edge_view(this->begin(), this->end());
  }
};



template<typename BaseIterator,typename BGLGraph>
class const_bgl_vertex_iterator_adaptor;

//////////////////////////////////////////////////////////////////////
/// @brief STAPL adaptor for the BGL vertex iterator.
/// @ingroup graphInterop
/// @tparam BaseIterator The base vertex iterator.
/// @tparam BGLGraph Type of the boost graph.
//////////////////////////////////////////////////////////////////////
template<typename BaseIterator,typename BGLGraph>
class bgl_vertex_iterator_adaptor
    : public boost::iterator_adaptor<
        bgl_vertex_iterator_adaptor<BaseIterator, BGLGraph>,
        BaseIterator>
{
private:
  friend class const_bgl_vertex_iterator_adaptor<BaseIterator,BGLGraph>;
  typedef boost::iterator_adaptor<bgl_vertex_iterator_adaptor<BaseIterator,
                                                              BGLGraph>,
                                  BaseIterator>          base_type;
  typedef bgl_vertex_iterator_adaptor<
    BaseIterator,BGLGraph>                               this_type;
  typedef typename boost::graph_traits < BGLGraph >::out_edge_iterator
                                                         bgl_out_edge_iterator;
  typedef bgl_edge_iterator_adaptor<bgl_out_edge_iterator, BGLGraph>
                                                         out_edge_iterator;

  BGLGraph*  m_graph;

 public:
  typedef typename boost::property_map<BGLGraph,
    boost::vertex_name_t>::type::value_type                  vertex_descriptor;
  typedef bgl_vertex_reference<BaseIterator, BGLGraph>       value_type;
  typedef typename boost::property_map<BGLGraph,
    boost::vertex_user_property_t>::type::value_type         property_type;

  bgl_vertex_iterator_adaptor()
    : m_graph(NULL)
  {}
  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes an iterator adaptor with a given iterator and a
  ///   pointer to a graph.
  //////////////////////////////////////////////////////////////////////
  bgl_vertex_iterator_adaptor(BaseIterator _iterator, BGLGraph* _graph)
    : base_type(_iterator), m_graph(_graph) {}

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the vertex reference this iterator points to.
  //////////////////////////////////////////////////////////////////////
  value_type operator*()  const {
    return value_type(this->base_reference(), m_graph);
  }
};// class vertex_iterator_adaptor


//////////////////////////////////////////////////////////////////////
/// @copydoc bgl_edge_reference
/// @ingroup graphInterop
//////////////////////////////////////////////////////////////////////
template <class BaseReference, class BGLGraph>
class const_bgl_edge_reference
{
  BaseReference   m_ref;
  const BGLGraph* m_graph;
 public:
  typedef typename boost::property_map<BGLGraph,
    boost::vertex_name_t>::type::value_type           vertex_descriptor;
  typedef bgl_edge_descriptor_adaptor<BGLGraph>       edge_descriptor;
  typedef typename boost::property_map<BGLGraph,
    boost::edge_weight_t>::type::value_type           edge_property;

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_edge_reference::bgl_edge_reference(BaseReference _ref, const BGLGraph* _graph)
  //////////////////////////////////////////////////////////////////////
  const_bgl_edge_reference(BaseReference _ref, const BGLGraph* _graph)
    : m_ref(_ref), m_graph(_graph) {}

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_edge_reference::descriptor() const
  //////////////////////////////////////////////////////////////////////
  edge_descriptor descriptor() const {
    return edge_descriptor(*(this->m_ref), this->id(), m_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_edge_reference::source() const
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor source(void) const {
    return boost::get(boost::vertex_name, *m_graph,
                      boost::source(*(this->m_ref), *m_graph));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_edge_reference::target() const
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor target(void) const {
    return boost::get(boost::vertex_name, *m_graph,
                      boost::target(*(this->m_ref), *m_graph));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_edge_reference::id() const
  //////////////////////////////////////////////////////////////////////
  size_t id() const {
    return boost::get(boost::edge_name, *m_graph, *(this->m_ref) );
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_edge_reference::property() const
  //////////////////////////////////////////////////////////////////////
  const edge_property& property() {
    return boost::get(boost::edge_weight, *m_graph)[*(this->m_ref)];
  }
};


//////////////////////////////////////////////////////////////////////
/// @copydoc bgl_edge_iterator_adaptor
/// @ingroup graphInterop
//////////////////////////////////////////////////////////////////////
template<typename BaseIterator,typename BGLGraph>
class const_bgl_edge_iterator_adaptor
    : public boost::iterator_adaptor<
                      const_bgl_edge_iterator_adaptor<BaseIterator, BGLGraph>,
                      BaseIterator>
{
 private:
  typedef boost::iterator_adaptor<const_bgl_edge_iterator_adaptor<BaseIterator,
    BGLGraph>, BaseIterator>                                     base_type;
  typedef const_bgl_edge_iterator_adaptor<BaseIterator,BGLGraph> this_type;
  typedef typename boost::property_map<BGLGraph,
    boost::edge_weight_t>::type::value_type                      edge_property;

  const BGLGraph*  m_graph;

 public:
  typedef typename boost::graph_traits < BGLGraph >::vertex_descriptor
                                                              vertex_descriptor;
  typedef bgl_edge_descriptor_adaptor<BGLGraph>               edge_descriptor;
  typedef const_bgl_edge_reference<BaseIterator, BGLGraph>    value_type;

  const_bgl_edge_iterator_adaptor()
    : m_graph(NULL)
  {}

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_edge_iterator_adaptor::bgl_edge_iterator_adaptor(BaseIterator _iterator, BGLGraph* _graph)
  //////////////////////////////////////////////////////////////////////
  const_bgl_edge_iterator_adaptor(BaseIterator _iterator,
                                  const BGLGraph* _graph)
    : base_type(_iterator), m_graph(_graph) {}

  const_bgl_edge_iterator_adaptor(bgl_edge_iterator_adaptor<BaseIterator,
                                                            BGLGraph>& iter)
    : base_type(iter.base()), m_graph(iter.m_graph) {}

  value_type operator*() const {
    return value_type((this->base_reference()), m_graph);
  }
};// class edge_iterator_adaptor


//////////////////////////////////////////////////////////////////////
/// @copydoc bgl_vertex_reference
/// @ingroup graphInterop
//////////////////////////////////////////////////////////////////////
template <typename BaseReference, typename BGLGraph>
class const_bgl_vertex_reference
{
private:
  typedef typename boost::graph_traits < BGLGraph >::out_edge_iterator
                                                       bgl_out_edge_iterator;
  typedef const_bgl_edge_iterator_adaptor<bgl_out_edge_iterator,
    BGLGraph>                                          const_out_edge_iterator;

 protected:
  BaseReference    m_ref;
  const BGLGraph*  m_graph;

 public:
  typedef typename boost::property_map<BGLGraph,
    boost::vertex_name_t>::type::value_type                  vertex_descriptor;
  typedef typename std::iterator_traits<BaseReference>::value_type
                                                             value_type;
  typedef typename boost::property_map<BGLGraph,
    boost::vertex_user_property_t>::type::value_type         property_type;
  typedef bgl_ve_view<const_out_edge_iterator>               adj_edge_view;

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_vertex_reference::bgl_vertex_reference(BaseReference _ref, BGLGraph* _graph)
  //////////////////////////////////////////////////////////////////////
  const_bgl_vertex_reference(BaseReference _ref, const BGLGraph* _graph)
    : m_ref(_ref) , m_graph(_graph) {}

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_vertex_reference::descriptor() const
  //////////////////////////////////////////////////////////////////////
  size_t descriptor()         const {
    return boost::get(boost::vertex_name, *m_graph, *(this->m_ref) );
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_vertex_reference::property()
  //////////////////////////////////////////////////////////////////////
  const property_type& property()   {
    return boost::get(boost::vertex_user_property, *m_graph)[*(this->m_ref)];
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_vertex_reference::begin()
  //////////////////////////////////////////////////////////////////////
  const_out_edge_iterator begin()   {
    return const_out_edge_iterator( boost::out_edges(*(this->m_ref),
                                                     *m_graph).first,m_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_vertex_reference::end()
  //////////////////////////////////////////////////////////////////////
  const_out_edge_iterator end()     {
    return const_out_edge_iterator( boost::out_edges(*(this->m_ref),
                                                     *m_graph).second,m_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_vertex_reference::size() const
  //////////////////////////////////////////////////////////////////////
  size_t size(void)           const {
    return boost::out_degree(*(this->m_ref), *m_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_vertex_reference::edges() const
  //////////////////////////////////////////////////////////////////////
  adj_edge_view edges()       const {
    return adj_edge_view(this->begin(), this->end());
  }
};


//////////////////////////////////////////////////////////////////////
/// @copydoc bgl_vertex_iterator_adaptor
/// @ingroup graphInterop
//////////////////////////////////////////////////////////////////////
template<typename BaseIterator,typename BGLGraph>
class const_bgl_vertex_iterator_adaptor
    : public boost::iterator_adaptor<
        const_bgl_vertex_iterator_adaptor<BaseIterator,BGLGraph>,
        BaseIterator>
{
private:
  typedef boost::iterator_adaptor<
    const_bgl_vertex_iterator_adaptor<BaseIterator, BGLGraph>,
    BaseIterator>                                      base_type;
  typedef const_bgl_vertex_iterator_adaptor<
    BaseIterator,BGLGraph>                             this_type;
  typedef typename boost::graph_traits<BGLGraph>::out_edge_iterator
                                                       bgl_out_edge_iterator;
  typedef const_bgl_edge_iterator_adaptor<bgl_out_edge_iterator, BGLGraph>
                                                       const_out_edge_iterator;

  const BGLGraph* m_graph;

 public:
  typedef typename boost::graph_traits < BGLGraph >::vertex_descriptor
    vertex_descriptor;
  typedef const_bgl_vertex_reference<BaseIterator, BGLGraph>
    value_type;
  typedef typename boost::property_map<BGLGraph,
    boost::vertex_name_t>::type::value_type  property_type;

  const_bgl_vertex_iterator_adaptor()
    : m_graph(NULL)
  {}

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_vertex_iterator_adaptor(BaseIterator _iterator,const BGLGraph* _graph)
  //////////////////////////////////////////////////////////////////////
  const_bgl_vertex_iterator_adaptor(BaseIterator _iterator,
                                    const BGLGraph* _graph) :
        base_type(_iterator), m_graph(_graph) {}

  const_bgl_vertex_iterator_adaptor(bgl_vertex_iterator_adaptor<BaseIterator,
                                    BGLGraph>& iter) :
        base_type(iter.base()), m_graph(iter.m_graph) {}

  //////////////////////////////////////////////////////////////////////
  /// @copydoc bgl_vertex_iterator_adaptor::operator*() const
  //////////////////////////////////////////////////////////////////////
  value_type operator*()  const {
    return value_type(this->base_reference(), m_graph);
  }
};// class vertex_iterator_adaptor


//////////////////////////////////////////////////////////////////////
/// @brief Core graph implementation based on BGL.
/// @tparam BGL_Graph Type of the boost graph.
/// @ingroup graphInterop
//////////////////////////////////////////////////////////////////////
template <class BGL_Graph>
class bgl_graph
{

  typedef bgl_graph<BGL_Graph>     this_type;
  typedef BGL_Graph                graph_t;

  typedef typename boost::graph_traits < graph_t >::vertex_iterator
    bgl_vertex_iterator;
  typedef typename boost::graph_traits < graph_t >::edge_iterator
    bgl_edge_iterator;
  typedef typename boost::graph_traits < graph_t >::out_edge_iterator
    bgl_out_edge_iterator;
  typedef typename boost::graph_traits < graph_t >::vertex_descriptor
    bgl_vertex_descriptor;
  typedef typename boost::graph_traits < graph_t >::edge_descriptor
    bgl_edge_descriptor;
 public:

  typedef typename boost::graph_traits < graph_t >::vertex_descriptor
    vertex_descriptor;
  typedef bgl_edge_descriptor_adaptor<graph_t>  edge_descriptor;

  typedef typename boost::property_map<graph_t,
    boost::vertex_user_property_t>::type::value_type  vertex_property;
  typedef typename boost::property_map<graph_t,
    boost::edge_weight_t>::type::value_type  edge_property;

  //vertex descriptor generator
  typedef vertex_descriptor_generator<size_t>  vdg_type;

  typedef bgl_vertex_iterator_adaptor<bgl_vertex_iterator,graph_t>
    vertex_iterator;
  typedef const_bgl_vertex_iterator_adaptor<bgl_vertex_iterator,graph_t>
    const_vertex_iterator;
  typedef bgl_edge_iterator_adaptor<bgl_edge_iterator, graph_t> edge_iterator;
  typedef const_bgl_edge_iterator_adaptor<bgl_edge_iterator, graph_t>
    const_edge_iterator;
  typedef bgl_edge_iterator_adaptor<bgl_out_edge_iterator, graph_t>
    adj_edge_iterator;
  typedef const_bgl_edge_iterator_adaptor<bgl_out_edge_iterator, graph_t>
    const_adj_edge_iterator;

 protected:
  graph_t  m_bgl_graph;
  vdg_type m_vdg;
  size_t   m_edge_id;
 public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a graph with a specific number of vertices.
  /// @param _nv The number of vertices to be created. By default a new graph
  ///   with 0 vertices will be created.
  //////////////////////////////////////////////////////////////////////
 bgl_graph(size_t _nv=0) : m_vdg(0) {
    this->m_edge_id = 0;
    for (size_t i=0;i<_nv;++i){
      this->add_vertex();
    }
  }

  virtual ~bgl_graph() = default;;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator referring to the first vertex of the graph.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator begin(){
    return vertex_iterator(boost::vertices(m_bgl_graph).first,&m_bgl_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc begin()
  //////////////////////////////////////////////////////////////////////
  const_vertex_iterator begin() const {
    return const_vertex_iterator(boost::vertices(m_bgl_graph).first,
                                 &m_bgl_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator referring to the past-the-end vertex of the
  ///   graph.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator end(){
    return vertex_iterator(boost::vertices(m_bgl_graph).second,&m_bgl_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc end()
  //////////////////////////////////////////////////////////////////////
  const_vertex_iterator end() const {
    return const_vertex_iterator(boost::vertices(m_bgl_graph).second,
                                 &m_bgl_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator referring to the first edge of the graph.
  //////////////////////////////////////////////////////////////////////
  edge_iterator edges_begin(){
    return edge_iterator(boost::edges(m_bgl_graph).first,&m_bgl_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc edges_begin()
  //////////////////////////////////////////////////////////////////////
  const_edge_iterator edges_begin() const {
    return const_edge_iterator(boost::edges(m_bgl_graph).first,&m_bgl_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator referring to the past-the-end edge of the
  ///   graph.
  //////////////////////////////////////////////////////////////////////
  edge_iterator edges_end(){
    return edge_iterator(boost::edges(m_bgl_graph).second,&m_bgl_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc edges_end()
  //////////////////////////////////////////////////////////////////////
  const_edge_iterator edges_end() const {
    return const_edge_iterator(boost::edges(m_bgl_graph).second,&m_bgl_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of vertices of the graph.
  //////////////////////////////////////////////////////////////////////
  size_t get_num_vertices() const {
    return boost::num_vertices(m_bgl_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update a descriptor
  /// @bug It seems to do do nothing.
  //////////////////////////////////////////////////////////////////////
  void update_descriptor(size_t&,
                         const vertex_iterator) {}

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the id of the last descriptor.
  //////////////////////////////////////////////////////////////////////
  size_t get_max_descriptor() const {
    return get_num_vertices();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of edges of the graph.
  //////////////////////////////////////////////////////////////////////
  size_t get_num_edges() const {
    return boost::num_edges(m_bgl_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a new vertex to the graph.
  /// @return The vertex descriptor of the new vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(void){
    vertex_descriptor vd = boost::add_vertex(m_bgl_graph);
    typename vdg_type::vertex_descriptor vid =  m_vdg.next() ;
    boost::put(boost::vertex_name, m_bgl_graph, vd, vid);
    return vd;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a new vertex with properties to the graph.
  /// @param _vp The vertex property of the newly created vertex.
  /// @return The vertex descriptor of the new vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_property& _vp){
    bgl_vertex_descriptor bgl_vd = boost::add_vertex(m_bgl_graph);
    typename vdg_type::vertex_descriptor vd =  m_vdg.next() ;
    boost::put(boost::vertex_name, m_bgl_graph, bgl_vd, vd);
    boost::put(boost::vertex_user_property, m_bgl_graph, bgl_vd, _vp);
    return vd;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a new edge to the graph.
  /// @return The edge descriptor of the new edge.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(const edge_descriptor& _ed) {
    return add_edge(_ed,edge_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a new edge with a specific descriptor and property to the
  ///   graph.
  /// @param _ed The descriptor of the new edge
  /// @param _p The property of the edge.
  /// @return The edge descriptor of the new edge.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(const edge_descriptor& _ed, const edge_property& _p){
    vertex_iterator vi1 = find_vertex(_ed.source());
    vertex_iterator vi2 = find_vertex(_ed.target());
    if (vi1 == this->end() && vi2 == this->end())
      return edge_descriptor(vertex_descriptor(INVALID_VALUE),
                             vertex_descriptor(INVALID_VALUE));
    else if (vi1 == this->end())
      return edge_descriptor(vertex_descriptor(INVALID_VALUE),_ed.target());
    else if (vi1 == this->end())
      return edge_descriptor(_ed.source(), vertex_descriptor(INVALID_VALUE));
    else {
      std::pair<bgl_edge_descriptor,bool> res = boost::add_edge(*(vi1.base()),
                                                                *(vi2.base()),
                                                                m_bgl_graph);
      if (res.second){
        //next auto increment edge id
        size_t eid = this->m_edge_id++;
        boost::put(boost::edge_weight, m_bgl_graph, res.first, _p);
        edge_descriptor new_ed(res.first, eid, &m_bgl_graph);//here we set also
        ///  the edge id (edge_name)
        return new_ed;
      }
      else{
        return edge_descriptor(_ed.source(), _ed.target());//it will be invalid
        ///  because the id will be invalid
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears the entire graph.
  //////////////////////////////////////////////////////////////////////
  void clear(void){
    this->m_edge_id = 0;
    m_bgl_graph.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes a vertex with a specific descriptor from the graph.
  /// @param _vd The descriptor of the vertex to be deleted.
  /// @return A bool value that indicates whether the vertex was sucessfully
  ///   deleted or not.
  /// @bug it seems that BGL doesn't remove all edges pointing to the deleted
  ///   vertex.
  //////////////////////////////////////////////////////////////////////
  bool delete_vertex(const vertex_descriptor& _vd){
    size_t nv = this->get_num_vertices();
    //here I have to find the BGL vertex descriptor for the stapl vertex descriptor
    vertex_iterator vi = find_vertex(_vd);
    if (vi == this->end()) return false;
    boost::clear_vertex(*(vi.base()), m_bgl_graph);
    boost::remove_vertex(*(vi.base()), m_bgl_graph);
    if (this->get_num_vertices() == nv-1) return true;
    else return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes a edge with a specific descriptor from the graph.
  /// @param _ed The descriptor of the edge to be deleted.
  /// @return A bool value that indicates whether the edge was sucessfully
  ///   deleted or not.
  /// @todo Check also the edge id to be consistent with stapl specification.
  /// @bug the return value is not correct.
  //////////////////////////////////////////////////////////////////////
  bool delete_edge(const edge_descriptor& _ed){
    boost::remove_edge(_ed.bgl_descriptor(),m_bgl_graph);
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds a edge with a specific descriptor from the graph.
  /// @param _ed The descriptor of the searched edge.
  /// @param _vi If the edge is found, _vi will refer to the source vertex of
  ///   that edge. Otherwise it will point to the graph past-to-end vertex.
  /// @param _ei If the edge is found, This iterator will point to the edge.
  /// @return A bool value that indicates whether the edge was sucessfully
  ///   found or not.
  //////////////////////////////////////////////////////////////////////
  bool find_edge(const edge_descriptor& _ed, vertex_iterator& _vi,
                 adj_edge_iterator& _ei){
    //edge_descriptor n_ed(_ed.bgl_descriptor(),_ed.id(),&m_bgl_graph);
    _vi=find_vertex(_ed.source());
    if (_vi == this->end()) return false;
    else{
      for (adj_edge_iterator ei = (*_vi).begin();ei != (*_vi).end();++ei){
        if ((*ei).target() == _ed.target()) {
          if (_ed.id() == (size_t)INVALID_VALUE){
            _ei=ei;
            return true;
          }
          else{//the id is valid
            if (_ed.id() == (*ei).id()){
              _ei=ei;
              return true;
            }
          }
        }
      }
      //if edge not found return false
      return false;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc  find_edge(const edge_descriptor& _ed, vertex_iterator& _vi, adj_edge_iterator& _ei)
  //////////////////////////////////////////////////////////////////////
  bool find_edge(const edge_descriptor& _ed, const_vertex_iterator& _vi,
                 const_adj_edge_iterator& _ei) const {
    vertex_iterator   i_vi;
    adj_edge_iterator i_ei;
    bool res = const_cast<this_type*>(this)->find_edge(_ed,i_vi,i_ei);
    _vi = const_vertex_iterator(i_vi.base(),&m_bgl_graph);
    _ei = const_adj_edge_iterator(i_ei.base(),&m_bgl_graph);
    return res;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc  find_vertex(const vertex_descriptor& _vd)
  //////////////////////////////////////////////////////////////////////
  const_vertex_iterator find_vertex(const vertex_descriptor& _vd) const {
    vertex_iterator vi = const_cast<this_type*>(this)->find_vertex(_vd);
    return const_vertex_iterator(vi.base(),&m_bgl_graph);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds a vertex with a given vertex descriptor.
  /// @param _vd The vertex descriptor
  /// @return An iterator pointing to the corresponding vertex. If no vertex
  ///   was found, the iterator points to past-the-end element
  //////////////////////////////////////////////////////////////////////
  vertex_iterator find_vertex(const vertex_descriptor& _vd) {
    vertex_iterator vi, startvi;
    if (this->get_num_vertices()==0) return this->end();

    if ( this->get_num_vertices() > _vd ) {
        startvi = this->begin() + _vd;
    } else {
        startvi = this->end() - 1;
    }

    vi = startvi;
    // look back from v[_vid]
    while ( vi != this->begin()  ) {
      if ( (*vi).descriptor() == _vd) {
            return  vi;
        } else {
            vi--;
        }
    }
    if (vi == this->begin()){
      if ( (*vi).descriptor() == _vd) return  vi;
    }

    // look forward from v[_vid]
    vi = startvi;
    while ( vi != this->end()  ) {
      if ( (*vi).descriptor() == _vd) {
            return vi;
        } else {
            ++vi;
        }
    }
    // if didn't find it return this->end() like STL find
    return this->end();
  }
};// class bgl_graph //-----------------------------

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the bgl_adaptor.
/// @ingroup graphInterop
/// @tparam D attribute to specify if the graph is directed or undirected.
/// @tparam M attribute to specify if the graph is a simple or a multi graph.
/// @tparam VertexP Type of a vertex property.
/// @tparam EdgeP Type of an edge property.
////////////////////////////////////////////////////////////////////////
template <graph_attributes D, graph_attributes M, class VertexP, class EdgeP>
class bgl_adaptor_traits
{
  typedef bgl_adaptor_traits<D,M,VertexP,EdgeP>      this_type;

  typedef boost::adjacency_list<boost::vecS,
    boost::vecS,
    boost::directedS,
    boost::property<boost::vertex_name_t,size_t,
    boost::property<boost::vertex_user_property_t,VertexP > >,//vertex id/data
    boost::property<boost::edge_name_t,size_t,
    boost::property<boost::edge_weight_t,EdgeP > >//edge id/data
    > bgl_type;

 public:
  typedef bgl_graph<bgl_type>                          core_graph_type;
  typedef typename core_graph_type::vertex_descriptor  vertex_descriptor;
  typedef typename core_graph_type::edge_descriptor    edge_descriptor;
  typedef EdgeP                                        edge_property;
  typedef VertexP                                      vertex_property;


  typedef typename graph_type<this_type,D>::type       directness_type;
  typedef typename graph_type<this_type,M>::type       multiplicity_type;
 };
}// namespace sequential
}

#endif
