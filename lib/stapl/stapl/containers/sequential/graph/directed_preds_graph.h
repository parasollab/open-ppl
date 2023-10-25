/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_DIRECTED_PREDS_GRAPH_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_DIRECTED_PREDS_GRAPH_HPP

#include <algorithm>
#include "graph.h"
#include "view/directed_preds_graph_view.h"

namespace stapl {

namespace sequential {

//////////////////////////////////////////////////////////////////////
/// @brief Wrapper property around user specified property to maintain
/// predecessors information.
/// @ingroup graphBaseUtil
/// @tparam VD The type of the vertex descriptor.
/// @tparam UserProperty The type of the user vertex property.
//////////////////////////////////////////////////////////////////////
template <class VD, class UserProperty>
class preds_property
{
  /// Stores the predecessors for this vertex.
  std::vector<VD> m_preds;
  /// Stores the user vertex property for this vertex.
  UserProperty m_property;

public:
  typedef std::vector<VD> predlist_type;
  typedef UserProperty user_property_type;

  preds_property()
      : m_preds(), m_property()
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a preds property based on the given user property.
  //////////////////////////////////////////////////////////////////////
  preds_property(UserProperty const& p)
      : m_preds(), m_property(p)
  {
  }

  preds_property(preds_property const& other)
      : m_preds(other.m_preds), m_property(other.m_property)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the predecessors of this vertex.
  //////////////////////////////////////////////////////////////////////
  predlist_type& predecessors(void)
  {
    return m_preds;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the predecessors of this vertex.
  //////////////////////////////////////////////////////////////////////
  predlist_type const& predecessors(void) const
  {
    return m_preds;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the user property of this vertex.
  //////////////////////////////////////////////////////////////////////
  UserProperty& user_property(void)
  {
    return m_property;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the user property of this vertex.
  //////////////////////////////////////////////////////////////////////
  UserProperty const& user_property(void) const
  {
    return m_property;
  }

#ifdef _STAPL
  void define_type(typer& t)
  {
    t.member(m_preds);
    t.member(m_property);
  }
#endif
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Helper for reading directed_preds_graph vertices
/// @ingroup graphBaseUtil
////////////////////////////////////////////////////////////////////////////////
template <class VD, class UserProperty>
std::istream& operator>>(std::istream& is, preds_property<VD, UserProperty>& pp)
{
  return is >> pp.user_property();
}

//////////////////////////////////////////////////////////////////////
/// @brief Helper to infer the user property (const or non const).
/// @ingroup graphBaseUtil
/// @todo Rename to remove leading underscore.
//////////////////////////////////////////////////////////////////////
template <typename Reference>
struct _hei_usr_prop
{
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper to infer the user property (const or non const).
/// Specialized for non-const.
/// @ingroup graphBaseUtil
/// @todo Rename to remove leading underscore.
//////////////////////////////////////////////////////////////////////
template <typename Value>
struct _hei_usr_prop<Value&>
{
  typedef typename Value::property_type::user_property_type user_property_type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper to infer the user property (const or non const).
/// Specialized for const.
/// @ingroup graphBaseUtil
/// @todo Rename to remove leading underscore.
//////////////////////////////////////////////////////////////////////
template <typename Value>
struct _hei_usr_prop<const Value&>
{
  typedef const typename Value::property_type::user_property_type
    user_property_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Class to provide a reference to a vertex object for the
/// @ref directed_preds_graph.
/// @ingroup graphBaseUtil
///
/// Provides the interface of a graph vertex, with additional functionality
/// for working with predecessors.
/// This is the type returned when a vertex iterator is dereferenced.
/// @tparam BaseReference The type of the base vertex iterator.
/// @tparam UserProperty The type of the user vertex property.
//////////////////////////////////////////////////////////////////////
template <typename BaseReference, typename UserProperty>
class dpg_vertex_reference
  : public vertex_reference<BaseReference>
{
private:
  typedef typename std::iterator_traits<BaseReference>::value_type
    internal_value_type;
  typedef typename std::iterator_traits<BaseReference>::reference reference;
  typedef typename internal_value_type::edgelist_type edgelist_type;
  typedef typename _hei<reference>::type ref_property_type;

public:
  typedef typename internal_value_type::vertex_descriptor vertex_descriptor;
  typedef typename internal_value_type::property_type property_type;
  typedef typename _hei_it<reference>::iterator adj_edge_iterator;
  typedef ve_view<edgelist_type, adj_edge_iterator> adj_edge_view;

  typedef typename _hei_usr_prop<reference>::user_property_type
    user_property_type;
  typedef typename property_type::predlist_type predlist_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a vertex reference based on the provided iterator.
  //////////////////////////////////////////////////////////////////////
  dpg_vertex_reference(BaseReference ref)
      : vertex_reference<BaseReference>(ref)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the user property. This provides transparent access
  /// to the user property from the actual property stored in the graph.
  //////////////////////////////////////////////////////////////////////
  user_property_type& property() const
  {
    return this->m_ref->property().user_property();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the predecessors of this vertex.
  //////////////////////////////////////////////////////////////////////
  predlist_type& predecessors()
  {
    return this->m_ref->property().predecessors();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class to provide a const reference to a vertex object for the
/// @ref directed_preds_graph.
/// @ingroup graphBaseUtil
///
/// Provides the interface of a graph vertex, with additional functionality
/// for working with predecessors.
/// This is the type returned when a const vertex iterator is dereferenced.
/// @tparam BaseReference The type of the base vertex iterator.
/// @tparam UserProperty The type of the user vertex property.
//////////////////////////////////////////////////////////////////////
template <typename BaseReference, typename UserProperty>
class const_dpg_vertex_reference
  : public vertex_reference<BaseReference>
{
private:
  typedef typename std::iterator_traits<BaseReference>::value_type
    internal_value_type;
  typedef typename std::iterator_traits<BaseReference>::reference reference;
  typedef typename internal_value_type::edgelist_type edgelist_type;
  typedef typename _hei<reference>::type ref_property_type;

public:
  typedef typename internal_value_type::vertex_descriptor vertex_descriptor;
  typedef typename internal_value_type::property_type property_type;
  typedef typename _hei_it<reference>::iterator adj_edge_iterator;
  typedef ve_view<edgelist_type, adj_edge_iterator> adj_edge_view;

  typedef typename _hei_usr_prop<reference>::user_property_type
    user_property_type;
  typedef typename property_type::predlist_type predlist_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a vertex reference based on the provided iterator.
  //////////////////////////////////////////////////////////////////////
  const_dpg_vertex_reference(BaseReference ref)
    : vertex_reference<BaseReference>(ref)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the user property. This provides transparent access
  /// to the user property from the actual property stored in the graph.
  //////////////////////////////////////////////////////////////////////
  user_property_type const& property() const
  { return this->m_ref->property().user_property(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the predecessors of this vertex.
  //////////////////////////////////////////////////////////////////////
  predlist_type const& predecessors() const
  { return this->m_ref->property().predecessors(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class to provide an adaptor for the vertex iterator over a
/// @ref directed_preds_graph.
/// @ingroup graphBaseUtil
/// @tparam BaseIterator The type of the vertex iterator.
/// @tparam UserProperty The type of the user vertex property.
//////////////////////////////////////////////////////////////////////
template <typename BaseIterator, typename UserProperty>
class dpg_vertex_iterator_adaptor
  : public boost::iterator_adaptor<dpg_vertex_iterator_adaptor<BaseIterator,
                                                               UserProperty>,
                                   BaseIterator>
{
private:
  typedef boost::iterator_adaptor<dpg_vertex_iterator_adaptor<BaseIterator,
                                                              UserProperty>,
                                  BaseIterator> base_type;
  typedef typename std::iterator_traits<BaseIterator>::value_type
    internal_value_type;
  typedef typename internal_value_type::edgelist_type edgelist_type;
  typedef vertex_pointer<dpg_vertex_reference<BaseIterator, UserProperty> > pointer_type;

public:
  typedef dpg_vertex_reference<BaseIterator, UserProperty> value_type;

  typedef typename edgelist_type::iterator adj_edge_iterator;
  typedef typename edgelist_type::const_iterator const_adj_edge_iterator;
  typedef typename internal_value_type::vertex_descriptor vertex_descriptor;
  typedef typename internal_value_type::property_type property_type;
  typedef typename property_type::predlist_type predlist_type;
  typedef typename property_type::user_property_type user_property_type;

  dpg_vertex_iterator_adaptor() = default;

  dpg_vertex_iterator_adaptor(BaseIterator iterator)
      : base_type(iterator)
  {
  }

  pointer_type operator->() const
  {
    return pointer_type(value_type(this->base_reference()));
  }

  value_type operator*() const
  {
    return value_type(this->base_reference());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class to provide a const adaptor for the vertex iterator over
/// a @ref directed_preds_graph.
/// @ingroup graphBaseUtil
/// @tparam BaseIterator The type of the vertex iterator.
/// @tparam UserProperty The type of the user vertex property.
//////////////////////////////////////////////////////////////////////
template <typename BaseIterator, typename UserProperty>
class const_dpg_vertex_iterator_adaptor : public boost::iterator_adaptor<
    const_dpg_vertex_iterator_adaptor<BaseIterator, UserProperty>, BaseIterator>
{
private:
  typedef boost::iterator_adaptor<
      const_dpg_vertex_iterator_adaptor<BaseIterator, UserProperty>,
      BaseIterator> base_type;

  typedef typename std::iterator_traits<BaseIterator>::value_type
    internal_value_type;
  typedef typename internal_value_type::edgelist_type edgelist_type;
  typedef vertex_pointer<dpg_vertex_reference<BaseIterator, UserProperty> > pointer_type;

public:
  typedef const_dpg_vertex_reference<BaseIterator, UserProperty> value_type;

  typedef typename edgelist_type::const_iterator adj_edge_iterator;
  typedef typename edgelist_type::const_iterator const_adj_edge_iterator;
  typedef typename internal_value_type::vertex_descriptor vertex_descriptor;
  typedef typename internal_value_type::property_type property_type;
  typedef typename property_type::predlist_type predlist_type;
  typedef typename property_type::user_property_type user_property_type;

  const_dpg_vertex_iterator_adaptor() = default;

  const_dpg_vertex_iterator_adaptor(BaseIterator iterator)
      : base_type(iterator)
  {
  }

  //iterator and const_iterator interoperability
  template <typename _Iter>
  inline const_dpg_vertex_iterator_adaptor(
      const dpg_vertex_iterator_adaptor<_Iter, UserProperty>& _other)
      : base_type(_other.base())
  {
  }

  pointer_type operator->() const
  {
    return pointer_type(value_type(this->base_reference()));
  }

  value_type operator*() const
  {
    return value_type(this->base_reference());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Selector class for vertex iterator for @ref directed_preds_graph.
/// @ingroup graphBaseUtil
/// @tparam Iterator The type of the iterator.
/// @tparam VD The type of the vertex descriptor.
/// @tparam UserProperty The type of the user vertex property.
//////////////////////////////////////////////////////////////////////
template <class Iterator, class VD, class UserProperty>
struct select_vertex_iterator<Iterator, preds_property<VD, UserProperty> >
{
  typedef dpg_vertex_iterator_adaptor<Iterator, UserProperty> type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Selector class for vertex iterator for @ref directed_preds_graph.
/// @ingroup graphBaseUtil
/// @tparam Iterator The type of the iterator.
/// @tparam VD The type of the vertex descriptor.
/// @tparam UserProperty The type of the user vertex property.
//////////////////////////////////////////////////////////////////////
template <class Iterator, class VD, class UserProperty>
struct select_const_vertex_iterator<Iterator, preds_property<VD, UserProperty> >
{
  typedef const_dpg_vertex_iterator_adaptor<Iterator, UserProperty> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Class implementing a directed graph with predecessors.
/// @ingroup graph
///
/// A directed predecessor graph extends the graph hierarchy
/// with methods to maintain predecessors. Methods like add/delete
/// vertex/edges add some overhead.
/// The directed with predecessors graph is an extension of the STAPL graph.
/// The property associated with each vertex is an extended property
/// where one of the fields is the list of predecessors. The directed
/// with predecessors graph will overload add/delete vertex/edge to
/// update this field.
/// The vertex property is wrapped inside an internal property that is
/// used to track predecessors info. For most part this is invisible
/// to the user.
/// @tparam M graph-attribute specifying Multiedge. (MULTIEDGES/NONMULTIEDGES).
/// @tparam VertexP type of property for the vertex. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam EdgeP type of property for the edge. Default is no_property.
/// Must be default assignable, copyable and assignable.
/// @tparam Traits A traits class that defines customizable components
/// of graph, such as the descriptor, model, storage, etc. The default traits
/// class is @ref adj_graph_traits.
//////////////////////////////////////////////////////////////////////
template <graph_attributes M, class VertexP = properties::no_property,
          class EdgeP = properties::no_property>
class directed_preds_graph
  : public graph<DIRECTED, M,
                 preds_property<size_t, VertexP>, EdgeP,
                 adj_graph_traits<DIRECTED, M,
                                  preds_property<size_t, VertexP>, EdgeP> >
{

  typedef graph<DIRECTED, M, preds_property<size_t, VertexP>, EdgeP,
      adj_graph_traits<DIRECTED, M,
                       preds_property<size_t, VertexP>, EdgeP> > base_type;

protected:
  /// When set true the methods are not updating the predecessors info.
  bool m_lazy_update;
  /// Flag specifying if the predecessors info is not updated.
  bool m_needs_update;

public:
  typedef typename base_type::vertex_descriptor vertex_descriptor;
  typedef typename base_type::edge_descriptor edge_descriptor;
  typedef typename base_type::vertex_iterator vertex_iterator;
  typedef typename base_type::const_vertex_iterator const_vertex_iterator;

  typedef typename base_type::adj_edge_iterator adj_edge_iterator;
  typedef typename base_type::const_adj_edge_iterator const_adj_edge_iterator;

  typedef typename base_type::edge_property edge_property;
  typedef preds_property<size_t, VertexP> vertex_property;
  typedef typename vertex_property::predlist_type::iterator preds_iterator;
  typedef typename vertex_property::predlist_type::const_iterator
    const_preds_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a graph with the specified number of vertices.
  ///
  /// Lazy updating of predecessor-info is disabled by default.
  //////////////////////////////////////////////////////////////////////
  directed_preds_graph(size_t sz = 0)
    : base_type(sz), m_lazy_update(false), m_needs_update(false)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of incident edges on the specified vertex.
  //////////////////////////////////////////////////////////////////////
  size_t get_in_degree(vertex_descriptor const& vd) const
  {
    const_vertex_iterator cvi = this->find_vertex(vd);
    if (cvi != this->end())
      return (*cvi).predecessors().size();
    else
      return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the predecessors of the specified vertex.
  /// @param vd The specified vertex.
  /// @param preds The output list of predecessors.
  /// @return The number of predecessors.
  //////////////////////////////////////////////////////////////////////
  size_t get_predecessors(vertex_descriptor const& vd,
                          std::vector<vertex_descriptor>& preds) const
  {
    preds.clear();
    const_vertex_iterator cvi = this->find_vertex(vd);
    if (cvi != this->end()) {
      const_preds_iterator cei = (*cvi).predecessors().begin();
      while (cei != (*cvi).predecessors().end()) {
        preds.push_back(*cei);
        ++cei;
      }
      return preds.size();
    } else
      return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the specified edge to the graph.
  ///
  /// This is specialized to add the proper predecessors info as well.
  /// @param ed The descriptor of the edge to be added.
  /// @return The edge descriptor of the added edge. If successful, the
  /// id() of the descriptor is populated with the actual edge's id, otherwise
  /// set to std::numeric_limits<size_t>::max().
  /// @note If lazy update is set to true, the predecessor-info is not updated,
  /// but the @ref m_needs_update flag is set, so this can be updated later
  /// in batch.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed)
  {
    return add_edge(ed, edge_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the specified edge with the given property to the graph.
  ///
  /// This is specialized to add the proper predecessors info as well.
  /// @param ed The descriptor of the edge to be added.
  /// @param p The edge property.
  /// @return The edge descriptor of the added edge. If successful, the
  /// id() of the descriptor is populated with the actual edge's id, otherwise
  /// set to std::numeric_limits<size_t>::max().
  /// @note If lazy update is set to true, the predecessor-info is not updated,
  /// but the @ref m_needs_update flag is set, so this can be updated later
  /// in batch.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(edge_descriptor const& ed, edge_property const& p)
  {
    //add the directed edge:
    edge_descriptor res = base_type::add_edge(ed, p);
    //add pred edge:
    if (res.id() != (size_t) INVALID_VALUE ) {
      if (!m_lazy_update) {
        vertex_iterator vi = this->find_vertex(ed.target());
        if (vi != this->end())
          (*vi).predecessors().push_back(ed.source());
        else {
          //assert maybe
          std::cout
              << "Directed with predecessors::add_edge unexpected results\n";
        }
      } else
        m_needs_update = true;
    }
    return res;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the edge with the specified source and target, with a
  /// default property to the graph.
  ///
  /// This is specialized to add the proper predecessors info as well.
  /// @param source The vertex descriptor of the source of the edge to be added.
  /// @param target The vertex descriptor of the target of the edge to be added.
  /// @return The edge descriptor of the added edge. If successful, the
  /// id() of the descriptor is populated with the actual edge's id, otherwise
  /// set to std::numeric_limits<size_t>::max().
  /// @note If lazy update is set to true, the predecessor-info is not updated,
  /// but the @ref m_needs_update flag is set, so this can be updated later
  /// in batch.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(vertex_descriptor const& source,
                           vertex_descriptor const& target)
  {
    edge_descriptor ed(source, target);
    return this->add_edge(ed, edge_property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the edge with the specified source and target, with the
  /// given property to the graph.
  ///
  /// This is specialized to add the proper predecessors info as well.
  /// @param source The vertex descriptor of the source of the edge to be added.
  /// @param target The vertex descriptor of the target of the edge to be added.
  /// @param p The edge property.
  /// @return The edge descriptor of the added edge. If successful, the
  /// id() of the descriptor is populated with the actual edge's id, otherwise
  /// set to std::numeric_limits<size_t>::max().
  /// @note If lazy update is set to true, the predecessor-info is not updated,
  /// but the @ref m_needs_update flag is set, so this can be updated later
  /// in batch.
  //////////////////////////////////////////////////////////////////////
  edge_descriptor add_edge(vertex_descriptor const& source,
                           vertex_descriptor const& target,
                           edge_property const& p)
  {
    edge_descriptor ed(source, target);
    return this->add_edge(ed, p);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the specified edge, while maintaining predecessors info.
  /// @return True if the delete was successful, or false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool delete_edge(edge_descriptor const& ed)
  {
    bool res = base_type::delete_edge(ed);    //delete the directed edge;
    //here add pred edge
    if (res) {
      if (!this->m_lazy_update) {
        vertex_iterator vi = this->find_vertex(ed.target());
        //find and erase
        preds_iterator pi = std::find((*vi).predecessors().begin(),
                                      (*vi).predecessors().end(), ed.source());
        if (pi != (*vi).predecessors().end())
          (*vi).predecessors().erase(pi);
        else {
          std::cout << "ERROR:: something is not right while deleting edge\n";
          res = false;
        }
      } else
        m_needs_update = true;
    }
    return res;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the specified edge identified by its source and target,
  /// while maintaining predecessors info.
  /// If there are multiple edges available, one of them (unspecified) will
  /// be removed.
  /// @return True if the delete was successful, or false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool delete_edge(vertex_descriptor const& source,
                   vertex_descriptor const& target)
  {
    return this->delete_edge(edge_descriptor(source, target));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the specified vertex and all edges pointing to it
  /// and from it, while maintaining predecessors info.
  /// @return True if vertex was deleted, or false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool delete_vertex(vertex_descriptor const& vd)
  {
    //delete preds for all outgoing vertices
    vertex_iterator vi = base_type::find_vertex(vd);
    adj_edge_iterator ei = (*vi).begin();
    while (ei != (*vi).end()) {
      vertex_iterator vi2 = this->find_vertex((*ei).target());
      if (!m_lazy_update) {
        preds_iterator pi;
        pi = std::find((*vi2).predecessors().begin(),
                       (*vi2).predecessors().end(), vd);
        while (pi != (*vi2).predecessors().end()) {
          (*vi2).predecessors().erase(pi);
          pi = std::find((*vi2).predecessors().begin(),
                         (*vi2).predecessors().end(), vd);
        }
      } else
        m_needs_update = true;
      ++ei;
    }
    //call the base to remove the vertex and the edges
    bool res = base_type::delete_vertex(vd);
    return res;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the graph in the lazy/non lazy update mode.
  ///
  /// If lazy update is not set, graph operations will keep the predecessor
  /// information up-to-date with each operation.
  /// If lazy update is set, graph operations will not update the predecessor
  /// information, instead relying on the user to call the
  /// @ref set_predecessors() method to update all predecessor information at
  /// once, before use.
  //////////////////////////////////////////////////////////////////////
  void set_lazy_update(bool f)
  {
    if (f) {
      this->m_lazy_update = true;
    } else {
      //Set the graph in the always update mode
      set_predecessors();
      this->m_lazy_update = false;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// When in the lazy update mode this method should be called to update
  /// all predecessors at once.
  //////////////////////////////////////////////////////////////////////
  void set_predecessors(void)
  {
    if (this->m_needs_update) {
      for (vertex_iterator vi = this->begin(); vi != this->end(); ++vi) {
        (*vi).predecessors().clear();
      }
      for (vertex_iterator vi = this->begin(); vi != this->end(); ++vi) {
        for (adj_edge_iterator ei = (*vi).begin(); ei != (*vi).end(); ++ei) {
          //find the vertex corresponding to the target and add pred info
          vertex_iterator vi2 = this->find_vertex((*ei).target());
          assert(vi2 != this->end());
          //this should never happen
          (*vi2).predecessors().push_back((*ei).source());
        }
      }
      this->m_needs_update = false;
    }    //else return
  }

};

} // namespace sequential
} // namespace stapl

#endif
