/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MANAGER_STATIC_GRAPH_HPP
#define STAPL_CONTAINERS_MANAGER_STATIC_GRAPH_HPP

#include <boost/icl/interval_map.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/functional/factory.hpp>
#include <boost/icl/interval_set.hpp>

#include "local_partition_info.hpp"
#include <stapl/containers/distribution/distributor.hpp>
#include <stapl/containers/distribution/container_manager/ordering/base_container_ordering.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>
#include <stapl/utility/tuple.hpp>

namespace stapl {

namespace cm_impl {
 //////////////////////////////////////////////////////////////////////
/// @brief Functor implementing the assignment of vertices of
/// a base container using the base container's add_element method.
///
/// This is passed to the @ref distributor struct implementing container
/// redistribution.
//////////////////////////////////////////////////////////////////////
struct vp_assign
{
  template <typename BaseContainer, typename GID, typename Value>
  void operator()(BaseContainer* bc, GID id, Value&& val)
  {
    bc->vp_set(id, val.property());
  }
};
}

//////////////////////////////////////////////////////////////////////
/// @brief Base-container manager for the @ref stapl::graph.
/// @tparam BaseContainer Type of the base-container.
/// @tparam Registry Storage class for the base containers
/// @ingroup pgraphDist
//////////////////////////////////////////////////////////////////////
template<typename BaseContainer, typename Registry>
class container_manager_static_graph
  : public Registry
{
public:
  typedef BaseContainer                                     base_container_type;
  STAPL_IMPORT_TYPE(typename base_container_type,           cid_type)
  STAPL_IMPORT_TYPE(typename base_container_type,           gid_type)

protected:
  typedef Registry storage_type;

public:
  STAPL_IMPORT_TYPE(typename storage_type, iterator)
  STAPL_IMPORT_TYPE(typename storage_type, const_iterator)

  typedef base_container_ordering                           ordering_type;
  typedef typename ordering_type::ptr_bcontainer_type       ptr_bcontainer_type;
  typedef BaseContainer                                     component_type;

protected:
  STAPL_IMPORT_TYPE(typename base_container_type, value_type)
  STAPL_IMPORT_TYPE(typename Registry, factory_type)

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Encapsulates functionality used by assignment operator and copy
  /// constructor to perform deep copy of base container pointers stored in
  /// @p m_bcontainers.
  //////////////////////////////////////////////////////////////////////
  void clone_bcontainers(container_manager_static_graph const& other)
  {
    this->clone_apply(other, boost::bind(
      &ordering_type::replace, boost::ref(m_ordering), _1, _2
    ));
  }

public:
  /// @brief Handle ordering amongst base containers across all locations
  ordering_type  m_ordering;

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a base-container manager with the given size, partition
  /// and mapper, constructing all vertices with the value provided.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper>
  container_manager_static_graph(Partition const& partition,
                                 Mapper const& mapper,
                                 value_type const& default_value = value_type())
  {
    this->init(partition, mapper,
         boost::bind(
           factory_type(),
           _1, _2, boost::cref(default_value)
         ), m_ordering
    );

    // update the vertex descriptor generator to start after the last GID
    // in the global domain
    auto const max_vd = partition.global_domain().last();

    if (max_vd != index_bounds<gid_type>::invalid())
      for (auto& bc : *this)
        bc.update_next_descriptor(max_vd);
  }

  container_manager_static_graph(container_manager_static_graph const& other)
  {
    this->clone_bcontainers(other);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Performs redistribution of the container elements into a new
  /// set of bContainers on possibly different locations to match the
  /// distribution specified by the partition and mapper provided.
  /// @param partition provides methods necessary to map element GIDs to
  /// the ids of the partitions (bContainers) that will store them.
  /// @param mapper provides the methods necessary to map partition ids to
  /// the ids of the locations that will store the partitions.
  ///
  /// The method is only available when view-based partition and mapper
  /// instances are used.  The distributor instance is given the partition
  /// and mapper information in the constructor.  The distributor function
  /// operator is given the current set of bContainers and their ordering in
  /// order to perform the redistribution.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper>
  void redistribute(Partition const& partition, Mapper const& mapper,
         typename boost::enable_if<
           boost::mpl::and_<is_view_based<Partition>,
                            is_view_based<Mapper> > >::type* = 0)
  {
    // get local base container ids
    typedef typename Partition::value_type::index_type index_type;
    typedef boost::icl::interval_set<index_type> interval_set_type;
    typedef typename boost::icl::interval<index_type>::type interval_type;

    // fields are domain, partition id, destination location, source location
    typedef std::tuple<interval_set_type, typename Mapper::cid_type,
      location_type, std::vector<location_type>>  bc_info_type;

    typedef cm_impl::distributor<typename storage_type::storage_type,
              bc_info_type, cm_impl::vp_assign> distributor_type;

    distributor_type d(this->m_intervals, m_ordering);
    d(partition, mapper);

    m_ordering.m_total_num_bc = partition.size();
    m_ordering.m_is_ordered   = true;
    d.advance_epoch();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if the base-container managed by this base-container
  /// manager contains the element specified by the GID.
  //////////////////////////////////////////////////////////////////////
  bool contains(gid_type const& gid) const
  {
    return this->find(gid) != this->end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an element with the given GID and value to the base-container.
  /// @param gid The GID of the element being added.
  /// @param val The value of the element being added.
  /// @note Adding elements to the static graph is not allowed.
  //////////////////////////////////////////////////////////////////////
  void add_element(gid_type const& gid, value_type const& val)
  {
    stapl_assert(false,
                 "trying to add an element to a static base container manager");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the element with the specified GID.
  /// @param gid The GID of the element being removed.
  /// @note Removing elements from the static graph is not allowed.
  //////////////////////////////////////////////////////////////////////
  void remove_element(gid_type const& gid)
  {
    stapl_assert(false,
      "trying to remove an element from a static base container manager");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the ID of the base-container storing the element
  /// with the specified GID.
  /// @param gid The GID of the element.
  /// @return The ID of the base-container.
  //////////////////////////////////////////////////////////////////////
  cid_type within(gid_type const& gid) const
  {
    return rank(this->find(gid));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in the base-container managed.
  //////////////////////////////////////////////////////////////////////
  size_t num_elements(void) const
  {
    return std::accumulate(this->begin(), this->end(), 0,
                           [](size_t sum, base_container_type const& bc)
                           { return sum + bc.size(); }
                           );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of edges in the base-container managed.
  //////////////////////////////////////////////////////////////////////
  size_t num_edges(void) const
  {
    return std::accumulate(this->begin(), this->end(), 0,
                           [](size_t sum, base_container_type const& bc)
                           { return sum + bc.num_edges(); }
                           );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of self edges in the base-container managed.
  //////////////////////////////////////////////////////////////////////
  size_t num_self_edges(void) const
  {
    return std::accumulate(this->begin(), this->end(), 0,
                           [](size_t sum, base_container_type const& bc)
                           { return sum + bc.num_self_edges(); }
                           );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the global rank of this base container in the pContainer
  //////////////////////////////////////////////////////////////////////
  size_t rank(base_container_type* bc) //const
  {
    return m_ordering.get_rank(bc);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function f to the element specified by the GID.
  /// @param gid The GID of the element.
  /// @param f The Functor to apply on the element.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply(gid_type const& gid, Functor const& f)
  {
    stapl_assert(this->contains(gid),
                 "invoking a function on an unknown base container");

    this->find_expect(gid)->apply(gid, f);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element at position GID
  ///   if the element exists at this location.
  /// @param gid GID of the element on which to apply the functor.
  /// @param f Functor to apply.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  bool contains_apply_set(gid_type const& gid, Functor const& f)
  {
    const iterator iter = this->find(gid);
    if (iter == this->end())
      return false;

    iter->apply(gid, f);
    return true;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function f to the element specified by the GID.
  /// @param gid The GID of the element.
  /// @param f The Functor to apply on the element.
  /// @todo Find out if this method is needed, as apply() does the same thing.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply_set(gid_type const& gid, Functor const& f)
  {
    apply(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function f to the property of the edge specified by
  /// the edge descriptor.
  /// @param ed The edge descriptor of the edge.
  /// @param f The Functor to apply on the property of the edge.
  //////////////////////////////////////////////////////////////////////
  template<typename ED, typename Functor>
  void ep_apply(ED const& ed, Functor const& f)
  {
    this->find_expect(ed.source())->ep_apply(ed, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function f to the property of the edge specified by
  /// the edge descriptor and returns the result.
  /// @param ed The edge descriptor of the edge.
  /// @param f The Functor to apply on the property of the edge.
  //////////////////////////////////////////////////////////////////////
  template<typename ED, typename Functor>
  typename Functor::result_type ep_apply_get(ED const& ed, Functor const& f)
  {
    return this->find_expect(ed.source())->ep_apply_get(ed, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function f to the property of the edge specified by
  /// the edge descriptor, if it exists and returns whether or not the edge
  /// exists.
  /// @param ed The edge descriptor of the edge.
  /// @param f The Functor to apply on the property of the edge.
  /// @return True if the edge exists, false otherwise.
  //////////////////////////////////////////////////////////////////////
  template<typename ED, typename Functor>
  bool ep_find_apply(ED const& ed, Functor const& f)
  {
    return this->find(ed.source())->ep_find_apply(ed, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function f to the element specified by the GID,
  /// and returns the result.
  /// @param gid The GID of the element.
  /// @param f The Functor to apply on the element.
  /// @return The result of applying the functor to the element.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type apply_get(gid_type const& gid, Functor const& f)
  {
    stapl_assert(contains(gid),
                 "invoking a function on an unknown base container");

    return this->find_expect(gid)->apply_get(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function f to the element specified by the GID,
  /// and returns the result.
  /// @param gid The GID of the element.
  /// @param f The Functor to apply on the element.
  /// @return The result of applying the functor to the element.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f) const
  {
    stapl_assert(contains(gid),
                 "invoking a function on an unknown base container");

    return this->find_expect(gid)->apply_get(gid, f);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element at position GID
  ///   if the element exists at this location.
  /// @param gid GID of the element on which to apply the functor.
  /// @param f Functor to apply.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  boost::optional<typename Functor::result_type>
  contains_apply_get(gid_type const& gid, Functor const& f)
  {
    typedef boost::optional<typename Functor::result_type> optional_type;
    iterator it = this->find(gid);

    if (it == this->end())
      return optional_type();

    return optional_type(it->apply_get(gid, f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element at position GID
  ///   if the element exists at this location.
  /// @param gid GID of the element on which to apply the functor.
  /// @param f Functor to apply.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  boost::optional<typename Functor::result_type>
  contains_apply_get(gid_type const& gid, Functor const& f) const
  {
    typedef boost::optional<typename Functor::result_type> optional_type;
    const_iterator it = this->find(gid);

    if (it == this->end())
      return optional_type();

    return optional_type(it->apply_get(gid, f));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a base container method on a certain GID. The
  ///   element must exist in the current base container manager.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param u   Arguments to pass to the member function.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename... T, typename... U>
  void invoke(gid_type const& gid, void (C::* const& pmf)(T...), U&&... u)
  {
    stapl_assert(contains(gid), "failed to find gid in container manager");

    ((*this->find_expect(gid)).*pmf)(std::forward<U>(u)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invokes a base container method on the given @p gid if it is
  ///        present on this location.
  /// @param gid           The GID of the element to invoke the method on.
  /// @param memberFuncPtr A pointer to a base container's member method.
  /// @param u             Arguments to pass to the member function.
  /// @return True if gid was found and functor applied, otherwise false.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename... T, typename... U>
  bool
  contains_invoke(gid_type const& gid, void (C::* const& pmf)(T...), U&&... u)
  {
    iterator iter = this->find(gid);

    if (iter == this->end())
      return false;

    // else
    ((*iter).*pmf)(std::forward<U>(u)...);

    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a base container method on a certain GID and return
  /// the result. The element must exist in the current base container manager.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param u   Arguments to pass to the member function.
  /// @return    The result of invoking the function pointer.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename Rtn, typename... T, typename... U>
  Rtn invoke(gid_type const& gid, Rtn (C::* const& pmf)(T...), U&&... u)
  {
    stapl_assert(contains(gid), "failed to find gid in container manager");

    return ((*this->find_expect(gid)).*pmf)(std::forward<U>(u)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invokes a base container method on the given @p gid if it is
  ///        present on this location.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param u   Arguments to pass to the member function.
  /// @return boost::optional with result of invocation if element was found.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename Rtn, typename... T, typename... U>
  boost::optional<Rtn>
  contains_invoke(gid_type const& gid, Rtn (C::* const& pmf)(T...), U&&... u)
  {
    iterator iter = this->find(gid);

    if (iter == this->end())
      return boost::optional<Rtn>();

    // else
    return boost::optional<Rtn>(((*iter).*pmf)(std::forward<U>(u)...));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invokes a base container method on the given @p gid if it is
  ///        present on this location.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param args Arguments to @p pmf.
  /// @return boost::optional with result of invocation if element was found.
  //////////////////////////////////////////////////////////////////////
  template<typename Class, typename Rtn, typename... PMFArgs, typename... Args>
  boost::optional<Rtn>
  const_contains_invoke(gid_type const& gid,
                  Rtn (Class::* const pmf)(PMFArgs...) const,
                  Args&&... args) const
  {
    auto const iter = this->find(gid);

    if (iter == this->end())
      return boost::optional<Rtn>();

    stapl_assert(contains(gid), "base container for gid not found");

    return boost::optional<Rtn>(((*iter).*pmf)(std::forward<Args>(args)...));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a const base container method on a certain GID and return
  /// the result. The element must exist in the current base container manager.
  /// @param gid The GID of the element to invoke the method on.
  /// @param pmf A pointer to a base container's member method.
  /// @param u   Arguments to pass to the member function.
  /// @return    The result of invoking the function pointer.
  //////////////////////////////////////////////////////////////////////
  template<typename C, typename Rtn, typename... T, typename... U>
  Rtn
  const_invoke(gid_type const& gid, Rtn (C::* const& pmf)(T...), U&&... u) const
  {
    stapl_assert(contains(gid), "failed to find gid in container manager");

    return ((*this->find_expect(gid)).*pmf)(std::forward<U>(u)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a base container method returning void on the base container
  /// that minimizes a user-defined comparator over the base containers.
  /// @param comp Comparator that receives two base containers and returns
  /// the more optimal of the two.
  /// @param pmf A pointer to a base container's member method.
  /// @param u   Arguments to pass to the member function.
  //////////////////////////////////////////////////////////////////////
  template<typename Comparator, typename C, typename... T, typename... U>
  void min_invoke(Comparator const& comp,
                  void (C::* const& pmf)(T...), U&&... u)
  {
    return ((*std::min_element(this->begin(), this->end(), comp)).*pmf)
      (std::forward<U>(u)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke a base container method on the base container
  /// that minimizes a user-defined comparator over the base containers.
  /// @param comp Comparator that receives two base containers and returns
  /// the more optimal of the two.
  /// @param pmf A pointer to a base container's member method.
  /// @param u   Arguments to pass to the member function.
  /// @return    The result of invoking the function pointer.
  //////////////////////////////////////////////////////////////////////
  template<typename Comparator, typename C, typename Rtn,
           typename... T, typename... U>
  Rtn min_invoke(Comparator const& comp, Rtn (C::* const& pmf)(T...), U&&... u)
  {
    return ((*std::min_element(this->begin(), this->end(), comp)).*pmf)
      (std::forward<U>(u)...);
  }
}; // class container_manager_static_graph

} // namespace stapl

#endif // STAPL_CONTAINERS_MANAGER_STATIC_GRAPH_HPP
