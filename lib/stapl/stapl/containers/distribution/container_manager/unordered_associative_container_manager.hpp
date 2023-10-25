/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BOOST_PP_IS_ITERATING

#ifndef STAPL_CONTAINERS_UNORDERED_ASSOCIATIVE_CONTAINER_MANAGER_HPP
#define STAPL_CONTAINERS_UNORDERED_ASSOCIATIVE_CONTAINER_MANAGER_HPP

#include <boost/preprocessor/repetition/enum_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>
#include <boost/preprocessor/repetition/repeat.hpp>
#include <boost/preprocessor/control/if.hpp>
#include <boost/preprocessor/comparison/equal.hpp>
#include <boost/preprocessor/tuple/eat.hpp>
#include <boost/preprocessor/iteration/iterate.hpp>
#include <boost/preprocessor/punctuation/comma_if.hpp>

#include "container_manager.hpp"
#include "ordering/ordering_functors.hpp"
#include <stapl/utility/invoke_arg.hpp>
#include <stapl/utility/tuple.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief The container manager is responsible for the local metadata of
///   the elements stored on this location.
///
///   It knows in which local base containers elements reside.
///   It also provides methods to invoke base container methods on specific
///   elements, abstracting out the need for external classes to know exactly
///   in which base container an element is stored. This container-manager only
///   supports one base-container per location,as multiple base-containers are
///   not needed for the unordered map.
/// @tparam BContainer The base container class.
/// @todo For completeness, implement a const_invoke that does not have a
/// return value.
//////////////////////////////////////////////////////////////////////
template<typename BContainer, typename Manager>
class unordered_associative_container_manager
  : public container_manager<BContainer>
{
  typedef container_manager<BContainer>                    base_type;

public:
  typedef BContainer                                       base_container_type;
  typedef typename BContainer::gid_type                    gid_type;
  typedef typename BContainer::mapped_type                 mapped_type;
  typedef typename BContainer::cid_type                    cid_type;
  typedef typename BContainer::stored_type                 stored_type;
  typedef typename BContainer::value_type                  value_type;
  typedef typename BContainer::hasher                      hasher;
  typedef typename BContainer::key_equal                   key_equal;

protected:
  typedef typename base_type::storage_type                 storage_type;
  typedef typename base_type::interval_type                interval_type;
  typedef base_container_ordering                          ordering_type;
  typedef typename ordering_type::ptr_bcontainer_type      ptr_bcontainer_type;

  Manager                                                  m_manager;
  hasher                                                   m_hash;
  key_equal                                                m_key_eq;

public:
  STAPL_IMPORT_TYPE(typename storage_type, iterator)
  STAPL_IMPORT_TYPE(typename storage_type, const_iterator)

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that initializes the initial metadata for the base
  ///   container.
  /// @param partition Partition object that translates elements to CIDs
  /// @param mapper Mapper object that translates CIDs to locations.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper>
  unordered_associative_container_manager(Partition const& partition,
                                          Mapper const& mapper,
                                          hasher const& hash = hasher(),
                                          key_equal const& key_eq = key_equal())
    : base_type(partition, mapper, boost::mpl::false_()),
      m_manager(partition, mapper), m_hash(hash), m_key_eq(key_eq)
  {
    typename Partition::value_type hrange;

    base_container_type* bc = this->insert_range(
      hrange.first(), hrange.last(),
      [=]() { return new base_container_type(hash, key_eq); }
    );

    this->m_ordering.insert(0, bc);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the first gid in the domain of the base container.
  /// @todo Probably needs to be moved to a different class/file operation
  ///   required for the domain interface.
  //////////////////////////////////////////////////////////////////////
  gid_type first() const
  {
    base_container_type* bc = this->m_ordering.first();

    if (bc != NULL) {
      if (!bc->domain().empty())
        return bc->domain().first();
    }

    // return invalid gid
    return index_bounds<gid_type>::invalid();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the last gid in the domain of the base container.
  //////////////////////////////////////////////////////////////////////
  gid_type last() const
  {
    base_container_type* bc = this->m_ordering.last();

    if (bc != NULL)
    {
      if (!bc->domain().empty())
        return bc->domain().last();
    }

    // return invalid gid
    return index_bounds<gid_type>::invalid();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the gid at a distance from another gid.
  /// @param g the gid of reference
  /// @param n the distance to advance
  //////////////////////////////////////////////////////////////////////
  gid_type advance(gid_type const& g, long long n)
  {
    if (n == 0)
      return g;

    typename storage_type::iterator it = this->begin();
    stapl_assert(it != this->end(), "gid not found");
    promise<gid_type> p;

    if (n > 0)
    {
      typedef ordering_detail::advance_fw<
        gid_type, base_container_type
      > advance_fw_t;

      advance_fw_t advfw(g, n, p);
      this->m_ordering.traverse_forward(advfw, &(*it));
    }
    else
    {
      typedef ordering_detail::advance_bw<
        gid_type, base_container_type
      > advance_bw_t;

      advance_bw_t advbw(g, n, p);
      this->m_ordering.traverse_backward(advbw, &(*it));
    }

    return p.get_future().get(); // sync_rmi() equivalent
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether a gid is present in the base container for which
  ///   this container_manager is responsible.
  /// @todo Why aren't we just using icl::contains() / m_bcontainer.find()?
  //////////////////////////////////////////////////////////////////////
  bool contains(gid_type const& g) const
  {
    return (this->size()!=0 && (this->begin()->find(g)!=this->begin()->end()));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns which base container a specified GID is in.
  //////////////////////////////////////////////////////////////////////
  cid_type within(gid_type const& gid) const
  {
    return this->m_ordering.begin()->second->cid();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element with the specified GID.
  /// @param gid GID of the element on which to apply the functor.
  /// @param f Functor to apply.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply(gid_type const& gid, Functor const& f)
  {
    stapl_assert(this->size() != 0,
                 "invoking a function on an unknown base container");

    this->begin()->apply(gid,f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element with the specified GID.
  /// @param gid GID of the element on which to apply the functor.
  /// @param f Functor to apply.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply_set(gid_type const& gid, Functor const& f)
  {
    stapl_assert(this->size() != 0,
                 "invoking a function on an unknown base container");

    this->begin()->apply_set(gid, f);
  }

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element at the position GID if
  ///        the element exists at this location.
  /// @param gid GID of the element on which to apply the functor.
  /// @param f Functor to apply.
  //////////////////////////////////////////////////////////////////////////////
  template<typename Functor>
  bool contains_apply(gid_type const& gid, Functor const& f)
  {
    typedef typename storage_type::const_iterator iter_t;
    const iter_t iter = this->m_ordering.find(gid);

    if (iter == this->ordering.end())
      return false;

    iter->second->apply(gid,f);
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element with the specified GID.
  /// @param gid GID of the element on which to apply the functor.
  /// @param f Functor to apply.
  /// @return The result of the functor operation.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f)
  {
    stapl_assert(this->size() != 0,
                 "invoking a function on an unknown base container");

    return this->begin()->apply_get(gid, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Find the first base container over all the base containers
  /// @param promise The promise that will be updated once the first is found.
  /// @note This function and find_last are needed for the distributed domain.
  //////////////////////////////////////////////////////////////////////
  template<typename Prom>
  void find_first(Prom const& promise)
  {
    typedef ordering_detail::find_fw<
      Prom, ordering_type, base_container_type
    >                                                        visitor_type;

    visitor_type findfw(promise, &(this->m_ordering));

    if (this->m_ordering.get_location_id() != 0)
    {
      typedef void (ordering_type::* mem_fun_t)(visitor_type&, bc_base*, bool)
      const;

      const mem_fun_t mem_fun = &ordering_type::traverse_forward;

      async_rmi(0, this->m_ordering.get_rmi_handle(),
                mem_fun, findfw, nullptr, false);
    }
    else
      this->m_ordering.traverse_forward(findfw, this->m_ordering.first());
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Find the last base container over all the base containers
  /// @param promise The promise that will be updated once the last is found.
  /// @note This function and find_first are needed for the distributed domain.
  //////////////////////////////////////////////////////////////////////
  template<typename Prom>
  void find_last(Prom const& promise)
  {
    typedef ordering_detail::find_bw<
      Prom, ordering_type, base_container_type
    >                                                        visitor_type;

    visitor_type findbw(promise, &(this->m_ordering));

    const size_t last_loc = this->m_ordering.get_num_locations()-1;

    if (this->m_ordering.get_location_id() != last_loc)
    {
      typedef void (ordering_type::* mem_fun_t)(visitor_type&, bc_base*, bool)
      const;

      const mem_fun_t mem_fun = &ordering_type::traverse_backward;

      async_rmi(last_loc, this->m_ordering.get_rmi_handle(),
                mem_fun, findbw, nullptr, false);
    }
    else
      this->m_ordering.traverse_backward(findbw, this->m_ordering.last());
  }

  ///////////////////////////////////////////////////////////////////////
  /// @brief Invokes the erase function of the base container that contains
  ///   the target GID asynchronously.
  /// @param gid The GID to be erased
  /// @return True if an element was erased, false otherwise
  ///////////////////////////////////////////////////////////////////////
  bool erase(gid_type const& gid)
  {
    const iterator iter = this->find(gid);

    if (iter != this->end())
    {
      iter->erase(gid);
      return true;
    }
    return false;
  }

  ///////////////////////////////////////////////////////////////////////
  /// @brief Invokes the erase function of the base container that contains
  ///   the target GID.
  /// @param gid The GID to be erased
  /// @return The number of elements erased
  ///////////////////////////////////////////////////////////////////////
  size_t erase_sync(gid_type const& gid)
  {
    const iterator iter = this->find(gid);

    if (iter != this->end())
      return iter->erase_sync(gid);

    //else
    return 0;
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Find or insert a base container for the current location
  /// @todo The key argument is useless here, consider removing it.
  ///   only insert calls find_or_insert_bc.
  /// @return The base container either newly created or already existing.
  /// @todo figure out where the base container needs to be placed
  ///   (connected to who)
  //////////////////////////////////////////////////////////////////////
  base_container_type& find_or_insert_bc(void)
  {
    const iterator iter = this->begin();

    if (iter != this->end())
      return *iter;

    // else
    std::pair<gid_type, gid_type> hrange = m_manager.range_of();

    base_container_type* bc = this->insert_range(
            hrange.first, hrange.second,
            [this]() { return new base_container_type(m_hash, m_key_eq); }
      );

    this->m_ordering.insert(0, bc);

    return *bc;
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts an element in the base container.  Implementation
  ///  for @ref set.  Overridden for @ref in @ref map_container_manager.
  //////////////////////////////////////////////////////////////////////
  void insert(stored_type const& s)
  {
    this->find_or_insert_bc().insert(s);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts an element in the base container.
  /// @param key The key of the element.
  /// @param val The mapped value of the element.
  //////////////////////////////////////////////////////////////////////
  void insert(gid_type const& gid, mapped_type const& m)
  {
    this->find_or_insert_bc().insert(stored_type(gid, m));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an entry for @p gid if one does not exist
  ///  and return true.  If one does exist, do nothing and return false.
  /// @param gid GID of the element that may be created
  /// @param comp_spec Optional parameter that can be provided when the
  /// element to be constructed is a container with a view-based distribution.
  //////////////////////////////////////////////////////////////////////
  bool try_create(gid_type const& gid, composed_dist_spec* comp_spec = nullptr)
  {
    return this->find_or_insert_bc().try_create(gid, comp_spec);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke the functor on the base container responsible for the
  /// specified @p gid, creating the base container if necessary.
  //////////////////////////////////////////////////////////////////////
  template<typename Class, typename Rtn, typename... PMFArgs, typename... Args>
  Rtn create_invoke(gid_type const& gid,
                    Rtn (Class::* pmf)(PMFArgs...),
                    Args&&... args)
  {
    return (find_or_insert_bc().*pmf)(std::forward<Args>(args)...);
  }

#ifndef CONTAINER_MAX_INVOKE_ARGS
  #define CONTAINER_MAX_INVOKE_ARGS 5
#endif

#define BOOST_PP_ITERATION_LIMITS (0, CONTAINER_MAX_INVOKE_ARGS)
#define BOOST_PP_FILENAME_1 "stapl/containers/distribution/container_manager/unordered_associative_container_manager.hpp"
#include BOOST_PP_ITERATE()

}; // class unordered_associative_container_manager

} // namespace stapl

#endif // STAPL_CONTAINERS_MANAGER_HPP

#else // BOOST_PP_IS_ITERATING

#define n BOOST_PP_ITERATION()

#define CONST_ref_invoke_arg(z, n, data) \
  const typename invoke_arg<BOOST_PP_CAT(Arg, n)>::type& BOOST_PP_CAT(arg, n)

#define MEM_func_arg_list(z)                  \
            BOOST_PP_IF(                      \
              BOOST_PP_EQUAL(n, 0),           \
              void BOOST_PP_TUPLE_EAT(2),     \
              BOOST_PP_ENUM_PARAMS            \
            )(n, Arg)


//////////////////////////////////////////////////////////////////////
/// @brief Invoke a base container method on the element with the specified
///   GID. The element must exist in the current base container manager.
/// @param gid The GID of the element to invoke the method on.
/// @param memberFuncPtr A pointer to a base container's member method.
//////////////////////////////////////////////////////////////////////
template<typename Class BOOST_PP_ENUM_TRAILING_PARAMS(n, typename Arg)>
void invoke(gid_type const& gid,
            void (Class::* memberFuncPtr)(MEM_func_arg_list(n))
            BOOST_PP_COMMA_IF(n)
            BOOST_PP_ENUM(n, CONST_ref_invoke_arg, ~))
{
  stapl_assert(this->size() != 0,
               "invoking a function on an unknown base container");

  ((*this->begin()).*memberFuncPtr)(BOOST_PP_ENUM_PARAMS(n, arg));
}

//////////////////////////////////////////////////////////////////////
/// @brief Invoke a base container method on the element with the specified
///   GID. The element must exist in the current base container manager.
/// @param gid The GID of the element to invoke the method on.
/// @param memberFuncPtr A pointer to a base container's member method.
/// @return The result of the invocation of the member function on the element.
//////////////////////////////////////////////////////////////////////
template<typename Class,
  typename Rtn BOOST_PP_ENUM_TRAILING_PARAMS(n, typename Arg)>
Rtn invoke(gid_type const& gid,
           Rtn (Class::* memberFuncPtr)(MEM_func_arg_list(n))
           BOOST_PP_COMMA_IF(n)
           BOOST_PP_ENUM(n, CONST_ref_invoke_arg, ~))
{
  stapl_assert(this->size() != 0,
    "invoking a function on an unknown base container");

//base_container_type* bc = this->begin();
//  stapl_assert(this->begin() != NULL,
//    "Bcontainer shouldn't be NULL");
  return ((*this->begin()).*memberFuncPtr)(BOOST_PP_ENUM_PARAMS(n, arg));
}

//////////////////////////////////////////////////////////////////////
/// @brief Invoke a base container method on the element with the specified
///   GID. The element must exist in the current base container manager.
/// @param gid The GID of the element to invoke the method on.
/// @param memberFuncPtr A pointer to a base container's member method.
/// @return The result of the invocation of the member function on the element.
//////////////////////////////////////////////////////////////////////
template<typename Class,
  typename Rtn BOOST_PP_ENUM_TRAILING_PARAMS(n, typename Arg)>
Rtn const_invoke(gid_type const& gid,
                 Rtn (Class::* const memberFuncPtr)(MEM_func_arg_list(n))
                 BOOST_PP_COMMA_IF(n)
                 BOOST_PP_ENUM(n, CONST_ref_invoke_arg, ~)) const
{
  stapl_assert(this->m_ordering.size() != 0,
               "invoking a function on an unknown base container");

  base_container_type* bc = this->m_ordering.begin()->second;
  return (bc->*memberFuncPtr)(BOOST_PP_ENUM_PARAMS(n, arg));
}

#undef MEM_func_arg_list
#undef CONST_ref_invoke_arg
#undef n

#endif // BOOST_PP_IS_ITERATING
