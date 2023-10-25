/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef BOOST_PP_IS_ITERATING

#ifndef STAPL_CONTAINERS_MANAGER_DYNAMIC_GRAPH_HPP
#define STAPL_CONTAINERS_MANAGER_DYNAMIC_GRAPH_HPP

#include "container_manager_static_graph.hpp"
#include <stapl/containers/distribution/container_manager/ordering/base_container_ordering.hpp>
#include <stapl/containers/distribution/container_manager/ordering/ordering_functors.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Base-container manager for the @ref dynamic_graph.
/// @tparam BaseContainer Type of the base-container.
/// @ingroup pgraphDist
///
/// This container manager stores only one base-container per location,
/// because it is used by unordered containers which don't need
/// multiple base-containers.
//////////////////////////////////////////////////////////////////////
template<typename BaseContainer, typename Registry>
class container_manager_dynamic_graph
  : public container_manager_static_graph<BaseContainer, Registry>
{
private:
  typedef container_manager_static_graph<BaseContainer, Registry> base_type;

public:
  typedef typename BaseContainer::cid_type             cid_type;
  typedef typename BaseContainer::gid_type             gid_type;
  typedef typename BaseContainer::value_type           value_type;
  typedef BaseContainer                                base_container_type;

  typedef typename base_type::iterator                 iterator;
  typedef typename base_type::const_iterator           const_iterator;

  /// Type for managing base container ordering.
  typedef base_container_ordering                      ordering_type;
  typedef typename ordering_type::ptr_bcontainer_type  ptr_bcontainer_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a base-container manager with the given size, partition
  /// and mapper, constructing all vertices with the value provided.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper>
  container_manager_dynamic_graph(Partition const& partition,
                                  Mapper const& mapper,
                                  value_type const& default_value=value_type())
    : base_type(partition, mapper, default_value)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an element with the given GID and value to the base-container.
  /// @param gid The GID of the element being added.
  /// @param val The value of the element being added.
  //////////////////////////////////////////////////////////////////////
  void add_element(gid_type const& gid, value_type const& val)
  {
    gid_type min_dist = std::numeric_limits<gid_type>::max();
    iterator min_bc = this->begin();
    iterator iter = this->begin();
    iterator iter_end = this->end();
    for (; iter != iter_end; ++iter)
    {
      if (gid - iter->domain().first() < min_dist)
      {
        min_dist = gid - iter->domain().first();
        min_bc = iter;
      }
    }
    min_bc->add_element(val);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the element with the specified GID.
  /// @param gid The GID of the element being removed.
  //////////////////////////////////////////////////////////////////////
  void remove_element(gid_type const& gid)
  {
    iterator iter     = this->begin();
    iterator iter_end = this->end();

    for (; iter != iter_end; ++iter)
    {
      if (iter->domain().contains(gid))
        break;
    }

    iter->suspend_vertex(gid);
  }
#if 0
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in the base-container managed.
  //////////////////////////////////////////////////////////////////////
  size_t num_elements() const
  {
    return this->m_base_container->size();
  }
#endif
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the GID of the global first element of the pContainer,
  /// as per the ordering specified by base-container ordering.
  /// This is needed for the distributed domain.
  //////////////////////////////////////////////////////////////////////
  template<typename Promise>
  void find_first(Promise const& promise)// const
  {
    ordering_detail::find_first<Promise,ordering_type, base_container_type>
      findfirst(promise, &(this->m_ordering));

    typedef void (ordering_type::*fn)
      (ordering_detail::find_first<Promise,ordering_type,base_container_type>&)
      const;

    location_type first_loc =
      this->m_ordering.m_first_loc != index_bounds<location_type>::invalid() ?
      this->m_ordering.m_first_loc : this->m_ordering.get_location_id();

    if (this->m_ordering.get_location_id()!=first_loc) {
      async_rmi(first_loc,this->m_ordering.get_rmi_handle(),
                (fn) &ordering_type::find_first, findfirst);
    } else {
      this->m_ordering.find_first(findfirst);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the GID of the global last element of the pContainer,
  /// as per the ordering specified by base-container ordering.
  /// This is needed for the distributed domain.
  //////////////////////////////////////////////////////////////////////
  template<typename Promise>
  void find_last(Promise const& promise)// const
  {
    ordering_detail::find_last<Promise,ordering_type, base_container_type>
      findlast(promise, &(this->m_ordering));

    typedef void (ordering_type::*fn)
      (ordering_detail::find_last<Promise,ordering_type,base_container_type>&)
      const;

    size_t last_loc =
      this->m_ordering.m_last_loc != index_bounds<location_type>::invalid() ?
      this->m_ordering.m_last_loc : this->m_ordering.get_location_id();

    if (this->m_ordering.get_location_id()!=last_loc) {
      async_rmi(last_loc,this->m_ordering.get_rmi_handle(),
                (fn) &ordering_type::find_last, findlast);
    } else {
      this->m_ordering.find_last(findlast);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the GID of the element that is the specified number of
  /// steps ahead than the provided GID, as per the ordering specified by
  /// base-container ordering.
  /// This is needed for the distributed domain.
  /// @param g The GID of the element to start from.
  /// @param n The number of steps to advance.
  /// @param globally Whether or not to advance globally.
  /// @return The GID of the element resulting from advancing the given GID
  /// by the specified number of steps.
  //////////////////////////////////////////////////////////////////////
  gid_type advance(gid_type const& g, long long n, bool globally=true)// const
  {
    if (n==0)
      return g;

    iterator iter = this->begin();
    iterator iter_end = this->end();

    for (; iter != iter_end; ++iter)
    {
      if (iter->domain().contains(g))
        break;
    }
    stapl_assert(iter!=this->end(),"gid not found");

    promise<gid_type> p;
    if (n>0) {
      ordering_detail::advance_fw<gid_type,base_container_type> advfw(g, n, p);
      this->m_ordering.traverse_forward(advfw, &(*iter));
    } else {
      ordering_detail::advance_bw<gid_type, base_container_type> advbw(g, n, p);
      this->m_ordering.traverse_backward(advbw, &(*iter));
    }

    return p.get_future().get(); // sync_rmi() equivalent
  }

#ifndef CONTAINER_MAX_INVOKE_ARGS
  #define CONTAINER_MAX_INVOKE_ARGS 5
#endif

#define BOOST_PP_ITERATION_LIMITS (0, CONTAINER_MAX_INVOKE_ARGS)
#define BOOST_PP_FILENAME_1 \
  "stapl/containers/distribution/container_manager/container_manager\
_dynamic_graph.hpp"
#include BOOST_PP_ITERATE()


}; // class container_manager_dynamic_graph

} // namespace stapl

#endif // STAPL_CONTAINERS_MANAGER_DYNAMIC_GRAPH_HPP

#else // BOOST_PP_IS_ITERATING

#define n BOOST_PP_ITERATION()

#define CONST_ref_invoke_arg(z, n, data) const \
typename invoke_arg<BOOST_PP_CAT(Arg, n)>::type& BOOST_PP_CAT(arg, n)
#define MEM_func_arg_list(z)                  \
            BOOST_PP_IF(                      \
              BOOST_PP_EQUAL(n, 0),           \
              void BOOST_PP_TUPLE_EAT(2),     \
              BOOST_PP_ENUM_PARAMS            \
            )(n, Arg)


//////////////////////////////////////////////////////////////////////
/// @brief Invoke a base container method on the base container
/// that minimizes a user-defined comparator over the base containers.
/// @param comp Comparator that receives two base containers and returns
/// the more optimal of the two.
/// @param memberFuncPtr A pointer to a base container's member method.
/// @return The result of invoking the function pointer
//////////////////////////////////////////////////////////////////////
template<typename Comparator, typename Class, typename Rtn
         BOOST_PP_COMMA_IF(n)
         BOOST_PP_ENUM_PARAMS(n, typename Arg)>
Rtn min_invoke(Comparator const& comp,
           Rtn (Class::* memberFuncPtr)(MEM_func_arg_list(n))
           BOOST_PP_COMMA_IF(n)
           BOOST_PP_ENUM(n, CONST_ref_invoke_arg, ~))
{

  if (this->begin() == this->end())
  { // If no base container was created
    size_t cid = stapl::get_location_id();

    value_type v;
    auto bc_fact = boost::bind(
           typename base_type::factory_type(),
           _1, _2, boost::cref(v)
         );

    stapl::indexed_domain<gid_type> domain;
    base_container_type* bc_ptr = this->insert_range(
          domain, [&]() { return bc_fact(domain, cid); }
        );


    // 1 BC per location

    // The following returns a nullptr if not found, passing [size_t(-1)] is OK
    // insert_after handles a nullptr of prev correctly
    auto* prev = this->m_ordering[cid - 1];
    this->m_ordering.insert_after(prev, bc_ptr, cid);
    this->m_ordering.m_is_ordered   = true;
    this->m_ordering.m_total_num_bc = stapl::get_num_locations();

    return (bc_ptr->*memberFuncPtr)(BOOST_PP_ENUM_PARAMS(n, arg));
  }
  else
  {
    base_container_type& bc =
      *std::min_element(this->begin(), this->end(), comp);
    return (bc.*memberFuncPtr)(BOOST_PP_ENUM_PARAMS(n, arg));
  }
}


#undef MEM_func_arg_list
#undef CONST_ref_invoke_arg
#undef n

#endif // BOOST_PP_IS_ITERATING
