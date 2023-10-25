/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BOOST_PP_IS_ITERATING

#ifndef STAPL_CONTAINERS_CM_HEAP_HPP
#define STAPL_CONTAINERS_CM_HEAP_HPP

#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/repetition.hpp>
#include <boost/preprocessor/repetition/enum_params.hpp>
#include <boost/preprocessor/control/if.hpp>
#include <boost/preprocessor/comparison/equal.hpp>
#include <boost/preprocessor/tuple/eat.hpp>
#include <boost/preprocessor/iteration/iterate.hpp>
#include <boost/preprocessor/punctuation/comma_if.hpp>

#include "ordering/base_container_ordering.hpp"
#include "ordering/ordering_functors.hpp"

#include <stapl/utility/invoke_arg.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Container manager used for the @ref heap container.
/// @tparam Container The heap base container type.
//////////////////////////////////////////////////////////////////////
template<typename Container>
class container_manager_heap
{
public:
  typedef typename Container::gid_type                 gid_type;
  typedef typename Container::value_type               value_type;
  typedef Container                                    base_container_type;
  typedef typename std::vector<base_container_type*>   vector_bc_type;
  typedef typename vector_bc_type::iterator            iterator;
  typedef typename vector_bc_type::const_iterator      const_iterator;

  // base container ordering
  typedef base_container_ordering                      ordering_type;
  typedef typename ordering_type::ptr_bcontainer_type  ptr_bcontainer_type;

  typedef std::pair<gid_type, value_type>       pair_type;

protected:
  /// Vector of local base container pointer.
  vector_bc_type m_vec_bcs;

  /// Ordering between containers used for iteration.
  ordering_type  m_ordering;

public:

  //Default constructor needed by heap.hpp:139
  container_manager_heap(void)
   : m_vec_bcs(), m_ordering()
  { }

  template<typename Partition, typename Mapper>
  container_manager_heap(Partition const& partition, Mapper const& mapper)
  {
    //TODO Take care of parameters
  }

  container_manager_heap(container_manager_heap const& other)
  {
    base_container_type* prev_bc = 0;
    for (const_iterator it = other.begin() ; it != other.end() ; ++it)
    {
      base_container_type* bc = new base_container_type(*(*it));
      m_vec_bcs.push_back(bc);
      m_ordering.insert_after(prev_bc,bc);
      prev_bc = bc;
    }
  }

  container_manager_heap& operator=(container_manager_heap const& other)
  {
    if (this != other)
    {
      clear();
      base_container_type* prev_bc = 0;
      for (const_iterator it = other->begin(); it != other->end() ; ++it)
      {
        base_container_type* bc = new base_container_type();
        m_vec_bcs.push_back(bc);
        m_ordering.insert_after(prev_bc,bc);
        prev_bc = bc;
      }
    }
    return *this;
  }

  ~container_manager_heap(void)
  {
    clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a vector of all local base containers pointers.
  //////////////////////////////////////////////////////////////////////
  vector_bc_type get_bcs(void) const
  {
    return m_vec_bcs;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the ordering between all local base containers.
  //////////////////////////////////////////////////////////////////////
  ordering_type get_ordering(void) const
  {
    return m_ordering;
  }

  //////////////////////////////////////////////////////////////////////
  /// @bried Clears the storage and destroys the metadata for all
  ///        local elements.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
   for (iterator it = m_vec_bcs.begin() ; it != m_vec_bcs.end() ; ++it)
   {
     //FIXME remove of ordering not working, uncomment when fixed
     //[#857] remove methods of container ordering is bugged
     //m_ordering.remove(*it);
     delete *it;
   }
   m_vec_bcs.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the total number of local elements.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    size_t num = 0;
    for (const_iterator it = m_vec_bcs.begin(); it != m_vec_bcs.end() ; ++it)
      num += (*it)->size();
    return num;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of local base containers.
  //////////////////////////////////////////////////////////////////////
  size_t num_bc(void) const
  {
    return m_vec_bcs.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a new empty base container locally
  //////////////////////////////////////////////////////////////////////
  void push_bc(void)
  {
    base_container_type* bc = new base_container_type();
    m_vec_bcs.push_back(bc);
    m_ordering.insert_after(m_ordering.last(),bc);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a base container into the container manager.
  /// @param bc The base container to added.
  //////////////////////////////////////////////////////////////////////
  void push_bc(base_container_type* bc)
  {
    m_vec_bcs.push_back(bc);
    m_ordering.insert_after(m_ordering.last(),bc);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erase a base container at a given position.
  /// @param pos Position to remove element.
  //////////////////////////////////////////////////////////////////////
  iterator erase(iterator pos)
  {
    return m_vec_bcs.erase(pos);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper method to compute the distance between the two
  ///   given gids (@p a and @p b). Result is set in @ref promise @p p.
  ///
  /// This method uses the base container ordering methods to traverse
  /// the base containers and the visitor functor @ref distance_fw.
  //////////////////////////////////////////////////////////////////////
  void defer_distance(gid_type const& gid_a, gid_type const& gid_b,
                      promise<size_t> p) const
  {
    stapl_assert(contains(gid_a), "gid not found");

    base_container_type* const bca = gid_a.m_base_container;

    if (gid_a.base_container_equal(gid_b))
    {
      p.set_value(bca->domain().distance(gid_a, gid_b) + 1);
      return;
    }

    typedef ordering_detail::distance_fw<
      gid_type, base_container_type
    > distance_fw_t;

    this->m_ordering.traverse_forward(distance_fw_t(gid_a, gid_b, p), bca);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Search for a gid in a given range.
  /// @param gid_a The first GID.
  /// @param gid_b The second GID.
  /// @param g The GID to search for.
  /// @return Indicated if the GID already exist or if it has a future set.
  //////////////////////////////////////////////////////////////////////
  bool search(gid_type const& gid_a, gid_type const& gid_b, gid_type const& g)
  {
    promise<bool> p;

   typedef ordering_detail::search_fw<gid_type, base_container_type> visitor_t;

    this->m_ordering.traverse_forward(
      visitor_t(gid_a, gid_b, g, p), gid_a.m_base_container);

    return p.get_future().get();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether or not this base container manager
  ///        is responsible for an element.
  /// @param gid The GID to look for.
  /// @return Indicates if the container manager is in charge of a GID.
  //////////////////////////////////////////////////////////////////////
  bool contains(gid_type const& gid) const
  {
    return (gid.m_location == get_location_id());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a local iterator to the first base
  ///        container pointer of the container manager.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return m_vec_bcs.begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a local iterator to one position past the
  ///        last base container pointer of the container manager.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return m_vec_bcs.end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a const local iterator to the first base container
  ///        pointer of the container manager.
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return m_vec_bcs.begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a const local iterator to one position past the last
  ///        base container pointer of the container manager.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return m_vec_bcs.end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the first GID of the local domain.
  //////////////////////////////////////////////////////////////////////
  gid_type first(void) const
  {
    base_container_type* bc = this->m_ordering.first();
    if (bc!=0) {
      if (!bc->domain().empty())
        return bc->domain().first();
    }
    return index_bounds<gid_type>::invalid();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the last GID of the local domain.
  //////////////////////////////////////////////////////////////////////
  gid_type last(void) const
  {
    base_container_type* bc = this->m_ordering.last();

    if (bc!=0)
      if (!bc->domain().empty())
        return bc->domain().last();

    return index_bounds<gid_type>::invalid();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Find the first GID to define the boundaries of the container's
  ///        domain. The GID is returned using a promise.
  /// @tparam Promise The promise type.
  /// @param promise The promise to return the first GID.
  //////////////////////////////////////////////////////////////////////
  template<typename Promise>
  void find_first(Promise const& promise) const
  {
    typedef ordering_detail::find_fw<
      Promise, ordering_type, base_container_type
    >                                                 visitor_type;

    visitor_type findfw(promise, &(this->m_ordering));

    typedef void (ordering_type::*fn)(visitor_type&, bc_base*, bool) const;

    if (m_ordering.get_location_id()!=0)
    {
      async_rmi(0, m_ordering.get_rmi_handle(),
                (fn) &ordering_type::traverse_forward, findfw, nullptr, false);
    }
    else {
      this->m_ordering.traverse_forward(findfw, m_ordering.first());
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Find the last GID to define the boundaries of the container's
  ///        domain. The GID is returned using a promise.
  /// @tparam Prom The promise type.
  /// @param promise The promise to return the last GID.
  //////////////////////////////////////////////////////////////////////
  template<typename Prom>
  void find_last(Prom const& promise) const
  {
    typedef ordering_detail::find_bw<
      Prom,ordering_type, base_container_type
     >                                                visitor_type;

     visitor_type findbw(promise, &(this->m_ordering));

    typedef void (ordering_type::*fn)(visitor_type&, bc_base*, bool) const;

    size_t last_loc = m_ordering.get_num_locations()-1;
    if (m_ordering.get_location_id()!=last_loc) {
      async_rmi(last_loc,m_ordering.get_rmi_handle(),(fn)
                &ordering_type::traverse_backward, findbw, nullptr, false);
    }
    else {
      this->m_ordering.traverse_backward(findbw, m_ordering.last());
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Helper method to advance the given @p gid, @p n
  ///        positions. Result gid is set in @ref promise @p p.
  ///
  /// This method uses the base container ordering methods to traverse
  /// the base containers and the visitor functors @ref advance_fw and
  /// @ref advance_bw.
  //////////////////////////////////////////////////////////////////////
  void defer_advance(gid_type const& gid, long long n, bool globally,
                     promise<gid_type> p) const
  {
    if (n == 0)
    {
      p.set_value(gid);
      return;
    }

    stapl_assert(contains(gid), "gid not found");

    base_container_type* const bc = gid.m_base_container;

    if (n > 0)
    {
      ordering_detail::advance_fw<gid_type, base_container_type>
        advfw(gid, n, p);
      this->m_ordering.traverse_forward(advfw, bc);
      return;
    }

    ordering_detail::advance_bw<gid_type, base_container_type>
      advbw(gid, n, p);

    this->m_ordering.traverse_backward(advbw, bc);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element at position GID.
  /// @param gid GID of the element on which to apply the functor.
  /// @param f Functor to apply.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void apply(gid_type const& gid, Functor const& f)
  {
    stapl_assert(contains(gid),
    "invoking a function on an unknown base container");
    f(*gid.m_pointer);
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
    if (!contains(gid))
      return false;

    // else
    f(*gid.m_pointer);

    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element at position GID.
  /// @param gid GID of the element on which to apply the functor.
  /// @param f Functor to apply.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type apply_get(gid_type const& gid, Functor const& f)
  {
    stapl_assert(contains(gid),
    "invoking a function on an unknown base container");
    return f(*gid.m_pointer);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies an arbitrary functor to the element at position GID.
  /// @param gid GID of the element on which to apply the functor.
  /// @param f Functor to apply.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f) const
  {
    stapl_assert(contains(gid),
    "invoking a function on an unknown base container");
    return f(*gid.m_pointer);
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
    if (!contains(gid))
      return boost::optional<typename Functor::result_type>();

    // else
    return boost::optional<typename Functor::result_type>((*gid.m_pointer));
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
    if (!contains(gid))
      return boost::optional<typename Functor::result_type>();

    // else
    return boost::optional<typename Functor::result_type>((*gid.m_pointer));
  }

#ifndef CONTAINER_MAX_INVOKE_ARGS
  #define CONTAINER_MAX_INVOKE_ARGS 5
#endif

#define BOOST_PP_ITERATION_LIMITS (0, CONTAINER_MAX_INVOKE_ARGS)
#define BOOST_PP_FILENAME_1 \
"stapl/containers/distribution/container_manager/cm_heap.hpp"
#include BOOST_PP_ITERATE()

};

} // namespace stapl

#endif // STAPL_CONTAINERS_CM_HEAP_HPP

#else // BOOST_PP_IS_ITERATING

#define n BOOST_PP_ITERATION()

#define CONST_ref_invoke_arg(z, n, data)      \
  const typename invoke_arg<                  \
    BOOST_PP_CAT(Arg, n)>::type&              \
      BOOST_PP_CAT(arg, n)

#define MEM_func_arg_list(z)                  \
            BOOST_PP_IF(                      \
              BOOST_PP_EQUAL(n, 0),           \
              void BOOST_PP_TUPLE_EAT(2),     \
              BOOST_PP_ENUM_PARAMS            \
            )(n, Arg)


//////////////////////////////////////////////////////////////////////
/// @brief Invoke a base container method on
///        an element with the specified GID.
///        The element must exist in the current base container manager.
/// @param gid The GID of the element to invoke the method on.
/// @param memberFuncPtr A pointer to a base container's member method.
//////////////////////////////////////////////////////////////////////
template<typename Class
         BOOST_PP_COMMA_IF(n)
         BOOST_PP_ENUM_PARAMS(n, typename Arg)>
void invoke(gid_type const& gid, void
     (Class::* memberFuncPtr)(MEM_func_arg_list(n))
            BOOST_PP_COMMA_IF(n)
            BOOST_PP_ENUM(n, CONST_ref_invoke_arg, ~))
{
  stapl_assert(contains(gid),
  "invoking a function on an unknown base container");

  base_container_type* bc = gid.m_base_container;
  (bc->*memberFuncPtr)(BOOST_PP_ENUM_PARAMS(n, arg));
}


//////////////////////////////////////////////////////////////////////
/// @brief Invokes a base container method on the given @p gid if it is
///        present on this location.
/// @param gid The GID of the element to invoke the method on.
/// @param memberFuncPtr A pointer to a base container's member method.
/// @return True if gid was found and functor applied, otherwise returns false.
//////////////////////////////////////////////////////////////////////
template<typename Class
         BOOST_PP_COMMA_IF(n)
         BOOST_PP_ENUM_PARAMS(n, typename Arg)>
bool contains_invoke(gid_type const& gid, void
     (Class::* memberFuncPtr)(MEM_func_arg_list(n))
            BOOST_PP_COMMA_IF(n)
            BOOST_PP_ENUM(n, CONST_ref_invoke_arg, ~))
{
  if (!contains(gid))
    return false;

  base_container_type* bc = gid.m_base_container;
  (bc->*memberFuncPtr)(BOOST_PP_ENUM_PARAMS(n, arg));

  return true;
}


//////////////////////////////////////////////////////////////////////
/// @brief Invoke a base container method on an element with
///        the specified GID and return a member function pointer.
///        The element must exist in the current base container manager.
/// @param gid The GID of the element to invoke the method on.
/// @param memberFuncPtr A pointer to a base container's member method.
//////////////////////////////////////////////////////////////////////
template<typename Class, typename Rtn
         BOOST_PP_COMMA_IF(n)
         BOOST_PP_ENUM_PARAMS(n, typename Arg)>
Rtn invoke(gid_type const& gid, Rtn
    (Class::* memberFuncPtr)(MEM_func_arg_list(n))
           BOOST_PP_COMMA_IF(n)
           BOOST_PP_ENUM(n, CONST_ref_invoke_arg, ~))
{
  stapl_assert(contains(gid),
  "invoking a function on an unknown base container");

  base_container_type* bc = gid.m_base_container;
  stapl_assert(bc != 0, "Crisis ! data live nowhere");
  return (bc->*memberFuncPtr)(BOOST_PP_ENUM_PARAMS(n, arg));
}


//////////////////////////////////////////////////////////////////////
/// @brief Invokes a base container method on the given @p gid if it is
///        present on this location.
/// @param gid The GID of the element to invoke the method on.
/// @param memberFuncPtr A pointer to a base container's member method.
/// @return boost::optional with result of invocation if element was found.
//////////////////////////////////////////////////////////////////////
template<typename Class,
         typename Rtn BOOST_PP_ENUM_TRAILING_PARAMS(n, typename Arg)>
boost::optional<Rtn>
contains_invoke(gid_type const& gid,
                Rtn (Class::* memberFuncPtr)(MEM_func_arg_list(n))
                BOOST_PP_COMMA_IF(n)
                BOOST_PP_ENUM(n, CONST_ref_invoke_arg, ~))
{
  if (!contains(gid))
    return boost::optional<Rtn>();

  // else
  base_container_type* bc = gid.m_base_container;
  stapl_assert(bc != 0, "Crisis ! data live nowhere");

  return boost::optional<Rtn>(
    (bc->*memberFuncPtr)(BOOST_PP_ENUM_PARAMS(n, arg))
  );
}


//////////////////////////////////////////////////////////////////////
/// @brief Invoke a const base container method on an element
///        with the specified GID and return a member function pointer.
///        The element must exist in the current base container manager.
/// @param gid The GID of the element to invoke the method on.
/// @param memberFuncPtr A pointer to a base container's member method.
//////////////////////////////////////////////////////////////////////
template<typename Class, typename Rtn
         BOOST_PP_COMMA_IF(n)
         BOOST_PP_ENUM_PARAMS(n, typename Arg)>
Rtn const_invoke(gid_type const& gid, Rtn
                (Class::* const memberFuncPtr)(MEM_func_arg_list(n))
                 BOOST_PP_COMMA_IF(n)
                 BOOST_PP_ENUM(n, CONST_ref_invoke_arg, ~)) const
{
  stapl_assert(contains(gid),
  "invoking a function on an unknown base container");

  base_container_type* bc = gid.m_base_container;
  return (bc->*memberFuncPtr)(BOOST_PP_ENUM_PARAMS(n, arg));
}

#undef MEM_func_arg_list
#undef CONST_ref_invoke_arg
#undef n

#endif // BOOST_PP_IS_ITERATING
