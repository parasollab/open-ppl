/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BOOST_PP_IS_ITERATING

#ifndef STAPL_CONTAINERS_CONST_PROPERTY_ACCESSOR_HPP
#define STAPL_CONTAINERS_CONST_PROPERTY_ACCESSOR_HPP

#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/containers/graph/functional.hpp>

#include <stapl/views/proxy/proxy.hpp>

#include <stapl/utility/invoke_arg.hpp>

#include <boost/bind.hpp>
#include <boost/bind/protect.hpp>

#include <boost/preprocessor/repetition.hpp>
#include <boost/preprocessor/repetition/enum_params.hpp>
#include <boost/preprocessor/control/if.hpp>
#include <boost/preprocessor/comparison/equal.hpp>
#include <boost/preprocessor/tuple/eat.hpp>
#include <boost/preprocessor/iteration/iterate.hpp>
#include <boost/preprocessor/punctuation/comma_if.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Accessor for the property on vertices of a graph providing
/// read-only access.
/// @tparam Container Type of the pGraph container.
//////////////////////////////////////////////////////////////////////
template <typename Container>
class const_property_accessor
{
public:
  typedef Container                                         container_type;
  typedef typename container_traits<Container>::gid_type    index_type;
  typedef const typename container_traits<Container>::value_type::property_type
    value_type;

protected:

  template <typename Derived, typename A, typename C, typename D>
  friend class iterator_facade;

  friend class accessor_core_access;

  container_type const*  m_container;
  index_type             m_index;

public:
  bool is_null() const
  {
    return m_container->domain().empty();
  };

  void define_type(typer& t)
  {
    t.member(m_container);
    t.member(m_index);
  }

  const_property_accessor(void)
    : m_container(),
      m_index()
  { }

  const_property_accessor(null_reference const&)
    : m_container(),
      m_index()
  { }

  const_property_accessor(const_property_accessor const& other)
    : m_container(other.m_container), m_index(other.m_index)
  { }

  const_property_accessor(container_type const* container,
                          index_type const& index)
    : m_container(container), m_index(index)
  { }

  value_type read() const
  {
    return m_container->get_element(m_index).property();
  }

  value_type ref() const
  {
    return this->read().property();
  }

  template<typename F>
  typename F::result_type apply_get(F const& f) const
  {
    return m_container->vp_apply(m_index, f);
  }


#ifndef CONTAINER_MAX_INVOKE_ARGS
  #define CONTAINER_MAX_INVOKE_ARGS 5
#endif

#define BOOST_PP_ITERATION_LIMITS (0, CONTAINER_MAX_INVOKE_ARGS)
#define BOOST_PP_FILENAME_1   "stapl/containers/iterators/const_property_accessor.hpp"
#include BOOST_PP_ITERATE()

}; // class property_accessor

} // stapl namespace

#endif // STAPL_CONTAINERS_CONST_PROPERTY_ACCESSOR_HPP

#else // BOOST_PP_IS_ITERATING

#define this_iteration BOOST_PP_ITERATION()

#define CONST_ref_invoke_arg(z, this_iteration, data) \
          const typename invoke_arg<BOOST_PP_CAT(Arg, this_iteration)>::type&\
 BOOST_PP_CAT(arg, this_iteration)

#define STATIC_cast_args(z, this_iteration, data) \
          static_cast<BOOST_PP_CAT(Arg, this_iteration)>(BOOST_PP_CAT(arg,\
            this_iteration))
#define MEM_func_arg_list(z)                          \
            BOOST_PP_IF(                              \
              BOOST_PP_EQUAL(z, 0),      \
              void BOOST_PP_TUPLE_EAT(2),             \
              BOOST_PP_ENUM_PARAMS                   \
            )(z, Arg)

template<typename Class, typename Rtn
         BOOST_PP_COMMA_IF(this_iteration)
         BOOST_PP_ENUM_PARAMS(this_iteration, typename Arg)>
Rtn const_invoke(Rtn (Class::* const memberFuncPtr)(
                   MEM_func_arg_list(this_iteration)) const
                 BOOST_PP_COMMA_IF(this_iteration)
                 BOOST_PP_ENUM(this_iteration, CONST_ref_invoke_arg, ~)) const
{
  return m_container->vp_apply(m_index, boost::bind(
    memberFuncPtr, _1 BOOST_PP_COMMA_IF(this_iteration) \
    BOOST_PP_ENUM(this_iteration, STATIC_cast_args, ~)));
}

#undef MEM_func_arg_list
#undef CONST_ref_invoke_arg
#undef STATIC_cast_args
#undef this_iteration

#endif // BOOST_PP_IS_ITERATING
