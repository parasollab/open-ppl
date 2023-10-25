/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_PROXY_MACROS_HPP
#define STAPL_VIEWS_PROXY_MACROS_HPP

#include <stapl/views/proxy/member_accessor.hpp>
#include <stapl/utility/import_types.hpp>
#include <stapl/containers/type_traits/is_container.hpp>
#include <stapl/utility/tuple/apply.hpp>
#include <stapl/views/proxy/proxy.hpp>

#include <type_traits>

#include <boost/preprocessor/repetition/enum_params.hpp>
#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/arithmetic/sub.hpp>

//////////////////////////////////////////////////////////////////////
/// @file proxy_macros.hpp
/// @todo Move this file into stapl/views/proxy
//////////////////////////////////////////////////////////////////////

namespace stapl {

template<typename C>
struct container_traits;


//////////////////////////////////////////////////////////////////////
/// @brief Functor creates a reference either as a proxy or a view for
/// a member of another proxy, based on @ref is_container
/// metafunction.
/// @tparam Accessor The @ref member_accessor instance used to reference
///   the proxy member.
///
/// Primary template constructs a proxy with the accessor and returns it
/// as the reference for the iterator dereference. The sole specialization
/// instead heap allocates the proxy and uses it as the container for a
/// view (the type of which is determined by @ref container_traits.
//////////////////////////////////////////////////////////////////////
template<typename Accessor,
         bool = is_container<
           typename std::remove_const<typename Accessor::value_type>::type
         >::value>
struct member_referencer
{
private:
  typedef typename std::remove_const<
    typename Accessor::value_type
  >::type                                                value_t;

public:
  typedef proxy<value_t, Accessor>                       result_type;

  template<typename ParentAccessor>
  result_type operator()(ParentAccessor const& acc) const
  {
    return result_type(acc);
  }
};


template<typename Accessor>
struct member_referencer<Accessor, true>
{
private:
  typedef typename std::remove_const<
    typename Accessor::value_type
  >::type                                                value_t;

  typedef proxy<value_t, Accessor>                       proxy_t;

public:
  typedef typename container_traits<value_t>::
    template construct_view<proxy_t>::type               result_type;

  template<typename ParentAccessor>
  result_type operator()(ParentAccessor const& acc) const
  {
    return result_type(new proxy_t(acc));
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Boilerplate for writing proxies. When writing a simple
///        proxy for some type T, the skeleton would be of the form:
///
/// @code
///    STAPL_PROXY_HEADER(T)
///    {
///      STAPL_PROXY_DEFINES(T)
///
///      // user defined functions
///    };
/// @endcode
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_HEADER(TYPE)                                               \
template <typename Accessor>                                                   \
class proxy<TYPE, Accessor>                                                    \
  : public Accessor

#define STAPL_PROXY_HEADER_TEMPLATE(...) BOOST_PP_CAT(                         \
  STAPL_PROXY_HEADER_TEMPLATE_,                                                \
  BOOST_PP_SUB(STAPL_PROXY_VARIADIC_SIZE(__VA_ARGS__), 1))(__VA_ARGS__)

#define STAPL_PROXY_HEADER_TEMPLATE_1(TYPE, A0)                                \
template <typename A0, typename Accessor>                                      \
class proxy<TYPE<A0>, Accessor>                                                \
  : public Accessor

#define STAPL_PROXY_HEADER_TEMPLATE_2(TYPE, A0, A1)                            \
template <typename A0, typename A1, typename Accessor>                         \
class proxy<TYPE<A0, A1>, Accessor>                                            \
  : public Accessor

#define STAPL_PROXY_HEADER_TEMPLATE_3(TYPE, A0, A1, A2)                        \
template <typename A0, typename A1, typename A2, typename Accessor>            \
class proxy<TYPE<A0, A1, A2>, Accessor>                                        \
  : public Accessor


#define STAPL_PROXY_DEFINES(TYPE)            \
  STAPL_PROXY_TYPES(TYPE, Accessor)          \
  STAPL_PROXY_CONSTRUCTOR(TYPE, Accessor)    \
  STAPL_PROXY_METHODS(TYPE, Accessor)

//////////////////////////////////////////////////////////////////////
/// @brief If customization of the boilerplate proxy is needed, the
///        following macros are defined. For example, if a custom
///        constructor is needed, one would need to write:
/// @code
///    STAPL_PROXY_HEADER(T)
///    {
///      STAPL_PROXY_TYPES(T)
///      STAPL_PROXY_METHODS(T)
///      proxy(U const& u)
///      { ... }
///    };
/// @endcode
//////////////////////////////////////////////////////////////////////

#define STAPL_PROXY_TYPES(TYPE, ACC)          \
private:                                      \
  friend class proxy_core_access;             \
  typedef TYPE target_t;


#define STAPL_PROXY_CONSTRUCTOR(TYPE, ACC)    \
public:                                       \
  explicit proxy(ACC const& acc)              \
   : ACC(acc)                                 \
  { }                                         \


#define STAPL_PROXY_METHODS(TYPE, ACC)        \
public:                                       \
  operator target_t() const                   \
  {                                           \
    return ACC::read();                       \
  }                                           \
  proxy const& operator=(proxy const& rhs)    \
  {                                           \
    ACC::write(rhs); return *this;            \
  }                                           \
  proxy const& operator=(target_t const& rhs) \
  {                                           \
    ACC::write(rhs);                          \
    return *this;                             \
  }                                           \
  proxy const& operator=(target_t&& rhs)      \
  {                                           \
    ACC::write(std::move(rhs));               \
    return *this;                             \
  }                                           \

//////////////////////////////////////////////////////////////////////
/// @brief Concat complex typenames. For example, when writing a proxy
///        for pair<T, U>, the type will be interpreted as two
///        arguments. In order to pass it to the following macros, one
///        would have to do:
/// @code
///    STAPL_PROXY_CONCAT(pair<T, U>)
/// @endcode
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_CONCAT(...) __VA_ARGS__

//////////////////////////////////////////////////////////////////////
/// @brief Helper to import nested types from the proxy's target type.
///
/// If there are nested types in a class of the form:
/// @code
///     typedef int foo;
///     typedef float bar;
/// @endcode
/// the following would automatically import the types:
/// @code
///     STAPL_PROXY_IMPORT_TYPES(foo, bar)
/// @endcode
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_IMPORT_TYPES(...)                                       \
 public:                                                                    \
 BOOST_PP_CAT(STAPL_IMPORT_TYPES_, STAPL_PROXY_VARIADIC_SIZE(__VA_ARGS__))( \
   typename target_t, __VA_ARGS__)

//////////////////////////////////////////////////////////////////////
/// @brief Helper to create methods for proxies.
///
/// If there is a method in a class with the signature
/// @code
///     void foo(int x, int y)
/// @endcode
/// the following would automatically generate the method in the proxy:
/// @code
///     STAPL_PROXY_METHOD(foo, int, int)
/// @endcode
/// If the function has a return, such as
/// @code
///     string bar(char c)
/// @endcode
/// the following would automatically generate the method in the proxy:
/// @code
///     STAPL_PROXY_METHOD_RETURN(bar, string, char)
/// @endcode
//////////////////////////////////////////////////////////////////////

#define STAPL_PROXY_METHOD_RETURN(...) BOOST_PP_CAT(STAPL_PROXY_METHOD_RETURN_,\
  BOOST_PP_SUB(STAPL_PROXY_VARIADIC_SIZE(__VA_ARGS__), 2))(__VA_ARGS__)

#define STAPL_PROXY_METHOD(...) BOOST_PP_CAT(STAPL_PROXY_METHOD_,              \
  BOOST_PP_SUB(STAPL_PROXY_VARIADIC_SIZE(__VA_ARGS__), 1))(__VA_ARGS__)

//////////////////////////////////////////////////////////////////////
/// @brief Helper to create member references for proxies.
///
/// If there is a member in a class named foo with the type T, the
/// following would automatically create the nested proxy:
/// @code
///     STAPL_PROXY_MEMBER(foo, T)
/// @endcode
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_MEMBER(NAME, TYPE)                               \
  STAPL_PROXY_SELECTOR_MEMBER(NAME)                                  \
                                                                     \
  typedef member_accessor<get_ ## NAME<TYPE>>   NAME ## _accessor;   \
  typedef typename member_referencer<                                \
    NAME ## _accessor                                                \
   >::result_type                               NAME ## _reference;  \
                                                                     \
  NAME ## _reference NAME;

  // typedef proxy<TYPE, NAME ## _accessor>        NAME ## _reference

//////////////////////////////////////////////////////////////////////
/// @brief Define the nested type T from the target.
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_REFLECT_TYPE(T) \
  typedef typename target_t::T T;

//////////////////////////////////////////////////////////////////////
/// @brief Selector generator for creating selectors for nested
///        proxies. Generates selectors in which the selection scheme
///        is retrieving a member of the outer proxy.
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_SELECTOR_MEMBER(NAME)      \
template<typename __Tp>                        \
struct get_ ## NAME                            \
{                                              \
  using result_type   = __Tp;                  \
  using accessor_type = Accessor;              \
                                               \
  template<typename Outer>                     \
  result_type operator()(Outer const& v) const \
  {                                            \
    return v. NAME;                            \
  }                                            \
  template<typename Outer>                     \
  result_type& operator()(Outer& v) const      \
  {                                            \
    return v. NAME;                            \
  }                                            \
};

//////////////////////////////////////////////////////////////////////
/// @name Nested proxy selectors
///
/// Generators for creating selectors for nested proxies. Generate
/// selectors in which the selection scheme is invoking a method on
/// the outer proxy.
///@{

#define STAPL_PROXY_SELECTOR_METHOD(NAME)                            \
  template<typename __Tp>                                            \
  struct get_ ## NAME                                                \
  {                                                                  \
    using result_type   = __Tp;                                      \
    using accessor_type = Accessor;                                  \
                                                                     \
    template<typename Outer>                                         \
    result_type operator()(Outer const& v) const                     \
    {                                                                \
      return v. NAME();                                              \
    }                                                                \
    template<typename Outer>                                         \
    result_type& operator()(Outer& v) const                          \
    {                                                                \
      return v. NAME();                                              \
    }                                                                \
  };


#define STAPL_PROXY_SELECTOR_METHOD_ARGS_1(NAME, FN, A0)             \
template<typename __Tp>                                              \
struct get_ ## NAME                                                  \
{                                                                    \
  A0 m_a0;                                                           \
                                                                     \
  get_ ## NAME(A0 const& a0)                                         \
    : m_a0(a0)                                                       \
  { }                                                                \
                                                                     \
  using result_type   = __Tp;                                        \
  using accessor_type = Accessor;                                    \
                                                                     \
  template<typename Outer>                                           \
  typename std::enable_if<                                           \
    !std::is_lvalue_reference<Outer>::value, result_type>::type      \
  operator()(Outer&& v) const                                        \
  {                                                                  \
    return v. FN(m_a0);                                              \
  }                                                                  \
                                                                     \
  template<typename Outer>                                           \
  result_type const& operator()(Outer const& v) const                \
  {                                                                  \
    return v. FN(m_a0);                                              \
  }                                                                  \
                                                                     \
  template<typename Outer>                                           \
  result_type& operator()(Outer& v) const                            \
  {                                                                  \
    return v. FN(m_a0);                                              \
  }                                                                  \
                                                                     \
  void define_type(::stapl::typer& t)                                \
  {                                                                  \
    t.member(m_a0);                                                  \
  }                                                                  \
};

//////////////////////////////////////////////////////////////////////
/// @brief Selector returning reference to an element of a lightweight
/// multiarray wrapped by a proxy.
///
/// The gid of the element can be given either as a variadic pack of indices
/// or a tuple of those - in both cases, the selector stores a tuple, but
/// passes it to lightweight_multiarray::operator() in the unpacked (variadic)
/// form upon invocation.
///
/// @sa STAPL_PROXY_REFERENCE_METHOD_1
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_SELECTOR_METHOD_ARGS_VARIADIC(NAME, FN)                 \
template<typename __Tp>                                                     \
struct get_ ## NAME                                                         \
{                                                                           \
  stapl::homogeneous_tuple_type_t<dims, std::size_t> m_a0;                  \
                                                                            \
  template<typename... Indices>                                             \
  get_ ## NAME(Indices... indices)                                          \
    : m_a0(static_cast<std::size_t>(indices)...)                            \
  { }                                                                       \
                                                                            \
  template<typename... Indices>                                             \
  get_ ## NAME(stapl::tuple<Indices...> const& gid)                         \
    : m_a0(gid)                                                             \
  { }                                                                       \
                                                                            \
  using result_type   = __Tp;                                               \
  using accessor_type = Accessor;                                           \
                                                                            \
  template<typename Outer>                                                  \
  typename std::enable_if<                                                  \
    !std::is_lvalue_reference<Outer>::value, result_type>::type             \
  operator()(Outer&& v) const                                               \
  {                                                                         \
    return tuple_ops::apply(                                                \
             access_outer<decltype(std::forward<Outer>(v))>(                \
               std::forward<Outer>(v)), m_a0);                              \
  }                                                                         \
                                                                            \
  template<typename Outer>                                                  \
  result_type const& operator()(Outer const& v) const                       \
  {                                                                         \
    return tuple_ops::apply(access_outer<Outer const&>(v), m_a0);           \
  }                                                                         \
                                                                            \
  template<typename Outer>                                                  \
  result_type& operator()(Outer& v) const                                   \
  {                                                                         \
    return tuple_ops::apply(access_outer<Outer&>(v), m_a0);                 \
  }                                                                         \
                                                                            \
  void define_type(::stapl::typer& t)                                       \
  {                                                                         \
    t.member(m_a0);                                                         \
  }                                                                         \
                                                                            \
private:                                                                    \
  template<typename OuterRef>                                               \
  class access_outer                                                        \
  {                                                                         \
    OuterRef m_v;                                                           \
                                                                            \
  public:                                                                   \
    access_outer(OuterRef v)                                                \
      : m_v(v)                                                              \
    {}                                                                      \
                                                                            \
    template<typename... Indices>                                           \
    auto operator() (Indices... indices)                                    \
    STAPL_AUTO_RETURN (                                                     \
      m_v. FN(indices...)                                                   \
    )                                                                       \
  };                                                                        \
};
///@}

#define STAPL_PROXY_REFERENCE_METHOD(NAME, TYPE)                     \
  STAPL_PROXY_REFERENCE_METHOD_0


#define STAPL_PROXY_REFERENCE_METHOD_0(NAME, TYPE)                   \
  STAPL_PROXY_SELECTOR_METHOD(NAME)                                  \
                                                                     \
  typedef member_accessor<get_ ## NAME<TYPE>> NAME ## _accessor;     \
  typedef proxy<TYPE, NAME ## _accessor>      NAME ## _reference;    \
                                                                     \
  NAME ## _reference NAME() const                                    \
  {                                                                  \
    return NAME ## _reference(*this);                                \
  }

#define STAPL_PROXY_REFERENCE_METHOD_1(NAME, FN, TYPE, A0)                 \
  STAPL_PROXY_SELECTOR_METHOD_ARGS_1(NAME, FN, A0)                         \
                                                                           \
  typedef get_ ## NAME<TYPE>                      NAME ## _selector_type;  \
  typedef member_accessor<NAME ## _selector_type> NAME ## _accessor;       \
  typedef proxy<TYPE, NAME ## _accessor>          NAME ## _reference;      \
                                                                           \
  NAME ## _reference FN(A0 const& a0) const                                \
  {                                                                        \
    return NAME ## _reference(                                             \
      NAME ## _accessor(*this, NAME ## _selector_type(a0))                 \
    );                                                                     \
  }

//////////////////////////////////////////////////////////////////////
/// @brief Create a method in an outer proxy that returns proxy to the
/// element in the inner (multidimensional) container wrapped by
/// that outer proxy.
///
/// @param NAME Basis for the names of the datatypes introduced into
/// the outer proxy by this macro.
/// @param FN  Name of the variadic method added to the proxy. Method
/// with the same name must be present in the wrapped container class
/// as it is used for the actual data access.
/// @param TYPE Type of the element proxy of which is being created.
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_REFERENCE_METHOD_VARIADIC_ONLY(NAME, FN, TYPE)         \
  STAPL_PROXY_SELECTOR_METHOD_ARGS_VARIADIC(NAME, FN)                      \
                                                                           \
  typedef get_ ## NAME<TYPE>                      NAME ## _selector_type;  \
  typedef member_accessor<NAME ## _selector_type> NAME ## _accessor;       \
  typedef proxy<TYPE, NAME ## _accessor>          NAME ## _reference;      \
                                                                           \
  template<typename... Indices>                                            \
  NAME ## _reference FN(Indices... indices) const                          \
  {                                                                        \
    return NAME ## _reference(                                             \
      NAME ## _accessor(*this, NAME ## _selector_type(indices...))         \
    );                                                                     \
  }                                                                        \

//////////////////////////////////////////////////////////////////////
/// @brief Create methods in an outer proxy that return proxy to the
///  element in the inner (multidimensional) container wrapped by
///  that outer proxy. Creates two methods allowing access to the
///  inner element using either variadic index pack or index tuple.
///
/// @param NAME Basis for the names of the datatypes introduced into
/// the outer proxy by this macro.
/// @param FNV  Name of the method accepting variadic indices. Method
/// with the same name must be present in the wrapped container class
/// as it is used for the actual data access.
/// @param FN1  Name of the method accepting index tuple. The tuple is
/// unpacked to actually access the data using the variadic method of
/// the underlying container.
/// @param TYPE Type of the element proxy of which is being created.
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_REFERENCE_METHOD_VARIADIC_AND_SINGLE(NAME, FNV, FN1, TYPE) \
  STAPL_PROXY_SELECTOR_METHOD_ARGS_VARIADIC(NAME, FNV)                         \
                                                                               \
  typedef get_ ## NAME<TYPE>                      NAME ## _selector_type;      \
  typedef member_accessor<NAME ## _selector_type> NAME ## _accessor;           \
  typedef proxy<TYPE, NAME ## _accessor>          NAME ## _reference;          \
                                                                               \
  template<typename... Indices>                                                \
  NAME ## _reference FNV(Indices... indices) const                             \
  {                                                                            \
    return NAME ## _reference(                                                 \
      NAME ## _accessor(*this, NAME ## _selector_type(indices...))             \
    );                                                                         \
  }                                                                            \
                                                                               \
  template<typename... Indices>                                                \
  NAME ## _reference FN1(stapl::tuple<Indices...> const& gid) const            \
  {                                                                            \
    return NAME ## _reference(                                                 \
      NAME ## _accessor(*this, NAME ## _selector_type(gid))                    \
    );                                                                         \
  }

// If STAPL_PROXY_REFERENCE_METHOD_VARIADIC is called with 4 arguments, use
// STAPL_PROXY_REFERENCE_METHOD_VARIADIC; if it is given 3 arguments, use
// STAPL_PROXY_REFERENCE_METHOD_VARIADIC_ONLY.
#define GET_MACRO(_1, _2, _3, _4, NAME, ...) NAME
#define STAPL_PROXY_REFERENCE_METHOD_VARIADIC(...)                          \
  GET_MACRO(__VA_ARGS__,                                                    \
            STAPL_PROXY_REFERENCE_METHOD_VARIADIC_AND_SINGLE,               \
            STAPL_PROXY_REFERENCE_METHOD_VARIADIC_ONLY)(__VA_ARGS__)        \

// Derived from BOOST_PP_VARIADIC_SIZE, current configuration in
// Boost 1.53 does not allow direct use (turned off for clang and
// C++98 GNU)
#define STAPL_PROXY_VARIADIC_SIZE(...)                                         \
  STAPL_PROXY_VARIADIC_SIZE_I(__VA_ARGS__,                                     \
  64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46,  \
  45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27,  \
  26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, \
  6, 5, 4, 3, 2, 1,)

#define STAPL_PROXY_VARIADIC_SIZE_I(e0, e1, e2, e3, e4, e5, e6, e7, e8, e9,    \
  e10, e11, e12, e13, e14, e15, e16, e17, e18, e19, e20, e21, e22, e23, e24,   \
  e25, e26, e27, e28, e29, e30, e31, e32, e33, e34, e35, e36, e37, e38, e39,   \
  e40, e41, e42, e43, e44, e45, e46, e47, e48, e49, e50, e51, e52, e53, e54,   \
  e55, e56, e57, e58, e59, e60, e61, e62, e63, size, ...) size


#define STAPL_PROXY_METHOD_RETURN_0(NAME, RTN)                       \
  RTN NAME() const                                                   \
  {                                                                  \
    return Accessor::const_invoke(&target_t::NAME);                  \
  }

#define STAPL_PROXY_METHOD_RETURN_1(NAME, RTN, A0)                   \
  RTN NAME(A0 const& a0) const                                       \
  {                                                                  \
     return Accessor::const_invoke(&target_t::NAME, a0);             \
  }

#define STAPL_PROXY_METHOD_RETURN_2(NAME, RTN, A0, A1)               \
  RTN NAME(A0 const& a0, A1 const& a1) const                         \
  {                                                                  \
    return Accessor::const_invoke(&target_t::NAME, a0, a1);          \
  }

#define STAPL_PROXY_METHOD_RETURN_3(NAME, RTN, A0, A1, A2)           \
  RTN NAME(A0 const& a0, A1 const& a1, A2 const& a2) const           \
  {                                                                  \
    return Accessor::const_invoke(&target_t::NAME, a0, a1, a2);      \
  }

#define STAPL_PROXY_METHOD_0(NAME)                                   \
  void NAME() const                                                  \
  {                                                                  \
  Accessor::invoke(&target_t::NAME);                                 \
  }

#define STAPL_PROXY_METHOD_1(NAME, A0)                               \
  void NAME(A0 const& a0) const                                      \
  {                                                                  \
    Accessor::invoke(&target_t::NAME, a0);                           \
  }

#define STAPL_PROXY_METHOD_2(NAME, A0, A1)                           \
  void NAME(A0 const& a0, A1 const& a1) const                        \
  {                                                                  \
    Accessor::invoke(&target_t::NAME, a0, a1);                       \
  }

#define STAPL_PROXY_METHOD_3(NAME, A0, A1, A2)                       \
  void NAME(A0 const& a0, A1 const& a1, A2 const& a2) const          \
  {                                                                  \
    Accessor::invoke(&target_t::NAME, a0, a1, a2);                   \
  }

} // namespace stapl

#endif // STAPL_VIEWS_PROXY_MACROS_HPP
