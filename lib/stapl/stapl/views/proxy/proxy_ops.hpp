/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_P_CONTAINER_VIEW_PROXY_OPS_HPP
#define STAPL_P_CONTAINER_VIEW_PROXY_OPS_HPP

#include <iosfwd>

#include <stapl/algorithms/functional.hpp>

#include <boost/static_assert.hpp>

#include <stapl/runtime/stapl_assert.hpp>
#include <stapl/runtime/serialization_fwd.hpp>

#include <stapl/views/proxy/transform_reference.hpp>
#include <stapl/views/proxy/accessor.hpp>

namespace stapl {

/////////////////////////////////////////////////////////////////////
/// @brief Specialization to allow STL to swap proxies as they won't
///        bind to references in general swap in typical usage cases.
//////////////////////////////////////////////////////////////////////
template<typename T, typename D1, typename D2>
void swap(proxy<T,D1> a, proxy<T,D2> b)
{
  T va = a;
  T vb = b;
  a = vb;
  b = va;
}

template<typename CharT, typename Traits, class T, class B>
std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits>& os, proxy<T,B> const& prx)
{
  return os << static_cast<T>(prx);
}

template<typename CharT, typename Traits, typename T, typename D>
std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits>& is, proxy<T,D>& prx)
{
  T v;
  is >> v;
  accessor_core_access::write(proxy_core_access::accessor(prx), v);
  return is;
}


//////////////////////////////////////////////////////////////////////
/// @todo Define the semantics of this operator over a proxy.
//////////////////////////////////////////////////////////////////////
template<typename T, typename D>
T* operator&(proxy<T,D> const&)
{
  BOOST_STATIC_ASSERT(sizeof(T) == 0);
  stapl_assert(0, "proxy::operator& blocked for now");
}


//////////////////////////////////////////////////////////////////////
/// @brief Macro used to define a functor to compute the given
///        compound operation @p op (e.g., +=, *=, etc.).
///
/// @param op Operator
/// @param name Description name for the functor (the functor name is
///             @c proxy_name_wf).
//////////////////////////////////////////////////////////////////////
#define STAPL_MAKE_PROXY_COMPOUND_OP_FUNCTOR(op,name) \
template<typename Q> \
struct proxy_ ## name ## _wf{ \
typedef void result_type; \
private: \
    Q inc; \
\
public: \
  proxy_ ## name ## _wf(Q const& i):inc(i){} \
\
    template<typename T> \
    void operator()(T& object) const{ \
        object op inc; \
    } \
\
    void define_type(typer& t) \
  { \
    t.member(inc); \
  } \
\
};


//////////////////////////////////////////////////////////////////////
/// @brief Macro used to specialize the given compound operation @p op
///        to work over proxies.
///
/// @param op Operator
/// @param name Description name for the functor to use.
//////////////////////////////////////////////////////////////////////
#define STAPL_MAKE_PROXY_COMPOUND_ASSIGNMENT_OP(op,name) \
template<typename T, typename A, typename Q> \
proxy<T,A> const& \
operator op (proxy<T,A> const& p, Q const& inc) \
{ \
  accessor_core_access::apply_set( \
    proxy_core_access::accessor(p), proxy_ ## name ## _wf<Q>(inc)); \
  return p; \
}

#define STAPL_PROXY_COMPOUND_ASSIGNMENT_OP(op, name) \
  STAPL_MAKE_PROXY_COMPOUND_OP_FUNCTOR(op,name) \
  STAPL_MAKE_PROXY_COMPOUND_ASSIGNMENT_OP(op,name)

STAPL_PROXY_COMPOUND_ASSIGNMENT_OP(+=, addition_assignment)
STAPL_PROXY_COMPOUND_ASSIGNMENT_OP(-=, subtraction_assignment)
STAPL_PROXY_COMPOUND_ASSIGNMENT_OP(*=, multiplication_assignment)
STAPL_PROXY_COMPOUND_ASSIGNMENT_OP(/=, division_assignment)
STAPL_PROXY_COMPOUND_ASSIGNMENT_OP(%=, modulo_assignment)
STAPL_PROXY_COMPOUND_ASSIGNMENT_OP(&=, bitwise_AND_assignment)
STAPL_PROXY_COMPOUND_ASSIGNMENT_OP(|=, bitwise_OR_assignment)
STAPL_PROXY_COMPOUND_ASSIGNMENT_OP(^=, bitwise_XOR_assignment)
STAPL_PROXY_COMPOUND_ASSIGNMENT_OP(<<=, bitwise_left_shift_assignment)
STAPL_PROXY_COMPOUND_ASSIGNMENT_OP(>>=, bitwise_right_shift_assignment)

#undef STAPL_MAKE_PROXY_COMPOUND_OP_FUNCTOR
#undef STAPL_MAKE_PROXY_COMPOUND_ASSIGNMENT_OP
#undef STAPL_PROXY_COMPOUND_ASSIGNMENT_OP


//////////////////////////////////////////////////////////////////////
/// @brief Macro used to define a functor to compute the given prefix
///        operation @p op.
///
/// @param op Operator
/// @param name Description name for the functor (the functor name is
///             @c proxy_name_wf).
//////////////////////////////////////////////////////////////////////
#define STAPL_MAKE_PROXY_PREFIX_OP_FUNCTOR(op, name) \
struct proxy_ ## name ## _wf{ \
typedef void result_type; \
public: \
    template<typename T> \
    void operator()(T& object) const{ \
      op object; \
    } \
\
};

//////////////////////////////////////////////////////////////////////
/// @brief Macro used to specialize the given prefix operation @p op
///        to work over proxies.
///
/// @param op Operator
/// @param name Description name for the functor to use.
//////////////////////////////////////////////////////////////////////
#define STAPL_MAKE_PROXY_PREFIX_OP(op, name) \
template<typename T, typename A> \
proxy<T,A> const& \
operator op (proxy<T,A> const& p) \
{ \
  accessor_core_access::apply_set( \
    proxy_core_access::accessor(p), proxy_ ## name ## _wf()); \
  return p; \
}

#define STAPL_PROXY_PREFIX_OP(op, name) \
  STAPL_MAKE_PROXY_PREFIX_OP_FUNCTOR(op, name) \
  STAPL_MAKE_PROXY_PREFIX_OP(op, name)

STAPL_PROXY_PREFIX_OP(++, prefix_increment)
STAPL_PROXY_PREFIX_OP(--, prefix_decrement)

#undef STAPL_MAKE_PROXY_PREFIX_OP_FUNCTOR
#undef STAPL_MAKE_PROXY_PREFIX_OP
#undef STAPL_PROXY_PREFIX_OP


//////////////////////////////////////////////////////////////////////
/// @brief Macro used to define a functor to compute the given suffix
///        operation @p op.
///
/// @param op Operator
/// @param name Description name for the functor (the functor name is
///             @c proxy_name_wf).
//////////////////////////////////////////////////////////////////////
#define STAPL_MAKE_PROXY_SUFFIX_OP_FUNCTOR(op, name) \
struct proxy_ ## name ## _wf{ \
typedef void result_type; \
public: \
    template<typename T> \
    void operator()(T& object) const{ \
      object op; \
    } \
\
};

//////////////////////////////////////////////////////////////////////
/// @brief Macro used to specialize the given suffix operation @p op
///        to work over proxies.
///
/// @param op Suffix operator
/// @param name Description name for the functor to use.
///
/// @todo Define the correct semantics for this type of operator. Some
///        aspects to consider:
/// (a) incur an additional remote op / fetch that is unnecessary.
/// (b) it's wrong; you get the value after the increment/decrement.
///
/// Probably shouldn't be done anyways (make the user explicitly copy
/// to a temp prior to calling prefix equivalent.  So it's more clear
/// that communication may be generated.
//////////////////////////////////////////////////////////////////////
#define STAPL_MAKE_PROXY_SUFFIX_OP(op, name) \
template<typename T, typename A>             \
T operator op (proxy<T,A> const& p, int)     \
{                                            \
  BOOST_STATIC_ASSERT(sizeof(T) == 0);       \
}

#define STAPL_PROXY_SUFFIX_OP(op, name) \
  STAPL_MAKE_PROXY_SUFFIX_OP_FUNCTOR(op, name) \
  STAPL_MAKE_PROXY_SUFFIX_OP(op, name)

STAPL_PROXY_SUFFIX_OP(++, suffix_increment)
STAPL_PROXY_SUFFIX_OP(--, suffix_decrement)

#undef STAPL_MAKE_PROXY_SUFFIX_OP_FUNCTOR
#undef STAPL_MAKE_PROXY_SUFFIX_OP
#undef STAPL_PROXY_SUFFIX_OP


//////////////////////////////////////////////////////////////////////
/// @brief Macro used to specialize the given binary operation @p op
///        when both parameters are proxies.
/// @param op Binary operator.
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_BINARY_OP_BOTH(op, result_type)                 \
  template <typename V1, typename A1, typename V2, typename A2>     \
  result_type                                                       \
  operator op(proxy<V1,A1> const& lhs, proxy<V2,A2> const& rhs)     \
  {                                                                 \
    typedef typename std::common_type<V1, V2>::type ct_t;           \
    return static_cast<ct_t>(lhs) op static_cast<ct_t>(rhs);        \
  }

//////////////////////////////////////////////////////////////////////
/// @brief Macro used to specialize the given binary operation @p op
///        when the left hand side is a proxy.
/// @param op Binary operator.
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_BINARY_OP_LEFT(op, result_type)                 \
  template <typename V1, typename A1, typename T>                   \
  result_type                                                       \
  operator op(proxy<V1,A1> const& lhs, T const& rhs)                \
  {                                                                 \
    typedef typename std::common_type<V1, T>::type ct_t;            \
    return static_cast<ct_t>(lhs) op static_cast<ct_t const&>(rhs); \
  }                                                                 \

//////////////////////////////////////////////////////////////////////
/// @brief Macro used to specialize the given binary operation @p op
///        when the right hand side is a proxy.
/// @param op Binary operator.
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_BINARY_OP_RIGHT(op, result_type)                \
  template <typename V1, typename A1, typename T>                   \
  result_type                                                       \
  operator op(T const& lhs, proxy<V1,A1> const& rhs)                \
  {                                                                 \
    typedef typename std::common_type<T, V1>::type ct_t;            \
    return static_cast<ct_t const&>(lhs) op static_cast<ct_t>(rhs); \
  }

#define STAPL_PROXY_BINARY_OP(op, result_type)                      \
  STAPL_PROXY_BINARY_OP_BOTH(op, result_type)                       \
  STAPL_PROXY_BINARY_OP_LEFT(op, result_type)                       \
  STAPL_PROXY_BINARY_OP_RIGHT(op, result_type)

  STAPL_PROXY_BINARY_OP(==, bool)
  STAPL_PROXY_BINARY_OP(!=, bool)
  STAPL_PROXY_BINARY_OP( <, bool)
  STAPL_PROXY_BINARY_OP(<=, bool)
  STAPL_PROXY_BINARY_OP( >, bool)
  STAPL_PROXY_BINARY_OP(>=, bool)

#undef STAPL_PROXY_BINARY_OP_BOTH
#undef STAPL_PROXY_BINARY_OP_LEFT
#undef STAPL_PROXY_BINARY_OP_RIGHT
#undef STAPL_PROXY_BINARY_OP


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to evaluate the given binary operation @p Op
///        when both input parameters are proxies.
///
/// @tparam Op Binary operation type.
/// @tparam V Return type.
/// @tparam A1,A2 Accessor types used for the proxies.
//////////////////////////////////////////////////////////////////////
template<typename Op, typename V, typename A1, typename A2,
         typename E1 = void, typename E2 = void>
struct handle_both_op
{
  typedef V result_type;

  result_type
  operator()(proxy<V,A1> const& lhs, proxy<V,A2> const& rhs) const
  {
    return Op()(static_cast<V>(lhs), static_cast<V>(rhs));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to evaluate the given binary operation @p Op
///        when both input parameters are proxies and the value types differ.
///
/// @tparam Op Binary operation type.
/// @tparam V Return type.
/// @tparam V1,V2 Value types used for the proxies.
/// @tparam A1,A2 Accessor types used for the proxies.
//////////////////////////////////////////////////////////////////////
template<typename Op, typename V, typename V1, typename V2,
         typename A1, typename A2,
         typename E1 = void, typename E2 = void>
struct handle_both_op_convert
{
  typedef V result_type;

  result_type
  operator()(proxy<V1,A1> const& lhs, proxy<V2,A2> const& rhs) const
  {
    return Op()(static_cast<V>(lhs), static_cast<V>(rhs));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to evaluate the given binary operation @p Op
///        when both input parameters are proxies but the left hand
///        side proxy supports deferred evaluation (it does not
///        necessarily reference a concrete value yet).
///
/// @tparam Op Binary operation type.
/// @tparam V Return type.
/// @tparam A1,A2 Accessor types used for the proxies.
//////////////////////////////////////////////////////////////////////
template<typename Op, typename V, typename A1, typename A2, typename E2>
struct handle_both_op<Op, V, A1, A2,
                      typename A1::deferred_evaluation_accessor_, E2>
{
  typedef typename result_of::transform_reference<
    proxy<V,A1>,
    typename result_of::bind2nd<Op, V>::type
  >::type result_type;

  result_type
  operator()(proxy<V,A1> const& lhs, proxy<V,A2> const& rhs) const
  {
    return transform_reference(lhs, bind2nd(Op(), static_cast<V>(rhs)));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to evaluate the given binary operation @p Op
///        when both input parameters are proxies but the right
///        hand side proxy supports deferred evaluation (it does not
///        necessarily reference a concrete value yet).
///
/// @tparam Op Binary operation type.
/// @tparam V Return type.
/// @tparam A1,A2 Accessor types used for the proxies.
//////////////////////////////////////////////////////////////////////
template<typename Op, typename V, typename A1, typename A2, typename E1>
struct handle_both_op<Op, V, A1, A2, E1,
                      typename A2::deferred_evaluation_accessor_>
{
  typedef typename result_of::transform_reference<
    proxy<V,A2>,
    typename result_of::bind1st<Op, V>::type
  >::type result_type;

  result_type
  operator()(proxy<V,A1> const& lhs, proxy<V,A2> const& rhs) const
  {
    return transform_reference(rhs, bind1st(Op(), static_cast<V>(lhs)));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to evaluate the given binary operation @p
///        Op when both input parameters are proxies both proxies
///        support deferred evaluation (it does not necessarily
///        reference a concrete value yet).
///
/// @tparam Op Binary operation type.
/// @tparam V Return type.
/// @tparam A1,A2 Accessor types used for the proxies.
//////////////////////////////////////////////////////////////////////
template<typename Op, typename V, typename A1, typename A2>
struct handle_both_op<Op, V, A1, A2,
                      typename A1::deferred_evaluation_accessor_,
                      typename A2::deferred_evaluation_accessor_>
{
  typedef typename result_of::
    transform_reference<proxy<V,A1>, proxy<V,A2>, Op>::type result_type;

  result_type
  operator()(proxy<V,A1> const& lhs, proxy<V,A2> const& rhs) const
  {
    return transform_reference(lhs, rhs, Op());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to evaluate the given binary operation @p Op
///        when the left hand side input parameter is a proxy.
///
/// @tparam Op Binary operation type.
/// @tparam V Return type.
/// @tparam A Accessor type used for the proxy.
//////////////////////////////////////////////////////////////////////
template<typename Op, typename V, typename A, typename E = void>
struct handle_left_op
{
  typedef V result_type;

  result_type
  operator()(proxy<V,A> const& lhs, V const& rhs) const
  {
    return Op()(static_cast<V>(lhs), rhs);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to evaluate the given binary operation @p Op
///        when the left hand side input parameter is a proxy with
///        support for deferred evaluation (it does not necessarily
///        reference a concrete value yet).
///
/// @tparam Op Binary operation type.
/// @tparam V Return type.
/// @tparam A Accessor type used for the proxy.
//////////////////////////////////////////////////////////////////////
template<typename Op, typename V, typename A>
struct handle_left_op<Op, V, A, typename A::deferred_evaluation_accessor_>
{
  typedef typename result_of::transform_reference<
    proxy<V,A>,
    typename result_of::bind2nd<Op, V>::type
  >::type result_type;

  result_type
  operator()(proxy<V,A> const& lhs, V const& rhs) const
  {
    return transform_reference(lhs, stapl::bind2nd(Op(), rhs));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to evaluate the given binary operation @p Op
///        when the right hand side input parameter is a proxy.
///
/// @tparam Op Binary operation type.
/// @tparam V Return type.
/// @tparam A Accessor type used for the proxy.
//////////////////////////////////////////////////////////////////////
template<typename Op, typename V, typename A, typename E = void>
struct handle_right_op
{
  typedef V result_type;

  result_type
  operator()(V const& lhs, proxy<V,A> const& rhs) const
  {
    return Op()(lhs, static_cast<V>(rhs));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to evaluate the given binary operation @p Op
///        when the right hand side input parameter is a proxy with
///        support for deferred evaluation (it does not necessarily
///        reference a concrete value yet).
///
/// @tparam Op Binary operation type.
/// @tparam V Return type.
/// @tparam A Accessor type used for the proxy.
//////////////////////////////////////////////////////////////////////
template<typename Op, typename V, typename A>
struct handle_right_op<Op, V, A, typename A::deferred_evaluation_accessor_>
{
  typedef typename result_of::transform_reference<
    proxy<V,A>,
    typename result_of::bind1st<Op, V>::type
  >::type result_type;

  result_type
  operator()(V const& lhs, proxy<V,A> const& rhs) const
  {
    return transform_reference(rhs, bind1st(Op(), lhs));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Macro used to specialize the given binary operation @p op
///        when both parameters are proxies.
/// @param op Binary operator.
/// @param functor Functor type used to compute the specified operation.
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_BINARY_OP_BOTH(op, functor)                     \
  template <typename V, typename A1, typename A2>                   \
  typename handle_both_op<functor<V>, V, A1, A2>::result_type       \
  operator op(proxy<V,A1> const& lhs, proxy<V,A2> const& rhs)       \
  {                                                                 \
    return handle_both_op<functor<V>, V, A1, A2>()(lhs, rhs);       \
  }


//////////////////////////////////////////////////////////////////////
/// @brief Macro used to specialize the given binary operation @p op
///        when both parameters are proxies and the value types of the
///        proxies differ.
///
/// Type conversion to the common type is performed before the operation
/// is invoked.
///
/// @param op Binary operator.
/// @param functor Functor type used to compute the specified operation.
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_BINARY_OP_CONVERT(op, functor)                  \
  template <typename V1, typename A1, typename V2, typename A2>     \
  typename std::common_type<V1, V2>::type                           \
  operator op(proxy<V1,A1> const& lhs, proxy<V2,A2> const &rhs)     \
  {                                                                 \
    using type = typename std::common_type<V1, V2>::type;           \
    return handle_both_op_convert<functor<type>, type,              \
             V1, V2, A1, A2>()(lhs,rhs);                            \
  }


//////////////////////////////////////////////////////////////////////
/// @brief Macro used to specialize the given binary operation @p op
///        when the left hand side is a proxy.
/// @param op Binary operator.
/// @param functor Functor type used to compute the specified operation.
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_BINARY_OP_LEFT(op, functor)                     \
  template <typename V, typename A>                                 \
  typename handle_left_op<functor<V>, V, A>::result_type            \
  operator op(proxy<V,A> const& lhs, V const& rhs)                  \
  {                                                                 \
    return handle_left_op<functor<V>, V, A>()(lhs, rhs);            \
  }


//////////////////////////////////////////////////////////////////////
/// @brief Macro used to specialize the given binary operation @p op
///        when the right hand side is a proxy.
/// @param op Binary operator.
/// @param functor Functor type used to compute the specified operation.
//////////////////////////////////////////////////////////////////////
#define STAPL_PROXY_BINARY_OP_RIGHT(op, functor)                    \
  template <typename V, typename A>                                 \
  typename handle_right_op<functor<V>, V, A>::result_type           \
  operator op(V const& lhs, proxy<V,A> const& rhs)                  \
  {                                                                 \
    return handle_right_op<functor<V>, V, A>()(lhs, rhs);           \
  }


#define STAPL_PROXY_BINARY_OP(op, functor)                          \
  STAPL_PROXY_BINARY_OP_BOTH(op, functor)                           \
  STAPL_PROXY_BINARY_OP_CONVERT(op, functor)                        \
  STAPL_PROXY_BINARY_OP_LEFT(op, functor)                           \
  STAPL_PROXY_BINARY_OP_RIGHT(op, functor)


STAPL_PROXY_BINARY_OP(+, plus)
STAPL_PROXY_BINARY_OP(-, minus)
STAPL_PROXY_BINARY_OP(*, multiplies)
STAPL_PROXY_BINARY_OP(/, divides)
STAPL_PROXY_BINARY_OP(%, modulus)

#undef STAPL_PROXY_BINARY_OP
#undef STAPL_PROXY_BINARY_OP_BOTH
#undef STAPL_PROXY_BINARY_OP_CONVERT
#undef STAPL_PROXY_BINARY_OP_LEFT
#undef STAPL_PROXY_BINARY_OP_RIGHT

} // namespace stapl

#endif // STAPL_P_CONTAINER_VIEW_PROXY_OPS_HPP
