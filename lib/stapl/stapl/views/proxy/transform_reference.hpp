/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TRANSFORM_REFERENCE_HPP
#define STAPL_VIEWS_TRANSFORM_REFERENCE_HPP

#ifndef BOOST_RESULT_OF_NUM_ARGS
#  define BOOST_RESULT_OF_NUM_ARGS 11
#endif

#include <stapl/views/proxy.h>
#include <functional>
#include <boost/utility/result_of.hpp>
#include <boost/function.hpp>
#include <stapl/views/null_reference.hpp>


namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Accessor to defer the application of a given unary
///        @p Functor until the evaluation is required.
//////////////////////////////////////////////////////////////////////
template<typename Reference, typename Functor>
struct unary_tm_accessor
  : private Functor
{
private:
  typedef typename
       proxy_core_access::value_type<Reference>::type           base_value_t;

  typedef typename
       proxy_core_access::accessor_type<Reference>::type        base_accessor_t;

  typedef typename
       boost::result_of<Functor(base_value_t)>::type            value_type;

  Reference m_base_ref;

  base_accessor_t const& base_accessor(void) const
  {
    return proxy_core_access::accessor(m_base_ref);
  }

public:
  typedef void deferred_evaluation_accessor_;

  unary_tm_accessor(Reference const& ref, Functor const& func)
    : Functor(func),
      m_base_ref(ref)
  { }

  unary_tm_accessor(unary_tm_accessor const& rhs)
    : Functor(rhs.functor()),
      m_base_ref(rhs.m_base_ref)
  { }

  unary_tm_accessor(void)
    : m_base_ref()
  { }

  explicit unary_tm_accessor(null_reference const& n)
    : m_base_ref(n)
  { }

  unary_tm_accessor&
  operator=(unary_tm_accessor const& rhs)
  {
    if (&rhs == this)
      return *this;

    static_cast<Functor&>(*this) = rhs.functor();

    proxy_core_access::reset(m_base_ref, rhs.m_base_ref);

    return *this;
  }

  void define_type(typer& t)
  {
    t.base<Functor>(*this);
    t.member(m_base_ref);
  }

  Functor const& functor(void) const
  {
    return static_cast<Functor const&>(*this);
  }

  bool is_null(void) const
  {
    return m_base_ref.is_null();
  }

  value_type read(void) const
  {
    stapl_assert(base_accessor().available(),
      "unary_tm_accessor: attempting to read unavailable value");

    // should I be passing the m_base_reference or it's value type?
    //
    return functor()(base_accessor().read());
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Address private visibility for the following methods.
  //////////////////////////////////////////////////////////////////////
  bool is_local(void) const
  {
    return base_accessor().is_local();
  }

  bool available(void) const
  {
    return base_accessor().available();
  }

  template<typename Notifier>
  void request_notify(Notifier notifier) const
  {
    base_accessor().request_notify(notifier);
  }
}; // unary_tm_accessor


//////////////////////////////////////////////////////////////////////
/// @brief Defines a counter for notifications, forwarding the
///        notification signal when @p N notifications are received.
/// @todo Probably this class should be moved to a different directory
///       (paragraph perhaps).
//////////////////////////////////////////////////////////////////////
template<std::size_t N>
struct basic_counter_notifier
{
private:
  typedef boost::function<void (void)>   notifier_t;

  std::size_t                            m_count;
  notifier_t                             m_notifier;

  basic_counter_notifier(basic_counter_notifier const&);

  basic_counter_notifier& operator=(basic_counter_notifier const&);

public:
  template<typename Notifier>
  explicit basic_counter_notifier(Notifier notifier)
    : m_count(0),
      m_notifier(notifier)
  { }

  /// FIXME - circular header in runtime...
  ///
  /// STAPL_USE_MANAGED_ALLOC(basic_counter_notifier)

  void operator()(void)
  {
    if (++m_count == N)
    {
      m_notifier();
      delete this;
    }
  }
}; // struct basic_counter_notifier


//////////////////////////////////////////////////////////////////////
/// @brief Accessor to defer the application of a given binary
///        @p Functor until the evaluation is required.
//////////////////////////////////////////////////////////////////////
template<typename Reference1, typename Reference2, typename Functor>
struct binary_tm_accessor
  : private Functor
{
private:
  typedef typename
       proxy_core_access::value_type<Reference1>::type      base_value1_t;

  typedef typename
       proxy_core_access::value_type<Reference2>::type      base_value2_t;

  typedef typename
       proxy_core_access::accessor_type<Reference1>::type   base_accessor1_t;

  typedef typename
       proxy_core_access::accessor_type<Reference2>::type   base_accessor2_t;

  typedef typename boost::result_of<
     Functor(base_value1_t, base_value2_t)>::type           value_type;

  Reference1 m_base_ref1;
  Reference2 m_base_ref2;

  base_accessor1_t const& base_accessor1(void) const
  {
    return proxy_core_access::accessor(m_base_ref1);
  }

  base_accessor2_t const& base_accessor2(void) const
  {
    return proxy_core_access::accessor(m_base_ref2);
  }

public:
  typedef void deferred_evaluation_accessor_;

  binary_tm_accessor(Reference1 const& ref1,
                     Reference2 const& ref2,
                     Functor const& func)
    : Functor(func),
      m_base_ref1(ref1),
      m_base_ref2(ref2)
  { }

  binary_tm_accessor(binary_tm_accessor const& rhs)
    : Functor(rhs.functor()),
      m_base_ref1(rhs.m_base_ref1),
      m_base_ref2(rhs.m_base_ref2)
  { }

  binary_tm_accessor(void)
    : m_base_ref1(),
      m_base_ref2()
  { }

  explicit binary_tm_accessor(null_reference const& n)
    : m_base_ref1(n),
      m_base_ref2(n)
  { }

  binary_tm_accessor&
  operator=(binary_tm_accessor const& rhs)
  {
    if (&rhs == this)
      return *this;

    static_cast<Functor&>(*this) = rhs.functor();

    proxy_core_access::reset(m_base_ref1, rhs.m_base_ref1);
    proxy_core_access::reset(m_base_ref2, rhs.m_base_ref2);

    return *this;
  }

  void define_type(typer& t)
  {
    t.base<Functor>(*this);
    t.member(m_base_ref1);
    t.member(m_base_ref2);
  }

  Functor const& functor(void) const
  {
    return static_cast<Functor const&>(*this);
  }

  bool is_null(void) const
  {
    stapl_assert(m_base_ref1.is_null() == m_base_ref2.is_null(),
      "binary_tm_accessor::is_null(), differing is_null for base_refs");

    return m_base_ref1.is_null();
  }

  value_type read(void) const
  {
    stapl_assert(base_accessor1().available(),
      "binary_tm_accessor: attempting to read unavailable value (first)");

    stapl_assert(base_accessor2().available(),
      "binary_tm_accessor: attempting to read unavailable value (second)");

    // should I be passing the m_base_reference or it's value type?
    //
    return functor()(base_accessor1().read(), base_accessor2().read());
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Address private visibility for the following methods.
  //////////////////////////////////////////////////////////////////////
  bool is_local(void) const
  {
    return base_accessor1().is_local() && base_accessor2().is_local();
  }

  bool available(void) const
  {
    return base_accessor1().available() && base_accessor2().available();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Notifies when the referenced values are available for
  ///        accessing them.
  //////////////////////////////////////////////////////////////////////
  template<typename Notifier>
  void request_notify(Notifier notifier) const
  {
    typedef basic_counter_notifier<2> aggr_notifier_t;

    // FIXME - is there a non heap way to do this? Not that i know of...
    // (unless we aggregate conforming requests across all instances of this
    // accessor (perhaps a static?)
    //
    aggr_notifier_t* aggr_notifier_ptr = new aggr_notifier_t(notifier);

    base_accessor1().request_notify(std::bind(std::ref(*aggr_notifier_ptr)));
    base_accessor2().request_notify(std::bind(std::ref(*aggr_notifier_ptr)));
  }
}; // binary_tm_accessor

} // namespace detail


namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Helper to determine the deferred reference based on the
///        given references (@p References1, @p References2) and
///        binary @p Functor type.
/// @todo Should be able to easily make this polymorphic and scrub type.
//////////////////////////////////////////////////////////////////////
template<typename Reference1, typename Reference2, typename Functor = void>
struct transform_reference
{
private:
  // The type of the element Reference is referring to.
  typedef typename
      proxy_core_access::value_type<Reference1>::type            value1_t;
  typedef typename
      proxy_core_access::value_type<Reference2>::type            value2_t;

  /// The type of the element this transformed reference will refer to.
  typedef typename
      boost::result_of<Functor(value1_t, value2_t)>::type        result_t;

public:
  typedef proxy<
    result_t,
    detail::binary_tm_accessor<
      Reference1, Reference2, Functor> >                         type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper to determine the deferred reference based on the
///        given @p References and unary @p Functor type.
//////////////////////////////////////////////////////////////////////
template<typename Reference, typename Functor>
struct transform_reference<Reference, Functor, void>
{
private:
  /// The type of the element Reference is referring to.
  typedef typename
      proxy_core_access::value_type<Reference>::type             value_t;

  /// The type of the element this transformed reference will refer to.
  typedef typename
      boost::result_of<Functor(value_t)>::type                   result_t;

public:
  typedef proxy<result_t,
                detail::unary_tm_accessor<Reference, Functor> >  type;
};

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Helper to construct a deferred reference to evaluate the
///        given unary functor @p func.
//////////////////////////////////////////////////////////////////////
template<typename Reference, typename Functor>
typename result_of::transform_reference<Reference, Functor>::type
transform_reference(Reference ref, Functor func)
{
  // The proxy type this function will return.
  typedef typename
      result_of::transform_reference<Reference, Functor>::type   ref_t;

  // The accessor of the proxy this function will return.
  typedef typename
      proxy_core_access::accessor_type<ref_t>::type              accessor_t;

  return ref_t(accessor_t(ref, func));
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper to construct a deferred reference to evaluate the
///        given binary functor @p func.
//////////////////////////////////////////////////////////////////////
template<typename Reference1, typename Reference2, typename Functor>
typename result_of::transform_reference<Reference1, Reference2, Functor>::type
transform_reference(Reference1 ref1, Reference2 ref2, Functor func)
{
  // The proxy type this function will return.
  typedef typename result_of::transform_reference<
    Reference1, Reference2, Functor>::type                       ref_t;

  // The accessor of the proxy this function will return.
  typedef typename
    proxy_core_access::accessor_type<ref_t>::type                accessor_t;

  return ref_t(accessor_t(ref1, ref2, func));
}

} // namespace stapl

#endif // STAPL_VIEWS_TRANSFORM_REFERENCE_HPP
