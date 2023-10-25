/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_PROXY_MEMBER_ACCESSOR_HPP
#define STAPL_VIEWS_PROXY_MEMBER_ACCESSOR_HPP

#include <stapl/views/proxy/accessor.hpp>
#include <stapl/views/proxy/accessor_base.hpp>

namespace stapl {

namespace proxy_impl {

template<typename Selector, typename Functor>
struct selector_apply_set
  : private Selector,
    private Functor
{
  typedef void result_type;

  selector_apply_set(Selector const& s, Functor const& f)
    : Selector(s), Functor(f)
  { }

  template<typename T>
  void operator()(T& t) const
  {
    static_cast<Functor const&>(*this)(
      static_cast<Selector const&>(*this)(t));
  }

  void define_type(typer& t)
  {
    t.base<Selector>(*this);
    t.base<Functor>(*this);
  }
};


template<typename Selector, typename Functor>
struct selector_apply_get
  : private Selector, private Functor
{
  selector_apply_get(Selector const& s, Functor const& f)
    : Selector(s), Functor(f)
  { }

  typedef typename Functor::result_type result_type;

  template<typename T>
  result_type operator()(T& t) const
  {
    return static_cast<Functor const&>(*this)(
      static_cast<Selector const&>(*this)(t));
  }

  template<typename T>
  result_type operator()(T const& t) const
  {
    return static_cast<Functor const&>(*this)(
      static_cast<Selector const&>(*this)(t));
  }

  void define_type(typer& t)
  {
    t.base<Selector>(*this);
    t.base<Functor>(*this);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction that returns @p OptionalAccessor if specified
/// and otherwise returns @p Selector::accessor_type.
//////////////////////////////////////////////////////////////////////
template<typename Selector, typename... OptionalAccessor>
struct compute_accessor;


template<typename Selector>
struct compute_accessor<Selector>
{
  using type = typename Selector::accessor_type;
};


template<typename Selector, typename Accessor>
struct compute_accessor<Selector, Accessor>
{
  using type = Accessor;
};

} // namespace proxy_impl


//////////////////////////////////////////////////////////////////////
/// @brief Accessor used to reference struct/class member through the
///        struct/class @c Accessor.
///
/// This accessor uses a parent accessor to forward the method
/// invocations, encapsulating the member's method invocation before
/// it is forwarded.
///
/// @tparam T Type of data accessed.
/// @tparam Selector Type of functor to get/set the referenced data.
/// @tparam OptionalAccessor Type of accessor used to forward
///   the invocation requests.  If not provided, the accessor type must
///   be reflected in @p Selector.
/// @todo Remove property_reference usage when graph accessors conform and
/// are supported by member_accessor.
//////////////////////////////////////////////////////////////////////
template<typename Selector, typename... OptionalAccessor>
class member_accessor
  : private Selector
{
private:
  friend class accessor_core_access;

  using accessor_type =
    typename proxy_impl::compute_accessor<Selector, OptionalAccessor...>::type;

  accessor_type m_parent;

public:
  using value_type = typename Selector::result_type;

  member_accessor(accessor_type const& acc, Selector const& s = Selector())
    : Selector(s), m_parent(acc)
  { }

  template<typename F>
  void apply_set(F const& f) const
  {
    accessor_core_access::apply_set(
      m_parent,
      proxy_impl::selector_apply_set<Selector, F>(
        static_cast<Selector const&>(*this), f));
  }

  template<typename F>
  typename F::result_type apply_get(F const& f) const
  {
    return accessor_core_access::apply_get(
      m_parent,
      proxy_impl::selector_apply_get<Selector, F>(
       static_cast<Selector const&>(*this), f));
  }

  value_type read(void) const
  {
    using F = stapl::identity<value_type>;

    return accessor_core_access::apply_get(
      m_parent,
      proxy_impl::selector_apply_get<Selector, F>(
        static_cast<Selector const&>(*this), F()));
  }

  template<typename U>
  void write(U const& value) const
  {
    using F = detail::assign_ref<U>;

    accessor_core_access::apply_set(
      m_parent,
      proxy_impl::selector_apply_set<Selector, F>(
        static_cast<Selector const&>(*this), F(value)));
  }

  template<typename Class, typename... Args>
  void invoke(void (Class::* const pmf)(Args...),
              typename std::decay<Args>::type const&... args) const

  {
    using F = detail::apply_set_helper<
                void (Class::*)(Args...), typename std::decay<Args>::type...>;

    accessor_core_access::apply_set(
      m_parent,
      proxy_impl::selector_apply_set<Selector, F>
        (static_cast<Selector const&>(*this), F(pmf, args...)));
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn invoke(Rtn (Class::* const pmf)(Args...),
             typename std::decay<Args>::type const&... args) const
  {
    using F = detail::apply_get_helper<
                Rtn (Class::*)(Args...), typename std::decay<Args>::type...>;

    return accessor_core_access::apply_get(
      m_parent,
      proxy_impl::selector_apply_get<Selector, F>(
        static_cast<Selector const&>(*this), F(pmf, args...)));
  }


  template<typename Class, typename Rtn, typename... Args>
  Rtn const_invoke(Rtn (Class::* const pmf)(Args...) const,
                   typename std::decay<Args>::type const&... args) const

  {
    using F = detail::const_apply_get_helper<
                Rtn (Class::*)(Args...) const,
                typename std::decay<Args>::type...>;

    return accessor_core_access::apply_get(
      m_parent,
      proxy_impl::selector_apply_get<Selector, F>(
        static_cast<Selector const&>(*this), F(pmf, args...)));
  }


  value_type& ref() const
  {
    return this->select()(m_parent.ref());
  }

  bool is_null(void) const
  {
    return m_parent.is_null();
  }

  bool is_local(void) const
  {
    return m_parent.is_local();
  }

  void define_type(typer& t)
  {
    t.base<Selector>(*this);
    t.member(m_parent);
  }

  /// @brief Allow proxies to compares containing graph vertex to be properly
  /// constructed (not accessed).
  using property_reference       = void;

  /// @brief Allow proxies to compares containing graph vertex to be properly
  /// constructed (not accessed).
  using const_property_reference = void;
}; // class member_accessor


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of stand-alone begin for rvalue references
/// to a proxy with an member_accessor.
///
/// Example:
/// auto it = stapl::begin(proxy<T, member_accessor<Outer>>(t));
///
/// @ingroup iteratorSpecializations
//////////////////////////////////////////////////////////////////////
template <typename T, typename Outer>
auto begin(stapl::proxy<T, member_accessor<Outer>>&& t)
  -> decltype(
    const_cast<stapl::proxy<T, member_accessor<Outer>> const&&>(t).begin())
{
  return const_cast<stapl::proxy<T, member_accessor<Outer>> const&&>(t).begin();
}


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of stand-alone begin for lvalue references
/// to a proxy with an member_accessor.
///
/// Example:
/// proxy<T, member_accessor<Outer>> p(t);
/// auto it = stapl::begin(p);
///
/// @ingroup iteratorSpecializations
//////////////////////////////////////////////////////////////////////
template <typename T, typename Outer>
auto begin(stapl::proxy<T, member_accessor<Outer>>& t)
  -> decltype(
    const_cast<stapl::proxy<T, member_accessor<Outer>> const &>(t).begin())
{
  return const_cast<stapl::proxy<T, member_accessor<Outer>> const &>(t).begin();
}


} // namespace stapl

#endif // STAPL_VIEWS_PROXY_MEMBER_ACCESSOR_HPP
