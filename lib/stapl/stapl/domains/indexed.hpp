/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DOMAINS_INDEXED_DOMAIN_HPP
#define STAPL_DOMAINS_INDEXED_DOMAIN_HPP

#include <cstddef>
#include <iosfwd>
#include <iostream>

#include <stapl/utility/vs_map.hpp>
#include <stapl/utility/pack_ops.hpp>
#include <stapl/utility/use_default.hpp>
#include <stapl/domains/indexed_fwd.hpp>
#include <stapl/views/mapping_functions/linearization.hpp>
#include <stapl/containers/multiarray/multiarray_localize.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/runtime/serialization_fwd.hpp>

namespace stapl {

namespace domain_impl {

struct identity_expand
{
  template<std::size_t N, typename T>
  static T apply(T t)
  { return t; }
};


struct contains
{
  template <typename First, typename Last, typename Index>
  bool
  operator()(First first, Last last, Index index) const
  {
    return
      first <= static_cast<First>(index) && static_cast<Last>(index) <= last;
  }
};


template <typename T>
struct is_invalid
{
private:
  T m_invalid_id;

public:
  is_invalid(T invalid_id)
    : m_invalid_id(invalid_id)
  { }

  template <typename Value>
  bool
  operator()(Value const& val1, Value const& val2) const
  {
    return val1 == m_invalid_id || val2 == m_invalid_id;
  }
};


/////////////////////////////////////////////////////////////////////
/// @brief Base class of @ref indexed_domain that holds traversal order
/// independent functionality.
/////////////////////////////////////////////////////////////////////
template<typename T, int N, typename = make_index_sequence<N>>
class indexed_domain_base;


template<typename T, int N, std::size_t... Indices>
class indexed_domain_base<T, N, index_sequence<Indices...>>
{
public:
  using dimension_type   = std::integral_constant<int, N>;
  using dimensions_type  = typename homogeneous_tuple_type<N, T>::type;
  using index_type       = typename homogeneous_tuple_type<N, T>::type;
  using size_type        = typename homogeneous_tuple_type<N, size_t>::type;
  using linear_size_type = size_t;


protected:
  index_type  m_first;
  index_type  m_last;

  /// @brief Flag to indicate that this domain references all the elements in
  ///        the container.
  bool       m_full_container;

  static bool check_size(size_type size)
  {
    return (pack_ops::functional::plus_(get<Indices>(size)...) > 0);
  }

  indexed_domain_base(void)
    : m_first(identity_expand::apply<Indices>(index_bounds<T>::invalid())...),
      m_last(identity_expand::apply<Indices>(index_bounds<T>::invalid())...),
      m_full_container(false)
  { }

  indexed_domain_base(size_type size)
    : m_full_container(true)
  {
    if (check_size(size))
    {
      m_first = index_type(identity_expand::apply<Indices>(0)...);
      m_last  = index_type((get<Indices>(size) - 1)...);

      return;
    }

    m_first = index_type(
      identity_expand::apply<Indices>(index_bounds<T>::invalid())...);

    m_last = index_type(
      identity_expand::apply<Indices>(index_bounds<T>::invalid())...);
  }

  /////////////////////////////////////////////////////////////////////
  /// @todo Commented code disables short circuiting. Replace when
  /// appropriate variadic primitive is available.
  /////////////////////////////////////////////////////////////////////
  indexed_domain_base(index_type const& first,
                      index_type const& last,
                      bool full_container)
    : m_first(first),
      m_last(last),
      m_full_container(full_container)
  {

    // if (pack_ops::functional::or_(
    //       (get<Indices>(first) > get<Indices>(last))...))
    if (vs_map_reduce(stapl::greater<T>(), stapl::logical_or<bool>(),
        false, first, last))
    {
      m_first = index_type(
        identity_expand::apply<Indices>(index_bounds<T>::invalid())...);

      m_last = index_type(
        identity_expand::apply<Indices>(index_bounds<T>::invalid())...);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new domain by restricting the given @p other
  ///        domain to be [first..last].
  //////////////////////////////////////////////////////////////////////
  indexed_domain_base(index_type const& first,
                      index_type const& last,
                      indexed_domain_base const& other)
    : m_first(first),
      m_last(last),
      m_full_container((other.m_first == m_first && other.m_last == m_last) ?
        other.m_full_container : false)
  { }

public:
  //////////////////////////////////////////////////////////////////////
  //  common functionality for all domains
  //////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c True if all the indexes in the domain are
  ///        contiguous. returns @c False otherwise.
  //////////////////////////////////////////////////////////////////////
  bool is_contiguous(void) const
  {
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c True if this domain has all the indexes to
  ///        reference all the data in a container.
  //////////////////////////////////////////////////////////////////////
  bool is_same_container_domain(void) const
  {
    return m_full_container;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Provide single dimension domain over @p Index dimension.
  //////////////////////////////////////////////////////////////////////
  template<int Index>
  indexed_domain<T,1> get_domain(void) const
  {
    return indexed_domain<T, 1>(
      get<Index>(m_first), get<Index>(m_last), m_full_container);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns least index of domain
  //////////////////////////////////////////////////////////////////////
  index_type first(void) const
  {
    return m_first;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns greatest index of domain
  //////////////////////////////////////////////////////////////////////
  index_type last(void) const
  {
    return m_last;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns True if specified index is in domain
  /// @todo Commented code disables short circuiting. Replace when
  /// appropriate variadic primitive is available.
  //////////////////////////////////////////////////////////////////////
  bool contains(index_type const& index) const
  {
    return vs_map_reduce(domain_impl::contains(),
                         stapl::logical_and<T>(),
                         true, m_first, m_last, index);
    // return pack_ops::functional::and_(
    //   (get<Indices>(m_first) <= get<Indices>(index)
    //   && get<Indices>(index) <= get<Indices>(m_last))...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns True if specified index is in domain
  /// @todo Commented code disables short circuiting. Replace when
  /// appropriate variadic primitive is available.
  //////////////////////////////////////////////////////////////////////
  template <typename... GID>
  bool contains(GID... indices) const
  {
    auto gid_tuple = stapl::make_tuple(indices...);
    return vs_map_reduce(domain_impl::contains(),
                         stapl::logical_and<T>(),
                         true, m_first, m_last, gid_tuple);
    // return pack_ops::functional::and_(
    //   (get<Indices>(m_first) <= pack_ops::get<Indices>(indices...)
    //   && pack_ops::get<Indices>(indices...) <= get<Indices>(m_last))...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c True if this domain has no indices
  /// @todo Commented code disables short circuiting. Replace when
  /// appropriate variadic primitive is available.
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return vs_map_reduce(
      domain_impl::is_invalid<T>(index_bounds<T>::invalid()),
      stapl::logical_or<bool>(), false, m_first, m_last);

    // return pack_ops::functional::or_(
    //   (get<Indices>(m_first) == index_bounds<T>::invalid()
    //   || get<Indices>(m_last) == index_bounds<T>::invalid())...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of indices in this domain
  //////////////////////////////////////////////////////////////////////
  linear_size_type size(void) const
  {
    if (empty())
      return 0;

    return pack_ops::functional::multiplies_(get<Indices>(dimensions())...);
  }

  size_type dimensions(void) const
  {
    if (empty())
      return m_first;

    return size_type((get<Indices>(m_last) - get<Indices>(m_first) + 1)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns how many positions are needed to advance from the
  ///        index @p i to reach the index @p j.
  /// @note The returned value does not define in which direction to
  ///       advance.
  //////////////////////////////////////////////////////////////////////
  size_type distance(index_type i, index_type j) const
  {
    return (j > i) ? (j - i) : (i - j);
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Commented code disables short circuiting. Replace when
  /// appropriate variadic primitive is available.
  //////////////////////////////////////////////////////////////////////
  bool less_than(index_type const& i0, index_type const& i1) const
  {
    // return pack_ops::functional::and_(
    //   (get<Indices>(i0) < get<Indices>(i1))...);
    return vs_map_reduce(stapl::less<T>(), stapl::logical_and<bool>(),
             true, i0, i1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "INDEXED_DOMAIN %p: " << this;

    if (msg)
      std::cerr << msg;

    std::cerr << "\n";
    std::cerr << " first " << first();
    std::cerr << " last " << last();
    std::cerr << " size " << size() << "\n";
  }

  void define_type(typer& t)
  {
    t.member(m_first);
    t.member(m_last);
    t.member(m_full_container);
  }
}; // class indexed_domain_base

} // namespace domain_impl


template<typename T, int N, typename ...OptionalTraversal>
class indexed_domain
  : public domain_impl::indexed_domain_base<T, N>
{
public:
  using traversal_type =
    typename tuple_element<
      0,
      typename compute_type_parameters<
        tuple<typename default_traversal<N>::type>, OptionalTraversal...
      >::type
    >::type;

  using index_type       = typename homogeneous_tuple_type<N, T>::type;
  using gid_type         = typename homogeneous_tuple_type<N, T>::type;
  using size_type        = typename homogeneous_tuple_type<N, size_t>::type;
  using linear_size_type = size_t;

private:
  using base_t                     = domain_impl::indexed_domain_base<T, N>;
  using linearization_type         = nd_linearize<gid_type, traversal_type>;
  using reverse_linearization_type =
    nd_reverse_linearize<gid_type, traversal_type>;

  linearization_type         m_linear_mf;
  reverse_linearization_type m_reverse_linear_mf;

public:
  indexed_domain(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a domain [0..@p size-1].
  /// @todo A constructor is required that takes index_type, index_type, size
  ///       for the multiarray.
  //////////////////////////////////////////////////////////////////////
  /*explicit*/ indexed_domain(size_type size)
    : base_t(size)
  {
    if (base_t::check_size(size))
    {
      m_linear_mf         = linearization_type(this->dimensions());
      m_reverse_linear_mf = reverse_linearization_type(this->dimensions());
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a domain containing the indexes from @p first
  ///        to @p last (included).
  ///
  /// If @p full_container is specified as @c true, the domain will be
  /// flagged as a domain that contains all the indexes of a container.
  //////////////////////////////////////////////////////////////////////
  indexed_domain(index_type const& first,
                 index_type const& last,
                 bool full_container = false)
    : base_t(first, last, full_container),
      m_linear_mf(this->dimensions()),
      m_reverse_linear_mf(this->dimensions())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new domain by restricting the given @p other
  ///        domain to be [first..last].
  //////////////////////////////////////////////////////////////////////
  indexed_domain(index_type const& first,
                 index_type const& last,
                 indexed_domain const& other)
    : base_t(first, last, other),
      m_linear_mf(this->dimensions()),
      m_reverse_linear_mf(this->dimensions())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns termination value for comparison
  //////////////////////////////////////////////////////////////////////
  index_type open_last(void) const
  {
    if (this->empty())
      return this->m_last;

    // else
    index_type olast(this->m_last);

    get<tuple_element<0, traversal_type>::type::value>(olast) += 1;
    return olast;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index resulting from advancing the index @p i, @p
  ///        n positions.
  ///
  /// @tparam Distance Integral type.
  //////////////////////////////////////////////////////////////////////
  template <typename Distance>
  index_type advance(index_type const& i, Distance m) const
  {
    const std::size_t local_linear =
      m_linear_mf(nd_localize<index_type>::apply(i, this->first()));

    std::size_t local_linear_next = local_linear + m;
    index_type local_next         = m_reverse_linear_mf(local_linear_next);

    return nd_globalize<index_type>::apply(local_next, this->first());
  }

  void define_type(typer& t)
  {
    t.base<base_t>(*this);
    t.member(m_linear_mf);
    t.member(m_reverse_linear_mf);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the resulting intersection with the given @p other
  ///        domain.
  //////////////////////////////////////////////////////////////////////
  indexed_domain operator&(indexed_domain const& other) const
  {
    if (!this->m_full_container)
    {
      index_type olower = other.first();
      index_type oupper = other.last();

      if (!this->empty())
      {
        olower = other.contains(this->m_first) ? this->m_first : olower;
        oupper = other.contains(this->m_last)  ? this->m_last  : oupper;
      } else {
        return *this;
      }

      if (!(other.contains(this->m_first)
            || other.contains(this->m_last)) )
        if (!(this->contains(olower) || this->contains(oupper)))
          return indexed_domain();

      // else
      return indexed_domain(olower, oupper);
    }
    else
    {
      // ensure the other domain is valid prior to returning.
      index_type first = other.first();
      index_type last  = other.last();
      if (this->contains(first) && this->contains(last))
        return indexed_domain(other.first(), other.last());
      else if (this->contains(first))
        return indexed_domain(first, this->m_last);
      else if (this->contains(last))
        return indexed_domain(this->m_first, last);
      else
        return indexed_domain(index_bounds<index_type>::invalid(),
                              index_bounds<index_type>::invalid());
    }
  }
};


template<typename T, int N, typename ...OptTrav>
std::ostream& operator<<(std::ostream& os,
                         indexed_domain<T, N, OptTrav...> const& d)
{
  print_tuple(os, d.first());
  os << "...";
  print_tuple(os, d.last());
  return os;
}


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for a one-dimensional domain where the indexes
///        are consecutive and numerable.
///
/// The domain is defined as a range of indexes between first index
/// and last index (included) @$ [first..last]@$.
///
/// @tparam T Index type.
/// @note @c T must be and integral type.
//////////////////////////////////////////////////////////////////////
template<typename T>
class indexed_domain<T, 1>
{
public:
  using traversal_type   = default_traversal<1>::type;
  using index_type       = T;
  using gid_type         = index_type;
  using size_type        = std::size_t;
  using linear_size_type = std::size_t;
  using dimensions_type  = std::size_t;
  using dimension_type   = std::integral_constant<int, 1>;

private:
  index_type  m_first;
  index_type  m_last;

  /// @brief Flag to indicate that this domain references all the elements in
  ///        the container.
  bool       m_full_container;

public:
  indexed_domain(void)
    : m_first(index_bounds<index_type>::invalid()),
      m_last(index_bounds<index_type>::invalid()),
      m_full_container(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a domain [0..@p size-1].
  /// @todo A constructor is required that takes index_type, index_type, size
  ///       for the multiarray.
  //////////////////////////////////////////////////////////////////////
  /*explicit*/ indexed_domain(size_type size)
    : m_full_container(true)
  {
    if (size > 0)
    {
      m_first = 0;
      m_last  = size-1;
    }
    else
    {
      m_first = index_bounds<index_type>::invalid();
      m_last  = index_bounds<index_type>::invalid();
    }
  }

  indexed_domain(tuple<size_type> const& size)
    : indexed_domain(stapl::get<0>(size))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a domain containing the indexes from @p first
  ///        to @p last (included).
  ///
  /// If @p full_container is specified as @c true, the domain will be
  /// flagged as a domain that contains all the indexes of a container.
  //////////////////////////////////////////////////////////////////////
  indexed_domain(index_type const& first,
                 index_type const& last,
                 bool full_container = false)
    : m_first(first),
      m_last(last),
      m_full_container(full_container)
  {
    if (first > last
        || last == index_bounds<index_type>::invalid())
    {
      m_first = index_bounds<index_type>::invalid();
      m_last  = index_bounds<index_type>::invalid();
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new domain by restricting the given @p other
  ///        domain to be [first..last].
  //////////////////////////////////////////////////////////////////////
  indexed_domain(index_type const& first,
                 index_type const& last,
                 indexed_domain const& other)
    : m_first(first),
      m_last(last),
      m_full_container(false)
  {
    if (other.m_first == m_first && other.m_last == m_last)
      m_full_container = other.m_full_container;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update the domain with a new last element.
  ///
  /// This method is invoked by the container update_distribution methods.
  //////////////////////////////////////////////////////////////////////
  void update(index_type const& new_last)
  {
    if (new_last <= m_last)
      return;
    m_last = new_last;
  }

  //////////////////////////////////////////////////////////////////////
  //  common functionality for all domains
  //////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////
  /// @brief Provide single dimension domain over @p Index dimension.
  ///
  /// For the 1D specialization, it simply returns the 1D domain itself.
  //////////////////////////////////////////////////////////////////////
  template<int Index>
  indexed_domain<T,1> get_domain(void) const
  {
    static_assert(Index == 0, "Attempted to get a 1D domain over a higher "
                              "dimension of a 1D indexed_domain.");
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns least index of domain
  //////////////////////////////////////////////////////////////////////
  index_type first(void) const
  {
    return m_first;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns greatest index of domain
  //////////////////////////////////////////////////////////////////////
  index_type last(void) const
  {
    return m_last;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns termination value for comparison
  //////////////////////////////////////////////////////////////////////
  index_type open_last(void) const
  {
    if (m_first == index_bounds<index_type>::invalid() &&
        m_last == index_bounds<index_type>::invalid())
      return m_last;

    // else
    return m_last + 1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns True if specified index is in domain
  //////////////////////////////////////////////////////////////////////
  bool contains(index_type const& index) const
  {
    return (m_first <= index) && (index <= m_last);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of indices in this domain
  //////////////////////////////////////////////////////////////////////
  size_type size(void) const
  {
    return (empty() ? 0 : (m_last - m_first + 1));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c True if this domain has no indices
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return ((m_first == index_bounds<index_type>::invalid()) &&
            (m_last == index_bounds<index_type>::invalid()));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c True if this domain has all the indexes to
  ///        reference all the data in a container.
  //////////////////////////////////////////////////////////////////////
  bool is_same_container_domain(void) const
  {
    return m_full_container;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "INDEXED_DOMAIN %p: " << this;
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << "\n";
    std::cerr << " first " << first();
    std::cerr << " last " << last();
    std::cerr << " size " << size() << "\n";
  }

  //////////////////////////////////////////////////////////////////////
  //  unique functionality for this domain
  //////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index resulting of advance the index @p i, @p
  ///        n positions.
  ///
  /// @tparam D Integral type.
  //////////////////////////////////////////////////////////////////////
  template<typename Distance>
  index_type advance(index_type i, Distance n) const
  {
    return (i + n);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns how many positions are needed to advance from the
  ///        index @p i to reach the index @p j.
  /// @note The returned value does not define in which direction to
  ///       advance.
  //////////////////////////////////////////////////////////////////////
  size_type distance(index_type i, index_type j) const
  {
    return (j > i) ? (j - i) : (i - j);
  }

  bool less_than(index_type const& i0, index_type const& i1) const
  {
    return i0 < i1;
  }

  void define_type(typer& t)
  {
    t.member(m_first);
    t.member(m_last);
    t.member(m_full_container);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c True if all the indexes in the domain are
  ///        contiguous. returns @c False otherwise.
  //////////////////////////////////////////////////////////////////////
  bool is_contiguous(void) const
  {
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the resulting intersection with the given @p other
  ///        domain.
  //////////////////////////////////////////////////////////////////////
  indexed_domain operator&(indexed_domain const& other) const
  {
    if (!m_full_container)
    {
      index_type olower = other.first();
      index_type oupper = other.last();

      if (!empty())
      {
        olower = other.contains(m_first) ? m_first : olower;
        oupper = other.contains(m_last)  ? m_last  : oupper;
      }

      if (!(other.contains(m_first) || other.contains(m_last)) )
        if (!(contains(olower) || contains(oupper)))
          return indexed_domain();

      // else
      return indexed_domain(olower, oupper);
    }
    else
    {
      // ensure the other domain is valid prior to returning.
      index_type first = other.first();
      index_type last = other.last();
      if (contains(first) && contains(last))
        return indexed_domain(other.first(), other.last());
      else if (contains(first))
        return indexed_domain(first, m_last);
      else if (contains(last))
        return indexed_domain(m_first, last);
      else
        return indexed_domain(index_bounds<index_type>::invalid(),
                              index_bounds<index_type>::invalid());
    }
  }

  size_type dimensions(void) const
  {
    return m_last - m_first + 1;
  }
}; // class indexed_domain<T, 1>


template<typename T>
std::ostream& operator<<(std::ostream& os, indexed_domain<T> const& d)
{
  if (d.empty())
    return os << "empty";
  return os << "[" << d.first() << ".." << d.last() << "]";
}

} // namespace stapl

#endif // STAPL_DOMAINS_INDEXED_DOMAIN_HPP
