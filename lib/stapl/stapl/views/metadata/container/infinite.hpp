/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_CONTAINER_INFINITE_HPP
#define STAPL_VIEWS_METADATA_CONTAINER_INFINITE_HPP

#include <stapl/runtime.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/views/metadata/metadata_traits.hpp>

#include <boost/iterator/transform_iterator.hpp>
#include <boost/optional.hpp>

namespace stapl {

namespace metadata {

namespace detail {

template<int N>
struct indexed_domain_type
{
    using type = indexed_domain<
      std::size_t, N, typename default_traversal<N>::type
    >;
};

template<>
struct indexed_domain_type<1>
{
    using type = indexed_domain<std::size_t, 1>;
};

//////////////////////////////////////////////////////////////////////
/// @brief Represents an infinite iterator
//////////////////////////////////////////////////////////////////////
template<class T>
class infinite_iter
  : public boost::forward_iterator_helper<
      infinite_iter<T>, T, std::ptrdiff_t, size_t, T>
{
private:
  boost::optional<T>      m_data;

public:
  typedef infinite_iter   self;
  typedef T               reference;
  typedef std::ptrdiff_t  distance;


  infinite_iter(T const& data)
    : m_data(data)
  { }

  self& operator=(self const& o)
  {
    m_data = o.m_data;

    return *this;
  }

  reference operator*() const
  {
    return *m_data;
  }

  self& operator++()
  {
    return *this;
  }

  bool operator==(self const& o) const
  {
    return m_data == o.m_data;
  }

  bool operator<(self const& o) const
  {
    return o.m_data;
  }

  friend distance operator-(self const& x, self const& y)
  {
    return distance(index_bounds<size_t>::highest());
  }
}; // class infinite_iter


//////////////////////////////////////////////////////////////////////
/// @brief Small static functor to create a valid component identifier
/// for @ref infinite_container to use when creating metadata
/// entries.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct generate_zero_cid
{
  static T apply(void)
  { return 0; };
};


template<typename... Args>
struct generate_zero_cid<tuple<Args...>>
{
  static tuple<Args...> apply(void)
  { return tuple<Args...>(Args(0)...); }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Metadata container used for views that represent an
///        infinite collection of elements (e.g. repeat_view,
///        output_view).
///
/// @tparam C Type of the infinite container.
/// @tparam MD Type of the metadata used to represent the metadata
///            information.
//////////////////////////////////////////////////////////////////////
template<typename C, typename MD>
class infinite_container
{
  typedef typename MD::domain_type                          domain_t;

  static constexpr int num_dimensions = dimension_traits<domain_t>::type::value;

  C* m_cont;

public:
  typedef typename detail::indexed_domain_type<
    num_dimensions>::type domain_type;
  typedef typename domain_type::dimensions_type             dimensions_type;
  typedef typename domain_type::index_type                  index_type;
  typedef MD                                                value_type;
  typedef value_type                                        reference;
  typedef detail::infinite_iter<MD>                         iterator;

private:
  typedef typename detail::generate_zero_cid<
    typename value_type::cid_type>                          cid_generator_t;

public:
  void define_type(typer& t)
  {
    t.member(m_cont);
  }

  infinite_container(C* cont)
    : m_cont(cont)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::begin
  /// The iteration space generated for this iterator is not bounded.
  //////////////////////////////////////////////////////////////////////
  iterator begin()
  {
    return iterator(value_type(
      cid_generator_t::apply(), domain_t(), m_cont, LQ_CERTAIN, get_affinity(),
      m_cont->distribution().get_rmi_handle(), get_location_id()
    ));
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::end
  //////////////////////////////////////////////////////////////////////
  iterator end()
  {
    return iterator();
  }

  reference operator[](index_type const&)
  {
    return value_type(
      cid_generator_t::apply(), domain_t(), m_cont, LQ_CERTAIN,
      get_affinity(), m_cont->distribution().get_rmi_handle(), get_location_id()
    );
  }

  domain_type domain() const
  {
    return domain_type(
      detail::generate_zero_cid<index_type>::apply(),
      index_bounds<index_type>::highest()
    );
  }

  size_t size() const
  {
    return domain_t().size();
  }

  dimensions_type dimensions() const
  {
    return index_bounds<index_type>::highest();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_local_vid
  //////////////////////////////////////////////////////////////////////
  index_type get_local_vid(index_type const& index)
  {
    return index;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::push_back_here(MD const&)
  /// This class does not support @c push_back_here.
  /// @todo Add a stapl_assert to ensure this method is not called.
  //////////////////////////////////////////////////////////////////////
  template <typename T>
  void push_back_here(T const&)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::local_size
  //////////////////////////////////////////////////////////////////////
  size_t local_size() const
  {
    return domain_t().size();
  }

  dimensions_type local_dimensions() const
  {
    return index_bounds<index_type>::highest();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_location_element(size_t)
  //////////////////////////////////////////////////////////////////////
  location_type get_location_element(index_type const&) const
  {
    return get_location_id();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc flat_container::update
  /// This class does not support @c push_back_here.
  /// @todo Add a stapl_assert to ensure this method is not called or
  ///       remove it.
  //////////////////////////////////////////////////////////////////////
  void update()
  { }
}; // class infinite_container

} // namespace metadata

template<typename C, typename MD>
struct metadata_traits<metadata::infinite_container<C, MD>>
{
  using is_isomorphic = std::true_type;
};

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_CONTAINER_INFINITE_HPP
