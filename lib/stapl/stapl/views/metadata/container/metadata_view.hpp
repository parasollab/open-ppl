/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_CONTAINER_METADATA_VIEW_HPP
#define STAPL_VIEWS_METADATA_CONTAINER_METADATA_VIEW_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/core_view.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/views/metadata/metadata_traits.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Metadata container base class for view
/// @tparam T Type of the metadata used as value_type in the coarsen
///           container information.
//////////////////////////////////////////////////////////////////////
template <typename T, typename Domain, bool Isomorphic>
struct view_base
{
  typedef T value_type;

  typedef typename std::conditional<
    Isomorphic,
    typename Domain::index_type,
    std::size_t
  >::type index_type;
  typedef index_type dimensions_type;

  virtual ~view_base() = default;

  virtual size_t local_size() const = 0;

  virtual size_t size() const = 0;

  virtual dimensions_type dimensions() const = 0;

  virtual dimensions_type local_dimensions() const = 0;

  virtual index_type get_local_vid(index_type const&) const = 0;

  virtual value_type operator[](index_type const&) const = 0;

  virtual location_type get_location_element(index_type const&) const = 0;

  virtual void push_back_here(value_type const&) const = 0;

  virtual void update(void) = 0;

  virtual void print(void) const
  {
    stapl_assert(0, "metadata::view_base::print shouldn't be called");
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Definition of a view over a metadata container.
/// @tparam C Container type.
/// @tparam Dom Domain type.
/// @tparam MapFunc Mapping function type.
//////////////////////////////////////////////////////////////////////
template <typename C, typename Dom, typename MapFunc>
class view
  : public view_base<
             typename C::value_type,
             Dom,
             metadata_traits<C>::is_isomorphic::value
           >,
    public core_view<C, Dom, MapFunc>
{
  typedef core_view<C, Dom, MapFunc>                      base_type;

public:
  typedef typename C::value_type                          value_type;
  typedef typename Dom::index_type                        index_type;
  typedef index_type                                      dimensions_type;

  view() = default;

  view(C* cont)
    : base_type(cont, Dom(cont->domain()))
  { }

  size_t local_size() const
  {
    return this->container().local_size();
  }

  size_t size() const
  {
    return this->get_container()->size();
  }

  dimensions_type local_dimensions() const
  {
    return this->get_container()->local_dimensions();
  }

  dimensions_type dimensions() const
  {
    return this->get_container()->dimensions();
  }

  index_type get_local_vid(index_type const& index) const
  {
    return this->container().get_local_vid(index);
  }

  value_type operator[](index_type const& index) const
  {
    return this->container()[index];
  }

  void set_element(index_type const& index, value_type const& value) const
  {
    this->container()[index] = value;
  }

  void push_back_here(value_type const& val) const
  {
    this->container().push_back_here(val);
  }

  location_type get_location_element(index_type const& index) const
  {
    return this->container().get_location_element(index);
  }

  void define_type(typer& t)
  {
    t.base<base_type>(this);
  }

  void update()
  {
    this->container().update();
  }

  void print(void) const
  { }
}; // class view


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class over a view used to erase
///        the type of the container metadata.
///
/// @tparam T Type of the metadata used as value_type in the coarsen
///           container information.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Domain, bool Isomorphic>
struct view_wrapper
  : public view_base<T, Domain, Isomorphic>
{
public:
  typedef view_base<T, Domain, Isomorphic>  base_ref;
  typedef T                                 value_type;
  typedef Domain                            domain_type;

  typedef typename std::conditional<
    Isomorphic,
    typename Domain::index_type,
    std::size_t
  >::type                                   index_type;
  typedef index_type                        dimensions_type;

  typedef size_t                            size_type;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Iterator over elements of the metadata container the
  /// @ref view_wrapper is abstracting.
  ///
  /// The iterator provides forward iteration and read-only access to the
  /// metadata as this is all that is currently required by view coarsening.
  //////////////////////////////////////////////////////////////////////
  struct index_iterator
  {
  protected:
    /// Index of the domain the iterator is referencing
    index_type                m_index;

    /// Reference to the metadata container the iterator is traversing
    std::shared_ptr<base_ref> m_ref;

    Domain                    m_domain;

  public:
    using reference         = T;
    using value_type        = T;
    using difference_type   = long int;
    using pointer           = index_iterator;
    using iterator_category = std::forward_iterator_tag;

    index_iterator(index_type const& index,
                   std::shared_ptr<base_ref> ref,
                   Domain const& domain)
      : m_index(index), m_ref(ref), m_domain(domain)
    { }

    T operator*() const
    {
      return (*m_ref)[m_index];
    }

    index_iterator& operator++()
    {
      // ++m_index;
      m_index = m_domain.advance(m_index, 1);
      return *this;
    }

    bool operator==(index_iterator const& other) const
    {
      return (m_index == other.m_index) && (&(*m_ref) == &(*other.m_ref));
    }

    bool operator!=(index_iterator const& other) const
    {
      return (m_index != other.m_index) || (&(*m_ref) != &(*other.m_ref));
    }

    bool operator<(index_iterator const& other) const
    {
      return m_index < other.m_index;
    }
  }; // struct index_iterator

protected:
  std::shared_ptr<base_ref> m_ref;
  Domain                    m_domain;

public:
  using iterator = index_iterator;

  void define_type(typer &t)
  {
    t.member(m_ref);
    t.member(m_domain);
  }

  view_wrapper(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a wrapper over the given view
  ///        @c CMDView.
  //////////////////////////////////////////////////////////////////////
  template<typename CMDView>
  view_wrapper(CMDView* part)
    : m_ref(part), m_domain(part->domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @todo Investigate whether m_domain should be set too (or just
  /// use default operator=).
  //////////////////////////////////////////////////////////////////////
  view_wrapper&
  operator=(view_wrapper const& other)
  {
    m_ref = other.m_ref;
    return *this;
  }

  view_wrapper(view_wrapper const& other)
    : m_ref(other.m_ref), m_domain(other.m_domain)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::local_size
  //////////////////////////////////////////////////////////////////////
  size_t local_size() const
  { return m_ref->local_size(); }

  size_t size() const
  { return m_ref->size(); }

  Domain domain() const
  { return m_domain; }

  dimensions_type dimensions() const
  {
    return this->m_ref->dimensions();
  }

  dimensions_type local_dimensions() const
  {
    return this->m_ref->local_dimensions();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_local_vid(size_t)
  //////////////////////////////////////////////////////////////////////
  index_type get_local_vid(index_type const& index) const
  { return m_ref->get_local_vid(index); }

  value_type operator[](index_type const& index) const
  { return (*m_ref)[index]; }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::begin
  //////////////////////////////////////////////////////////////////////
  iterator begin()
  { return iterator(m_domain.first(), m_ref, m_domain); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::end
  //////////////////////////////////////////////////////////////////////
  iterator end()
  { return iterator(m_domain.advance(m_domain.last(), 1), m_ref, m_domain); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_location_element(size_t)
  //////////////////////////////////////////////////////////////////////
  location_type get_location_element(index_type const& index) const
  {
    return m_ref->get_location_element(index);
  }

  void push_back_here(T const&) const
  {
    stapl::abort("cannot add metadata to read only metadata view wrapper");
  }

  void update()
  {
    stapl::abort("cannot add metadata to read only metadata view wrapper");
  }

  void print(void) const
  {
    m_ref->print();
  }
}; // struct view_wrapper

} // namespace metadata

template<typename T, typename Domain, bool Isomorphic>
struct metadata_traits<metadata::view_wrapper<T, Domain, Isomorphic> >
{
  using is_isomorphic = std::integral_constant<bool, Isomorphic>;
  using value_type = T;
};

template<typename C, typename Domain, typename MF>
struct metadata_traits<metadata::view<C, Domain, MF>>
{
  using is_isomorphic = typename metadata_traits<C>::is_isomorphic;
  using value_type = typename metadata_traits<C>::value_type;
};

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_CONTAINER_METADATA_VIEW_HPP
