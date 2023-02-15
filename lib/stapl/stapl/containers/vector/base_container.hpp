/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_VECTOR_BASE_CONTAINER_HPP
#define STAPL_CONTAINERS_VECTOR_BASE_CONTAINER_HPP

#include <stapl/domains/indexed.hpp>

#include <stapl/containers/base/bc_base.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>
#include <stapl/containers/distribution/base_container_metadata.hpp>
#include <stapl/containers/type_traits/define_value_type.hpp>
#include "traits/base_container_traits.hpp"

namespace stapl {

template<typename Traits>
struct vector_base_container;


template<typename Traits>
struct container_traits<vector_base_container<Traits> >
{
  typedef typename Traits::value_type                          value_type;
  typedef typename Traits::domain_type                         domain_type;
  typedef typename domain_type::index_type                     gid_type;
  typedef typename Traits::container_type                      container_type;
  typedef local_accessor<vector_base_container<Traits> >       accessor_t;
  typedef proxy<value_type, accessor_t>                        reference;
  typedef const_local_accessor<vector_base_container<Traits> > const_accessor_t;
  typedef proxy<value_type, const_accessor_t>                  const_reference;
};


//////////////////////////////////////////////////////////////////////
/// @brief The base container used for @ref vector.
/// @ingroup pvectorDist
///
/// @see vector
/// @tparam Traits The collection of traits types that defines customizable
///         components used by the base container.
////////////////////////////////////////////////////////////////////////
template<typename Traits>
struct vector_base_container
  : public bc_base
{
  typedef typename Traits::value_type                     value_type;
  typedef typename Traits::domain_type                    domain_type;
  typedef size_t                                          cid_type;
  typedef typename domain_type::index_type                gid_type;
  typedef typename domain_type::size_type                 size_type;

  // backend container
  typedef typename Traits::container_type                 container_type;
  typedef typename Traits::container_constructor          container_constructor;

  typedef local_accessor<vector_base_container>           accessor_t;
  typedef proxy<value_type, accessor_t>                   reference;
  typedef const_local_accessor<vector_base_container>     const_accessor_t;
  typedef proxy<value_type, const_accessor_t>             const_reference;

  typedef local_iterator<vector_base_container>           iterator;
  typedef const_local_iterator<vector_base_container>     const_iterator;

  typedef base_container_metadata<vector_base_container>  loc_dist_metadata;

protected:
  // ========================
  // Data
  // ========================
  /// The domain of this component.
  domain_type            m_domain;
  /// The underlying raw data.
  container_type         m_data;
  /// This component's id.
  cid_type               m_cid;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the local position of a global id.
  /// @param gid The GID to find the local position of.
  //////////////////////////////////////////////////////////////////////
  gid_type local_position(gid_type const& gid) const
  {
    return m_domain.empty() ? 0 : gid - m_domain.first();
  }

public:
  /// @name Constructors
  /// @{

  vector_base_container(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a base container with a specific ID.
  /// @param cid The id for this base container.
  //////////////////////////////////////////////////////////////////////
  vector_base_container(cid_type const& cid)
    : m_cid(cid)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a base container with a specific domain.
  /// @param domain The explicit domain for this component.
  /// @param cid The id for this component.
  //////////////////////////////////////////////////////////////////////
  vector_base_container(domain_type const& domain, cid_type const& cid)
    : m_domain(domain), m_data(m_domain.size()), m_cid(cid)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a base container with a specific domain, ID and initial
  /// value.
  /// @param domain The explicit domain for this base container.
  /// @param cid The id for this base container.
  /// @param default_value The default value for all elements in the base
  ///        container.
  //////////////////////////////////////////////////////////////////////
  vector_base_container(domain_type const& domain,
                        cid_type const& cid,
                        value_type const& default_value)
    : m_domain(domain),
      m_data(m_domain.size(), typename Traits::stored_type(default_value)),
      m_cid(cid)
  { }

  vector_base_container(vector_base_container const& other)
    : m_domain(other.m_domain), m_data(other.m_data), m_cid(other.m_cid)
  { }

  vector_base_container& operator=(vector_base_container const& other)
  {
    m_domain = other.m_domain;
    m_data   = other.m_data;
    m_cid    = other.m_cid;
    return *this;
   }

  /// @}

  ~vector_base_container(void)
  {
    bc_base_impl::cleanup_elements<value_type>::apply(m_data,
                                                      this->m_defer_cleanup);
  }

  /// @name Element Manipulation
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a reference to an element of the base container
  ///        using its GID.
  /// @param gid The GID for which to create a reference.
  /// @return A proxy of the element at gid.
  //////////////////////////////////////////////////////////////////////
  reference make_reference(gid_type const& gid)
  {
    return reference(accessor_t(this, m_data.begin()+local_position(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the iterator of a GID.
  /// @param gid The GID for which to create the iterator.
  /// @return An iterator to the element at gid.
  //////////////////////////////////////////////////////////////////////
  iterator make_iterator(gid_type const& gid)
  {
    return iterator(m_data.begin()+local_position(gid),this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct a local reference to a specific index of the base
  /// container.
  /// @param gid The GID for which to create the reference
  /// @return A proxy of the value at gid
  ////////////////////////////////////////////////////////////////////////
  reference operator[](gid_type const& gid)
  {
    return make_reference(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the element corresponding to the specified gid.
  /// @param gid The id associated with the element for
  ///        which we want to read the value.
  /// @return A reference to the element.
  //////////////////////////////////////////////////////////////////////
  value_type get_element(gid_type const& gid)
  {
    return m_data[local_position(gid)];
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the element corresponding to a specific GID.
  ///
  /// This method is used in the redistribution of composed containers.
  /// It is needed to allow the distributor object to get the instance of
  /// the @ref container_wrapper_ref for a container instance on one location
  /// and send it to another location where it will be placed in a base
  /// container by calling set_element. @ref set_element only accepts
  /// instances of the stored type.
  ///
  /// @param gid The id associated with the element for which we want to read
  /// the value.
  ///
  /// @return A copy of the stored instance of the element.
  //////////////////////////////////////////////////////////////////////
  typename Traits::stored_type get_stored_element(gid_type const& gid) const
  {
    return m_data[local_position(gid)];
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke the functor provided on the element with the specified key.
  /// @param gid The GID of the element to be processed.
  /// @param f The functor to apply
  //////////////////////////////////////////////////////////////////////
  template <typename F>
  void apply_set(gid_type const& gid, F const& f)
  {
    f(m_data[local_position(gid)]);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply a function object to an element at a specific GID and
  ///   return the result.
  /// @param gid The GID associated with the element for which we want to apply
  ///   the functor and read the result.
  /// @param f The functor to apply to gid
  /// @return The result of applying f to the value at gid
  //////////////////////////////////////////////////////////////////////
  template <typename Functor>
  typename Functor::result_type
  apply_get(gid_type const& gid, Functor const& f)
  {
    return f(m_data[local_position(gid)]);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set an element in the base container.
  /// @param gid The gid of the element which should be set.
  /// @param t Value to place in the base container.
  //////////////////////////////////////////////////////////////////////
  void set_element(gid_type const& gid, typename Traits::stored_type const& t)
  {
    m_data[local_position(gid)]  = t;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update the values starting at @p gid using elements in the
  /// given iterator range.
  //////////////////////////////////////////////////////////////////////
  template<typename Iterator>
  typename std::enable_if<is_local_iterator<Iterator>::value>::type
  set_elements(gid_type const& gid, Iterator iter, Iterator e_iter)
  {
    stapl_assert(m_domain.contains(gid), "gid not in container domain");

    std::copy(iter.base_iterator(),
              e_iter.base_iterator(),
              m_data.begin() + local_position(gid));
  }

  template<typename Iterator>
  typename std::enable_if<!is_local_iterator<Iterator>::value>::type
  set_elements(gid_type const& gid, Iterator iter, Iterator e_iter)
  {
    stapl_assert(m_domain.contains(gid), "gid not in container domain");

    std::copy(iter, e_iter, m_data.begin() + local_position(gid));
  }

  //////////////////////////////////////////////////////////////////////
  // @brief Update the values starting at @p gid using elements in the view.
  //////////////////////////////////////////////////////////////////////
  template<typename View>
  void set_elements(gid_type const& gid, View&& view)
  {
    set_elements(gid, view.begin(), view.end());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add @p t at the end of the base container.
  /// @param t value to add.
  /// @return Operation successful or not.
  //////////////////////////////////////////////////////////////////////
  bool push_back(value_type const& t)
  {
    if (m_domain.empty())
      this->m_domain = domain_type(0,0);
    else
      this->m_domain = domain_type(m_domain.first(), m_domain.last()+1);

    m_data.push_back(t);

    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add @p t before @p gid in the base container.
  /// @param t value to add.
  /// @param gid The gid to insert @p t after.
  //////////////////////////////////////////////////////////////////////
  void insert(gid_type const& gid, value_type const& t)
  {
    if (m_domain.empty())
      this->m_domain = domain_type(gid, gid);
    else
      this->m_domain = domain_type(m_domain.first(),m_domain.last()+1);

    typename container_type::iterator it = m_data.begin();
    std::advance(it,local_position(gid));
    m_data.insert(it,t);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erase an element of the base container.
  /// @param gid gid referencing the element to remove.
  //////////////////////////////////////////////////////////////////////
  void erase(gid_type const& gid)
  {
    stapl_assert(!m_domain.empty(),"base container is empty");
    this->m_domain = domain_type(m_domain.first(), m_domain.last()-1);

    typename container_type::iterator it = m_data.begin();
    std::advance(it, local_position(gid));
    m_data.erase(it);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove all elements from the base container.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    // construct an empty domain.
    this->m_domain = domain_type();
    m_data.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Remove the last element of the base container and return
  ///        its gid.
  /// @return Returns the gid of the last element just removed.
  //////////////////////////////////////////////////////////////////////
  gid_type pop_back(void)
  {
    const gid_type gid = m_domain.last();

    m_data.pop_back();
    this->m_domain = domain_type(m_domain.first(), m_domain.last() - 1);

    return gid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return a local iterator to the first element
  ///        of the base container.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return iterator(m_data.begin(), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return a const local iterator to the first element
  ///        of the base container.
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return const_iterator(m_data.begin(), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return a local iterator to one position past the
  ///        last element of the base container.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return iterator(m_data.end(), this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct and return a const local iterator to one
  ///        position past the last element of the base container.
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return const_iterator(m_data.end(),this);
  }
  /// @}

  /// @name Memory and Domain Management
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the data of the base container.
  //////////////////////////////////////////////////////////////////////
  container_type& container(void)
  {
    return m_data;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements in this component.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return this->m_domain.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the cid of this component.
  //////////////////////////////////////////////////////////////////////
  cid_type const& cid(void) const
  {
    return m_cid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the base container domain.
  //////////////////////////////////////////////////////////////////////
  domain_type domain(void) const
  {
    return m_domain;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the base container domain.
  //////////////////////////////////////////////////////////////////////
  void set_domain(domain_type const& dom)
  {
    m_domain = dom;
  }
  /// @}
}; // class vector_base_container

} // namespace stapl

#endif // STAPL_CONTAINERS_VECTOR_BASE_CONTAINER_HPP
