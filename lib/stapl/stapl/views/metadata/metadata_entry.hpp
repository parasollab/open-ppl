/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_METADATA_ENTRY_HPP
#define STAPL_VIEWS_METADATA_METADATA_ENTRY_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/core_view.hpp>
#include <stapl/views/proxy_macros.hpp>
#include <stapl/utility/tuple/print.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines the metadata entry that is used as value_type of a
///        coarsen metadata container (e.g. metadata::growable_container).
///
/// The stored information about a domain, which base container
/// contains the elements referenced by the domain and the location
/// where the base container is stored. In cases where the given
/// domain references elements that are in more than one base container,
/// the referenced base container is considered invalid (NULL).
/// @todo Track down uses of default ctor and remove.  It's error prone
///   to default construct and then manually initialize.
//////////////////////////////////////////////////////////////////////
template<typename Domain, typename Container,
         typename CID =
           typename std::remove_pointer<Container>::type::cid_type>
struct metadata_entry
{
private:
  typedef std::pair<stapl::location_type, stapl::loc_qual>  pl_pair_t;
  typedef typename std::remove_pointer<Container>::type     container_t;

  CID                                  m_id;
  Domain                               m_dom;
  Container                            m_comp;

  loc_qual                             m_qualifier;
  affinity_tag                         m_affinity;
  rmi_handle::reference                m_handle;
  location_type                        m_location;

  bool                                 m_info_set;

public:
  /// @brief used to delay template instantiation of prefix_scan
  /// in @ref metadata::growable_container.
  typedef size_t                      delay_type;
  typedef typename Domain::gid_type   index_type;
  typedef Domain                      domain_type;
  typedef Container                   component_type;
  typedef CID                         cid_type;

  metadata_entry()
    : m_id(),
      m_comp(NULL),
      m_affinity(invalid_affinity_tag),
      m_location(invalid_location_id),
      m_info_set(false)
  { }

  metadata_entry(cid_type id, domain_type const& dom, component_type sc,
              loc_qual qualifier, affinity_tag affinity,
              rmi_handle::reference const& handle, location_type location)
    : m_id(id), m_dom(dom), m_comp(sc), m_qualifier(qualifier),
      m_affinity(affinity), m_handle(handle), m_location(location),
      m_info_set(true)
  {
    stapl_assert(handle != rmi_handle::reference(), "invalid handle");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a copy of the domain that this metadata entry represents.
  ///
  /// @todo This should possibly return the domain by reference. The proxy
  ///       then needs to be updated to return a proxy with a member_accessor.
  //////////////////////////////////////////////////////////////////////
  Domain domain() const
  {
    return m_dom;
  }

  Container const& component() const
  {
    return m_comp;
  }

  rmi_handle::reference handle() const
  {
    return m_handle;
  }

  location_type location() const
  {
    return m_location;
  }

  affinity_tag affinity() const
  {
    return m_affinity;
  }

  void set_id(CID id)
  {
    m_id = id;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief The component ids passed do not match the component id specified
  ///        at the instantiation in some cases.  This method works around that
  ///        temporarily and will be removed.
  ///
  /// @todo Remove this method.
  //////////////////////////////////////////////////////////////////////
  template <typename OtherCid>
  void set_id(OtherCid id)
  { }

  CID id() const
  {
    return m_id;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the qualifier associated with the stored location.
  ///
  /// @todo Verify if this method is required or the information can
  /// be obtained from the callee in a different way.
  //////////////////////////////////////////////////////////////////////
  loc_qual location_qualifier() const
  {
    stapl_assert(m_info_set, "info not set");

    return m_qualifier;
  }

  size_t size() const
  {
    return m_dom.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns True if the stored information is valid, because
  ///        it was created by given properly values.
  //////////////////////////////////////////////////////////////////////
  bool is_info_set() const
  {
    return m_info_set;
  }

  void set_domain(Domain const& dom)
  {
    m_dom = dom;
  }

  void define_type(typer &t)
  {
    t.member(m_id);
    t.member(m_dom);

    // NOTE: The value associated with m_comp needs to be packed as is
    // (without following the pointer). The validity of m_comp is defined
    // by the associated affinity tag.

    t.member(m_qualifier);
    t.member(m_affinity);
    t.member(m_handle);
    t.member(m_location);
    t.member(m_info_set);
  }

  friend
  std::ostream& operator<<(std::ostream& os,
                           metadata_entry<Domain, Container, CID> const& entry)
  {
    os << " domain "      << entry.m_dom
       << " size = "      << entry.m_dom.size()
       << ", location = " << entry.m_location
       << ", comp = "     << entry.m_comp
       << ", affinity = " << entry.m_affinity
       << ", id = ";
    stapl::print_tuple(os, entry.m_id);

    return os;
  }
}; // struct metadata_entry


STAPL_PROXY_HEADER_TEMPLATE(metadata_entry, Domain, Container, CID)
{
  STAPL_PROXY_TYPES(STAPL_PROXY_CONCAT(metadata_entry<Domain, Container, CID>), Accessor)
  STAPL_PROXY_IMPORT_TYPES(domain_type)
  STAPL_PROXY_IMPORT_TYPES(component_type)
  STAPL_PROXY_IMPORT_TYPES(cid_type)
  STAPL_PROXY_METHODS(STAPL_PROXY_CONCAT(metadata_entry<Domain, Container, CID>), Accessor)
  STAPL_PROXY_METHOD_RETURN(domain, domain_type)
  STAPL_PROXY_METHOD_RETURN(id, cid_type)
  STAPL_PROXY_METHOD_RETURN(location, location_type)
  STAPL_PROXY_METHOD_RETURN(affinity, affinity_tag)
  STAPL_PROXY_REFERENCE_METHOD_0(component, component_type const)
  STAPL_PROXY_METHOD_RETURN(handle, rmi_handle::reference)
  STAPL_PROXY_METHOD_RETURN(location_qualifier, loc_qual)

  void set_id(cid_type const& a0) const
  {
    typedef void (target_t::* fn_t)(cid_type);
    fn_t fn = &target_t::set_id;

    Accessor::invoke(fn, a0);
  }

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }
};

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_METADATA_ENTRY_HPP
