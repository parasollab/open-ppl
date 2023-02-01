/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_EXTRACTION_GENERIC_HPP
#define STAPL_VIEWS_METADATA_EXTRACTION_GENERIC_HPP

#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/runtime/executor/anonymous_executor.hpp>
#include <stapl/views/metadata/container_fwd.hpp>
#include <stapl/domains/extract_domain.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor with partial specialization to get an rmi_handle for
///  containers that aren't p_objects.
//////////////////////////////////////////////////////////////////////
template<typename C, bool = is_p_object<C>::value>
struct determine_rmi_handle
{
  static rmi_handle::reference apply(C*)
  {
    return get_anonymous_executor().get_rmi_handle();
  }
};


template<typename C>
struct determine_rmi_handle<C, true>
{
  static rmi_handle::reference apply(C* c)
  {
    return c->get_rmi_handle();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor with partial specialization to get num_locations for
///  containers that aren't p_objects.
//////////////////////////////////////////////////////////////////////
template<typename C, bool = is_p_object<C>::value>
struct determine_num_locations
{
  static size_t apply(C*)
  {
    return get_num_locations();
  }
};


template<typename C>
struct determine_num_locations<C, true>
{
  static size_t apply(C* c)
  {
    return c->get_num_locations();
  }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to return the locality metadata information of
///        a general container (container that does not provide a
///        metadata locality extractor functor).
///
/// @tparam C The container for which metadata is to be extracted
///
/// @todo This catch all method for coarsening ignores the locality of
///  the underlying container and just creates a balanced partition of the
///  elements across the locations of the PARAGRAPH.  Containers should
///  explicitly request this behavior instead of a fallback. Otherwise we
///  silently perform badly.
//////////////////////////////////////////////////////////////////////
template<typename C>
class generic_metadata_extraction
{
  typedef detail::extract_domain<C>                            get_dom_t;
  typedef typename get_dom_t::result_type                      domain_type;
  typedef typename domain_type::index_type                     index_type;
  typedef C                                                    component_type;

public:
  typedef metadata_entry<domain_type, component_type*, size_t> value_type;
  typedef metadata::flat_container<value_type>                 md_cont_type;
  typedef std::pair<bool, md_cont_type*>                       return_type;

  return_type operator()(C* c) const
  {
    const std::size_t n = detail::determine_num_locations<C>::apply(c);

    balanced_partition<domain_type> npart(get_dom_t()(c), n);

    const std::size_t num_part = npart.size();

    md_cont_type* out_part = new md_cont_type(num_part);

    if (num_part > 0)
    {
      // Compare location id with number of partitions,
      // (num_part < n) is possible
      if (out_part->get_location_id() < num_part)
      {
        const std::size_t id = out_part->get_location_id();
        (*out_part)[id] =
          value_type(
            id, npart[id], c,
            LQ_CERTAIN, get_affinity(),
            detail::determine_rmi_handle<C>::apply(c),
            id
          );
      }
    }

    out_part->update();

    return std::make_pair(false, out_part);
  }
};

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_EXTRACTION_GENERIC_HPP
