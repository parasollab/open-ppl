/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_EXTRACTION_GENERATOR_HPP
#define STAPL_VIEWS_METADATA_EXTRACTION_GENERATOR_HPP

#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/views/metadata/container/generator.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to return the locality metadata information of
///        a general container (container that does not provide a
///        metadata locality extractor functor).
//////////////////////////////////////////////////////////////////////
template<typename C>
class generator_extractor
{
  using domain_type    = typename container_traits<C>::domain_type;
  using component_type = C;
  using index_type     = typename domain_type::index_type;

  using value_type = metadata_entry<domain_type, component_type*, index_type>;

public:
  using return_type = std::pair<bool, generator_container<value_type>*>;

  return_type operator()(C* c) const
  {
    return std::make_pair(true, new generator_container<value_type>(c));
  }
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_EXTRACTION_GENERATOR_HPP
