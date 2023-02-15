/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_TEST_VIEWS_METADATA_UTILS
#define STAPL_TEST_VIEWS_METADATA_UTILS

#include <stapl/views/metadata/coarseners/multiview.hpp>

namespace stapl {

template<typename... V>
auto coarsen_views(V... v)
  -> decltype(stapl::default_coarsener()(std::make_tuple(v...)))
{
  return stapl::default_coarsener()(std::make_tuple(v...));
}

template<typename View, typename MD>
bool coarsening_covers_space(View const& v, MD const& md)
{
  auto md_container_dom = md.domain();

  using index_type = decltype(md_container_dom.first());

  std::size_t elems_in_all_domains = 0;

  domain_map(md_container_dom, [&](index_type const& idx) {
    elems_in_all_domains += md[idx].domain().size();
  });

  // all locations have sum
  stapl::rmi_fence();

  return elems_in_all_domains == v.size();
}

template<typename V>
typename metadata::extract_metadata<
  typename std::decay<V>::type
>::return_type
extract_metadata_from_view(V&& v)
{
  typedef metadata::extract_metadata<
    typename std::decay<V>::type
  > extractor_t;

  return extractor_t()(&std::forward<V>(v));
}

} // namespace stapl

#endif
