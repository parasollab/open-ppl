/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SPANS_SUMMA_HPP
#define STAPL_SKELETONS_SPANS_SUMMA_HPP

#include <stapl/skeletons/spans/blocked.hpp>

namespace stapl {
namespace skeletons {
namespace spans {

template<int i>
class summa
  : public blocked<i>
{
public:
  using metadata_type = stapl::tuple<size_t, size_t, size_t>;

  using dims_num           = std::integral_constant<int, i>;
  using traversal_type     = typename default_traversal<i>::type;
  using partition_type     = multiarray_impl::block_partition<
                               typename default_traversal<i>::type>;
  using domain_type        = typename partition_type::domain_type;
  using index_type         = typename domain_type::index_type;
  using size_type          = typename domain_type::size_type;
  using dimension_type     = std::integral_constant<int, i>;
  using linearization_type = nd_linearize<index_type, traversal_type>;

private:
  using base_t = blocked<i>;

  metadata_type m_mx_sizes;

public:
  template <bool forced = false, typename Spawner, typename... Views>
  void set_size(Spawner const& spawner, Views const&... views)
  {
    auto view_domains =
      make_tuple(
        skeletons::domain_type<
          typename Views::domain_type, has_finite_domain<Views>::value
        >(views.domain())...);

    this->set_size_impl(spawner, get<2>(tuple<Views const&...>(views...)));

    // get m, k and p sizes of input views
    m_mx_sizes =
      make_tuple(stapl::get<0>((stapl::get<0>(view_domains)).dimensions()),
                 stapl::get<1>((stapl::get<0>(view_domains)).dimensions()),
                 stapl::get<1>((stapl::get<1>(view_domains)).dimensions()));
  }

  metadata_type dimensions_metadata() const
  {
    return m_mx_sizes;
  }

  void define_type(typer& t)
  {
    t.base<base_t>(*this);
    t.member(m_mx_sizes);
  }
};


} // namespace spans
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_SPANS_SUMMA_HPP
