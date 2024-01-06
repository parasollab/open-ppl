/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FLOWS_INDEXED_FLOW_HPP
#define STAPL_SKELETONS_FLOWS_INDEXED_FLOW_HPP

#include <vector>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/flows/producer_info.hpp>
#include <stapl/views/mapping_functions/linearization.hpp>
#include <stapl/domains/interval.hpp>
#include <stapl/domains/domain_interval.hpp>

namespace stapl {
namespace skeletons {
namespace flows {


inline std::size_t
linearize_wf(stapl::indexed_domain<std::size_t> const& domain,
             std::size_t index)
{
  return index;
}

inline std::size_t
linearize_wf(stapl::domset1D<std::size_t> const& domain,
             std::size_t index)
{
  return index;
}

template<typename Distribution>
inline std::size_t
linearize_wf(stapl::domainset1D<Distribution> const& domain,
             std::size_t index)
{
  return index;
}

template <int N, typename Traversal>
inline std::size_t linearize_wf(
  stapl::indexed_domain<std::size_t, N, Traversal> const& domain,
  typename stapl::indexed_domain<std::size_t, N, Traversal>::index_type const&
    index)
{
  using dom_t = stapl::indexed_domain<std::size_t, N, Traversal>;
  return stapl::nd_linearize<typename dom_t::index_type,
                             typename dom_t::traversal_type
                            >(domain.dimensions())(index);
}


template <typename T, typename Domain, bool inter_pg = false,
          typename QueryMapper = void>
class indexed_flow;

////////////////////////////////////////////////////////////////////////
/// @brief An elementary flow representing the output of a single node
/// in the dataflow graph corresponding to a single skeleton which allows
/// a skeleton to be connected to all its underlying spawned tasks.
///
/// @see elem.hpp
///
/// @ingroup skeletonsFlowsElem
////////////////////////////////////////////////////////////////////////
template <typename T, typename Domain, bool inter_pg>
class indexed_flow<T, Domain, inter_pg, void>
{
public:
  using flow_value_type = T;
  using domain_type     = typename std::decay<Domain>::type;

private:
  using index_type = std::size_t;

  std::size_t const m_initial_offset;
  domain_type       m_domain;

public:
  indexed_flow(Domain domain, std::size_t initial_offset)
    : m_initial_offset(initial_offset),
      m_domain(std::move(domain))
  { }

  template <typename F = skeletons::no_filter>
  using producer_type = indexed_producer<
                          flow_value_type, index_type, F, inter_pg>;

  template <typename Index,
            typename F = skeletons::no_filter,
            typename Mapper = skeletons::no_mapper>
  producer_type<F>
  consume_from(Index const& index,
               F const& f = F(),
               Mapper const& mapper = Mapper()) const
  {
    std::size_t offset = linearize_wf(m_domain, stapl::get<0>(index));
    return producer_type<F>(m_initial_offset + offset, f);
  }

  template <typename Index>
  index_type
  depend_on(Index const& index) const
  {
    return this->consume_from(index).get_index();
  }

  template <typename Indices,
            typename F = skeletons::no_filter,
            typename Mapper = skeletons::no_mapper>
  indexed_producers<flow_value_type, std::vector<index_type>, F>
  consume_from_many(Indices const& indices,
                    F const& f = F(),
                    Mapper const& mapper = Mapper()) const
  {
    using producers_type =
      indexed_producers<flow_value_type, std::vector<index_type>, F>;

    std::vector<index_type> deps;
    deps.reserve(indices.size());

    for (auto&& e : indices) {
      deps.push_back(this->consume_from(e).get_index());
    }

    return producers_type(deps, f);
  }

  domain_type domain(void) const
  {
    return m_domain;
  }
};

template <typename T,
          bool inter_pg = false,
          typename QueryMapper = void,
          typename Domain>
indexed_flow<T, Domain, inter_pg, QueryMapper> make_indexed_flow(
  Domain&& domain, std::size_t const offset)
{
  return indexed_flow<T, Domain, inter_pg, QueryMapper>(
    std::forward<Domain>(domain), offset);
}

} // namespace flows
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FLOWS_INDEXED_FLOW_HPP
