/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_ALLTOALL_HELPERS_HPP
#define STAPL_SKELETONS_ALLTOALL_HELPERS_HPP

#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/utility/tuple/tuple.hpp>
#include <vector>
#include <iterator>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

template <typename T, typename Tag>
class alltoall_pd;

//////////////////////////////////////////////////////////////////////
/// @brief The workfunction used in flat and hybrid alltoall for merging
/// the incoming messages.
///
/// Alltoall in STAPL can be done in at least three ways (1)
/// butterfly-based (@ref alltoall_butterfly_merge), (2) flat in which
/// all participants communicate with each other in only one level,
/// (3) hybrid which combines the ideas of butterfly and flat.
///
/// In the latter two cases, the merging of the incoming messages happens
/// using the following workfunction. The corresponding filter for this
/// workfunction is @ref alltoall_filter.
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename T, typename Tag>
struct alltoall_merge
{
private:
  typedef std::vector<stapl::tuple<std::size_t>> indices_t;
  indices_t m_indices;
public:
  typedef std::vector<T> result_type;

  alltoall_merge(indices_t const& indices)
    : m_indices(indices)
  { }

  template <typename V1, typename V2>
  result_type operator()(V1&& v1, V2&& v2) const
  {
    result_type res(v1);

    for (std::size_t i = 0; i < v2.size(); ++i) {
      res[stapl::get<0>(m_indices[i])] = v2[i][0];
    }
    return res;
  }

  void define_type(typer& t)
  {
    t.member(m_indices);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief The filtering used in hybrid, flat, and pairwise exchange
/// @c alltoalls for sending only the requested part to other participants.
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename T, typename Tag>
struct alltoall_filter
{
private:
  std::size_t m_pair_index;

public:
  typedef std::vector<T> result_type;

  alltoall_filter(std::size_t pair_index)
    : m_pair_index(pair_index)
  { }

  template <typename V>
  result_type operator()(V const& v) const
  {
    return result_type(1, v[m_pair_index]);
  }

  bool operator==(alltoall_filter const& other) const
  {
    return m_pair_index == other.m_pair_index;
  }

  void define_type(typer& t)
  {
    t.member(m_pair_index);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A specific filtering used in recursive-doubling (butterfly)
/// implementation of all_to_all.
///
/// In each step, the filter passes through every other @c butterfly_size
/// elements. The starting offset of the filtering depends on whether
/// the edge is an up-edge or down-edge in the butterfly skeleton.
///
/// @tparam T type of each element in the given input.
///
/// @note This implementation can only be used for power-of-two sizes
/// for other cases, one should use a ring-based, pair-wise, or any
/// other alltoall implementations that can handle other cases.
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename T, bool B>
struct alltoall_filter<T, tags::butterfly<B>>
{
  typedef std::vector<T> result_type;

  bool m_is_downedge;
  std::size_t m_hops;

  alltoall_filter()
    : m_is_downedge(false),
      m_hops(0)
  { }

  void set_position(std::size_t butterfly_size,
                    std::size_t cur_index, std::size_t bflied_index,
                    std::size_t /* ignored */)
  {
    m_hops = butterfly_size;
    m_is_downedge = cur_index >  bflied_index;
  }

  template <typename V1>
  result_type operator()(V1&& vec1) const
  {
    result_type result;
    result.reserve(vec1.size()/2);

    auto&& it = vec1.begin();
    auto&& it_end = vec1.end();

    if (m_is_downedge) {
      std::advance(it, m_hops);
    }

    for (; it < it_end;)
    {
      auto end_chunk_it = it;
      std::advance(end_chunk_it, m_hops);
      result.insert(result.end(), it, end_chunk_it);
      std::advance(it, 2*m_hops);
    }
    return result;
  }

  bool operator==(alltoall_filter const& other) const
  {
    return m_is_downedge == other.m_is_downedge &&
           m_hops == other.m_hops;
  }

  void define_type(typer& t)
  {
    t.member(m_is_downedge);
    t.member(m_hops);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Merges the results from two given inputs by putting one
/// chunk of size @c butterfly_size from the first and
/// then the same size from the second until both inputs are exhausted.
///
/// @tparam T type of each element in the given inputs.
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename T, bool B>
class alltoall_merge<T, tags::butterfly<B>>
{
  bool m_is_downedge;
  std::size_t m_hops;
public:
  typedef std::vector<T> result_type;

  alltoall_merge()
    : m_is_downedge(false),
      m_hops(0)
  { }

  void set_position(std::size_t butterfly_size,
                    std::size_t cur_index, std::size_t bflied_index,
                    std::size_t /* ignored */)
  {
    m_hops = butterfly_size;
    m_is_downedge = cur_index >  bflied_index;
  }

  template <typename V1, typename V2>
  result_type insert_values(V1&& v1, V2&& v2) const
  {
    result_type result;
    result.reserve(v1.size()*2);

    auto&& it1 = v1.begin();
    auto&& it2 = v2.begin();

    for (; it1 < v1.end() && it2 < v2.end();)
    {
      auto end_chunk_it1 = it1;
      auto end_chunk_it2 = it2;

      std::advance(end_chunk_it1, m_hops);
      std::advance(end_chunk_it2, m_hops);

      result.insert(result.end(), it1, end_chunk_it1);
      result.insert(result.end(), it2, end_chunk_it2);

      std::advance(it1, m_hops);
      std::advance(it2, m_hops);
      }

    return result;
  }

  template <typename V1, typename V2>
  result_type operator()(V1&& v1, V2&& v2) const
  {
    return m_is_downedge ?
      insert_values(v2, v1):
      insert_values(v1, v2);
  }

  void define_type(typer& t)
  {
    t.member(m_is_downedge);
    t.member(m_hops);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief The workfunction used in pairwise exchange alltoall for merging
/// the incoming messages.
///
/// @ingroup skeletonsParamDepsInternal
//////////////////////////////////////////////////////////////////////
template <typename T>
struct alltoall_merge<T, tags::pairwise_exchange>
{
private:
  std::size_t m_pair_index;
public:
  typedef std::vector<T> result_type;

  alltoall_merge(std::size_t pair_index)
    : m_pair_index(pair_index)
  { }

  template <typename V1, typename V2>
  result_type operator()(V1&& v1, V2&& v2) const
  {
    result_type res(v1);
    res[m_pair_index] = v2[0];
    return res;
  }

  void define_type(typer& t)
  {
    t.member(m_pair_index);
  }
};

} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates the simplest alltoall parametric dependency skeleton.
///
/// @copybrief skeletons_impl::alltoall_hybrid_pd
///
/// @ingroup skeletonsParamDepsExchange
//////////////////////////////////////////////////////////////////////
template <typename T, typename Tag>
skeletons_impl::alltoall_pd<T, Tag>
alltoall_pd()
{
  return skeletons_impl::alltoall_pd<T, Tag>();
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_ALLTOALL_HELPERS_HPP
