/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_PARTITIONS_NDIM_PARTITION_FIND_HPP
#define STAPL_CONTAINERS_PARTITIONS_NDIM_PARTITION_FIND_HPP

#include <vector>
#include <utility>
#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>

namespace stapl {
namespace details {

template <std::size_t N,
          typename Dom, typename Part, typename ODom, typename MFG,
          std::size_t I,
          typename... Iters>
std::vector<std::pair<Dom, bool>>
invoke_find_in_part(std::vector<std::pair<Dom,bool>>& result,
                    Part const& part, ODom const& dom,
                    std::integral_constant<std::size_t, I>&&,
                    Iters&&... iters)
{
  for (auto&& iter : get<N - I>(part).find(get<N - I>(dom)))
  {
    invoke_find_in_part(result, part, dom,
                        std::integral_constant<std::size_t, I-1>(),
                        iters..., iter);
  }
}

template <std::size_t N,
          typename Dom, typename Part, typename ODom, typename MFG,
          std::size_t I,
          typename... Iters>
std::vector<std::pair<Dom, bool>>
invoke_find_in_part(std::vector<std::pair<Dom,bool>>& result,
                    Part const& part, ODom const& dom,
                    std::integral_constant<std::size_t, 0>&&,
                    Iters&&... iters)
{
  std::initializer_list<bool> cs = {iters.second...};

  bool is_contained =
    std::all_of(cs.begin(), cs.end(), [](bool b) { return b;});

  result.push_back(stapl::make_tuple(iters.first...), is_contained);
}


//////////////////////////////////////////////////////////////////////
/// @brief Determine which partition has the elements referenced
///        for the given domain.
///
/// The returned information is a collection (possibly empty) of
/// pairs. Each pair contains information about which partitions are
/// included in the given domain and how they are included (True: if
/// is fully contained, False: if is partially included).The
/// returned collection only has elements if there is at least one
/// partition that contains elements on the given domain.
///
/// @param dom Domain to compare
/// @param mfg Mapping function generator used to get the associated
///            mapping function to each partition. The generated
///            mapping function is used to project generated
///            partitioned domains into the given domain.
/// @return a vector of pairs.
//////////////////////////////////////////////////////////////////////
template <typename Dom, typename Part, typename ODom, typename MFG>
std::vector<std::pair<Dom,bool> >
find_in_part(Part const& part, ODom const& dom, MFG const& mfg)
{
  std::vector<std::pair<Dom, bool>> result;
  invoke_find_in_part<Part::num_partitions>(
    result, part, dom,
    std::integral_constant<std::size_t, Part::num_partitions>());

  return result;
}

} // namespace details
} // namespace stapl


#endif // STAPL_CONTAINERS_PARTITIONS_NDIM_PARTITION_FIND_HPP
