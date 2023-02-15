/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_SKELETON_HPP
#define STAPL_SKELETONS_UTILITY_SKELETON_HPP

#include <type_traits>
#include <boost/mpl/has_xxx.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/runtime/serialization/typer_fwd.hpp>

namespace stapl {
namespace skeletons {

BOOST_MPL_HAS_XXX_TRAIT_DEF(skeleton_tag_type)


template <typename WF>
struct is_nested_skeleton;

//////////////////////////////////////////////////////////////////////
/// @brief Checks if the given element is a skeleton.
//////////////////////////////////////////////////////////////////////
template <typename S, bool = has_skeleton_tag_type<S>::value>
struct is_skeleton
  : std::true_type
{ };


template <typename S>
struct is_skeleton<S, false>
  : std::false_type
{ };


template <typename SkeletonTag, typename ExecutionTag>
struct skeleton_execution_traits
{
  template <typename OutputValueType>
  using result_type = void;
};

template <typename SkeletonTag>
struct skeleton_execution_traits<SkeletonTag,
                                 tags::nested_execution<false>>
{
  template <typename OutputValueType>
  using result_type = OutputValueType;
};

namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Tags a skeleton with a given @c Tag.
///
/// Tagging a skeleton can be useful in various scenarios:
/// @li customizing existing skeleton transformation
/// @li modifying the behavior of the skeleton (e.g., converting
///     a @ref reduce skeleton to lower-level primitives.
/// @li customizing existing skeleton execution strategies (e.g., using
///     a faster @c scan available on a system)
///
/// @tparam S   the enclosed skeleton
/// @tparam Tag a tag which uniquely identifies this skeleton
///
/// @todo do we need the skeleton and the tag at the same time? can we
/// restore a skeleton using the tag and operations? This would reduce
/// the compilation time.
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template <typename Skeleton, typename Tag>
class tagged_skeleton
  : public Skeleton
{
public:
  using skeleton_type     = Skeleton;
  using skeleton_tag_type = Tag;
  using type              = tagged_skeleton<Skeleton, Tag>;

  explicit tagged_skeleton(Skeleton const& skeleton)
    : Skeleton(skeleton)
  { }

  void define_type(typer& t)
  {
    t.base<Skeleton>(*this);
  }
};

} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Tags a skeleton with a given @c Tag.
///
/// @tparam Tag      a tag which uniquely identifies this skeleton
/// @param  skeleton skeleton to be tagged
//////////////////////////////////////////////////////////////////////
template <typename Tag, typename Skeleton>
auto tag(Skeleton const& skeleton)
STAPL_AUTO_RETURN((
  skeletons_impl::tagged_skeleton<Skeleton, Tag>(skeleton)
))

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_SKELETON_HPP
