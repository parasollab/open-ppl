/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXPLICIT_TREE_INFO_H
#define STAPL_SKELETONS_EXPLICIT_TREE_INFO_H

namespace stapl {

namespace paragraph_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Represents the statistics of a balanced binary tree of tasks
/// that is part of a PARAGRAPH performing a reduction or prefix scan
/// on a view.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
struct tree_info
{
  /// The width of the tree.
  int width;

  /// The height of the tree.
  int height;

  /// @brief The number of view elements to the left of the first view element
  ///   processed by this tree.
  int leaf_offset;

  /// The id of the task that will process the leftmost leaf of the tree.
  int id_offset;

  /// The id of the root task in the tree.
  int root_task_id;

  /// @brief The current task id for the tree.  Needed because the
  /// @ref map_reduce_factory and @ref prefix_scan_factory generate tasks of the
  ///   task graph beginning at the leaves of the reduction tree and generating
  ///   one level at a time, which requires traversing the entire set of trees
  ///   for each level of tasks.
  int curr_id;

  /// @brief The number of view elements that are processed by smaller trees to
  ///   the right of the current tree.
  int elements_remaining;
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function used when traversing the set of trees to determine
/// which tree contains the task to process the specified view element.
///
/// @ingroup skeletonsExplicitFactoriesInternal
//////////////////////////////////////////////////////////////////////
struct contains_leaf
{
  int leaf;

  explicit contains_leaf(int l)
    : leaf(l)
  { }

  bool operator()(tree_info const& tree)
  {
    return (tree.leaf_offset <= leaf) &&
           (leaf < tree.leaf_offset + tree.width);
  }
};

} // namespace paragraph_impl

} // namespace stapl

#endif
