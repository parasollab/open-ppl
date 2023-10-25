/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_NULL_COARSENER_HPP
#define STAPL_VIEWS_METADATA_NULL_COARSENER_HPP

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Identity coarsener. The same input views are returned as
///        result of the coarsener.
//////////////////////////////////////////////////////////////////////
struct null_coarsener
{
  template<typename Views>
  Views operator()(Views const& views) const
  {
    return views;
  }
};

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_NULL_COARSENER_HPP
