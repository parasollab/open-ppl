/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_INCREMENTAL_HPP
#define STAPL_PARAGRAPH_INCREMENTAL_HPP

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Concept base class used by factory workfunctions to denote
///   that they may not complete their work of creating tasks during a
///   single invocation of the function operator and hence must be
///   queried to check for required reinvocation.
///
/// Adds the requirement that the factory workfunction implement the
///   following method signature which will guard calls to any reinvocations,
///   (i.e., only invoked again if method returns false).
///
/// bool finished(void) const;
///
/// @ingroup paragraph
//////////////////////////////////////////////////////////////////////
class incremental_wf
{ };

} // namespace stapl

#endif // STAPL_PARAGRAPH_INCREMENTAL_HPP
