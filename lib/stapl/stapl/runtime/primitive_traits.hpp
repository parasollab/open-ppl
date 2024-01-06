/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_PRIMITIVE_TRAITS_HPP
#define STAPL_RUNTIME_PRIMITIVE_TRAITS_HPP

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Traits for primitives.
///
/// These traits document the nature of the primitives and their communication
/// characteristics.
///
/// @see instrumentation
/// @ingroup instrumentation
//////////////////////////////////////////////////////////////////////
struct primitive_traits
{
  enum
  {
    /// Other primitive.
    other        = 0x0000,
    /// Environment creation primitive.
    environment  = 0x0001,
    /// Communication primitive.
    comm         = 0x0002,
    /// Synchronization primitive.
    sync         = 0x0004,
    /// Yielding primitive.
    yield        = 0x0008,
    /// Point-to-point primitive.
    p2p          = 0x0010,
    /// Point-to-many primitive.
    p2m          = 0x0020,
    /// Collective primitive.
    coll         = 0x0040,
    /// Blocking primitive.
    blocking     = 0x0100,
    /// Non-blocking primitive.
    non_blocking = 0x0200,
    /// Ordered primitive.
    ordered      = 0x1000,
    /// Ordered primitive.
    unordered    = 0x2000
  };
};

} // namespace runtime

} // namespace stapl

#endif
