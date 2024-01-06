/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_P_OBJECT_REGISTRY_HPP
#define STAPL_RUNTIME_P_OBJECT_REGISTRY_HPP

#include "rmi_handle_fwd.hpp"
#include <typeinfo>

namespace stapl {

namespace runtime {

class p_object_registry
{
public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Associates handle @p h with @p p of type @p t.
  //////////////////////////////////////////////////////////////////////
  static void register_object(const rmi_handle* h,
                              const void* p,
                              std::type_info const& t);

  //////////////////////////////////////////////////////////////////////
  /// @brief Disassociates handle @p h from its registered object.
  //////////////////////////////////////////////////////////////////////
  static void unregister_object(const rmi_handle* h);

  //////////////////////////////////////////////////////////////////////
  /// @brief Verifies that @p p can be cast to type @p t.
  //////////////////////////////////////////////////////////////////////
  static void verify_object_type(const void* p, std::type_info const& t);
};

} // namespace runtime

} // namespace stapl

#endif
