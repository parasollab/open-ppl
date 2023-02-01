/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_LOCATION_SPECIFIC_STORAGE_HPP
#define STAPL_RUNTIME_LOCATION_SPECIFIC_STORAGE_HPP

#include "context.hpp"
#include <functional>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Supports global objects in the presence of locations.
///
/// All non-const global object have to be declared as
/// @ref location_specific_storage objects, since locations are effectively
/// virtual threads. This class is modeled after @c boost::thread_specific_ptr.
///
/// Global const variables do not have to be instances of
/// @ref location_specific_storage, since the latter imposes some additional
/// overhead.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
template<typename T>
class location_specific_storage
{
private:
  void* const                  m_id;
  const std::function<T(void)> m_creator_f;

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an object of type @p T with the given arguments.
  //////////////////////////////////////////////////////////////////////
  struct creator
  {
    template<typename... Args>
    T operator()(Args&&... args) const
    { return T(std::forward<Args>(args)...); }
  };

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a @ref location_specific_storage object that default
  ///        constructs the underlying @p T object.
  //////////////////////////////////////////////////////////////////////
  location_specific_storage(void)
  : m_id{this}
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a @ref location_specific_storage object that calls the
  ///        constructor of the underlying @p T object with the given arguments.
  //////////////////////////////////////////////////////////////////////
  template<typename... Args>
  location_specific_storage(Args&&... args)
  : m_id{this},
    m_creator_f{std::bind(creator{}, std::forward<Args>(args)...)}
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Replaces the stored object.
  //////////////////////////////////////////////////////////////////////
  template<typename U>
  T& reset(U&& u)
  {
    auto& ctx = runtime::this_context::get();
    return ctx.get_location_md().reset_lss<T>(m_id, std::forward<U>(u));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the stored @p T object.
  ///
  /// If a stored object @p T has not been created yet, it will be created with
  /// any arguments given through @ref location_specific_storage(Args&&...).
  //////////////////////////////////////////////////////////////////////
  T& get(void) const
  {
    auto& ctx = runtime::this_context::get();
    return ctx.get_location_md().get_lss<T>(m_id, m_creator_f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Attempts to return the stored @p T object.
  ///
  /// If a stored object @p T has not been created yet, it returns @c nullptr.
  //////////////////////////////////////////////////////////////////////
  T* try_get(void) const
  {
    auto* const ctx = runtime::this_context::try_get();
    return (!ctx ? nullptr : ctx->get_location_md().try_get_lss<T>(m_id));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Destroys the stored @p T object.
  //////////////////////////////////////////////////////////////////////
  void destroy(void)
  {
    auto* const ctx = runtime::this_context::try_get();
    if (ctx)
      ctx->get_location_md().destroy_lss(m_id);
  }
};

} // namespace stapl

#endif
