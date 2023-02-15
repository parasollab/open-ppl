/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RMI_DELEGATE_HPP
#define STAPL_RUNTIME_RMI_DELEGATE_HPP

#include "../rmi_handle_fwd.hpp"
#include "../type_traits/type_id.hpp"
#include <cstring>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Delegate for an RMI request.
///
/// It describes the request by keeping the type id of the request and a handle
/// to the object it was invoked on.
///
/// Unlike ordinary delegates, the user cannot use this to call the function. It
/// is only a means for comparing requests.
///
/// It is used when combining requests so that two unrelated requests are not
/// accidentally combined.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
class rmi_delegate
{
private:
  class dummy_class;
  typedef void (dummy_class::*pmf_type)(void);

  type_id                          m_request;
  rmi_handle::internal_handle_type m_handle;
  pmf_type                         m_pmf;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an invalid @ref rmi_delegate.
  //////////////////////////////////////////////////////////////////////
  constexpr rmi_delegate(void) noexcept
  : m_request(),
    m_pmf(nullptr)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref rmi_delegate based on the request type, the
  ///        object handle and the member function pointer.
  //////////////////////////////////////////////////////////////////////
  template<typename Handle, typename MemFun>
  rmi_delegate(const type_id request_id,
               Handle const& h,
               MemFun const& pmf) noexcept
  : m_request(request_id),
    m_handle(h.internal_handle()),
    m_pmf(nullptr)
  {
    static_assert(sizeof(MemFun)<=sizeof(pmf_type),
                  "Cannot store pointer to member function.");
    std::memcpy(&m_pmf, &pmf, sizeof(MemFun));
  }

  friend constexpr bool operator==(rmi_delegate const& x,
                                   rmi_delegate const& y) noexcept
  {
    return (x.m_request==y.m_request &&
            x.m_handle==y.m_handle   &&
            x.m_pmf==y.m_pmf);
  }

  friend constexpr bool operator!=(rmi_delegate const& x,
                                   rmi_delegate const& y) noexcept
  {
    return !(x==y);
  }
};

} // namespace runtime

} // namespace stapl

#endif
