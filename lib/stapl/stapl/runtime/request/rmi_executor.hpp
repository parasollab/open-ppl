/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_REQUEST_RMI_EXECUTOR_HPP
#define STAPL_RUNTIME_REQUEST_RMI_EXECUTOR_HPP

#include "rmi_request.hpp"
#include "../context.hpp"
#include "../message.hpp"
#include "../exception.hpp"
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Executes RMI requests for a specific context, preserving the requests
///        that could not be executed for a later invocation.
///
/// @ingroup requestExecution
//////////////////////////////////////////////////////////////////////
class rmi_executor
{
private:
  message_ptr                 m_msg;
  message::payload_range_type m_payload;

public:
  bool empty(void) const noexcept
  { return !m_msg; }

  bool operator()(context& ctx)
  {
    STAPL_RUNTIME_ASSERT(!empty());
    do {
      auto& req = *reinterpret_cast<rmi_request*>(m_payload.begin());
      const bool done = req(ctx);
      if (!done)
        return false;
      m_payload.advance_begin(req.size());
    } while (!m_payload.empty());
    m_msg.reset();
    ctx.count_processed();
    return true;
  }

  bool operator()(context& ctx, message_ptr m)
  {
    STAPL_RUNTIME_ASSERT(empty());
    m_msg     = std::move(m);
    m_payload = m_msg->payload();
    return operator()(ctx);
  }
};

} // namespace runtime

} // namespace stapl

#endif
