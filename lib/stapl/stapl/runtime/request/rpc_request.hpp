/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_REQUEST_RPC_REQUEST_HPP
#define STAPL_RUNTIME_REQUEST_RPC_REQUEST_HPP

#include "../message.hpp"
#include "../exception.hpp"
#include <cstddef>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Encapsulates an RPC request directed to a process for subsequent
///        execution via the function operator.
///
/// @ref rpc_request objects package all information for buffering and transfer.
/// The 'header' contains the size of the request. The 'body' (derived class)
/// has the desired function and any arguments required to invoke it.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
class rpc_request
{
public:
  typedef std::size_t size_type;

private:
  size_type m_size;

public:
  constexpr explicit rpc_request(const size_type size) noexcept
  : m_size(size)
  { }

  rpc_request(rpc_request const&) = delete;
  rpc_request& operator=(rpc_request const&) = delete;

protected:
  ~rpc_request(void) = default;

public:
  constexpr size_type size(void) const noexcept
  { return m_size; }

  size_type& size(void) noexcept
  { return m_size; }

  virtual void operator()(message_shared_ptr&) = 0;
};


//////////////////////////////////////////////////////////////////////
/// @brief Executes RPC requests for a process.
///
/// @ingroup requestExecution
//////////////////////////////////////////////////////////////////////
struct rpc_executor
{
  void operator()(message_ptr m) const
  {
    STAPL_RUNTIME_ASSERT(m->type()==header::RPC);
    message_shared_ptr sm{std::move(m)};
    message::payload_range_type payload = sm->payload();
    do {
      auto& req = *reinterpret_cast<rpc_request*>(payload.begin());
      payload.advance_begin(req.size());
      req(sm);
    } while (!payload.empty());
  }
};

} // namespace runtime

} // namespace stapl

#endif
