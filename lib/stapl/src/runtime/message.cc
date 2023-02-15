/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/config.hpp>
#include <stapl/runtime/message.hpp>
#include <stapl/runtime/exception.hpp>
#include <stapl/runtime/instrumentation.hpp>
#include <stapl/runtime/system.hpp>
#include <stapl/runtime/concurrency/concurrency.hpp>
#include <stapl/runtime/type_traits/aligned_storage.hpp>
#include <stapl/runtime/utility/pool.hpp>
#include <algorithm>
#include <cstring>
#include <utility>
#ifdef STAPL_RUNTIME_DEBUG
# include <atomic>
#endif

#ifndef STAPL_RUNTIME_MSG_SIZE
/// Default message size.
# define STAPL_RUNTIME_MSG_SIZE \
  (4096 - (sizeof_message + message::header_size()))
#endif

#ifndef STAPL_RUNTIME_MSG_POOL_MIN_NUM
/// Minimum number of preallocated messages.
# define STAPL_RUNTIME_MSG_POOL_MIN_NUM 128
#endif

#ifndef STAPL_RUNTIME_MSG_POOL_MAX_MEM
/// Default maximum fraction of available system memory used for messages in
/// pool.
# define STAPL_RUNTIME_MSG_POOL_MAX_MEM 0.10
#endif

namespace stapl {

namespace runtime {

/// Aligned @c sizeof @ref message.
const std::size_t sizeof_message = sizeof(aligned_storage_t<sizeof(message)>);

/// Default message capacity.
static std::size_t default_msg_capacity = 0;

/// Pool allocation enable flag.
static bool enable_msg_pool = true;

/// Number of processes per node.
static unsigned int nppn = 0;

/// Minimum number of messages in the pool.
static std::size_t min_num_msgs = 0;

/// Maximum fraction of memory used for communication buffers.
static double max_mem_fraction = 0.0;

/// Message pool.
static pool msg_pool;

#ifdef STAPL_RUNTIME_DEBUG
/// Allocated bytes in messages.
static std::atomic<std::size_t> alloc_bytes(0);
#endif


//////////////////////////////////////////////////////////////////////
/// @brief Returns the number of initial number of messages to allocate and the
///        maximum number of messages allowed.
///
/// The calculation of the maximum number of messages allowed is based on a
/// heuristic that uses the number of processes running on a node.
//////////////////////////////////////////////////////////////////////
static
std::pair<std::size_t, std::size_t> allocation_sizes(std::size_t size) noexcept
{
  // available memory
  const std::size_t avail_mem = get_available_physical_memory();
  // memory per process per node
  const float mem_ppn = (float(avail_mem) / float(nppn));
  // max memory available to the runtime
  const std::size_t rts_mem = (mem_ppn * max_mem_fraction);
  // number of messages (initial, maximum)
  const std::size_t max_nmsgs = (rts_mem / size);
  // correcting initial number of messages if exceeded by max number of messages
  return std::make_pair(std::min(min_num_msgs, max_nmsgs), max_nmsgs);
}


// Initializes the message pool
void message::initialize(option const& opts, const unsigned int ppn)
{
  const std::size_t size = opts.get<std::size_t>("STAPL_RUNTIME_MSG_SIZE",
                                                 STAPL_RUNTIME_MSG_SIZE);

  nppn = ppn;
  enable_msg_pool = (opts.get<int>("STAPL_RUNTIME_MSG_POOL_DISABLE", 0)==0);
  if (enable_msg_pool) {
    min_num_msgs = opts.get<std::size_t>("STAPL_RUNTIME_MSG_POOL_MIN_NUM",
                                         STAPL_RUNTIME_MSG_POOL_MIN_NUM);
    if (min_num_msgs==0) {
      STAPL_RUNTIME_ERROR("Minimum number of messages "
                          "(STAPL_RUNTIME_MSG_POOL_MIN_NUM) has to be at least "
                          "1 if message pooling is enabled.");
    }

    max_mem_fraction = opts.get<double>("STAPL_RUNTIME_MSG_POOL_MAX_MEM",
                                        STAPL_RUNTIME_MSG_POOL_MAX_MEM);
    if (max_mem_fraction<0.0 || max_mem_fraction>1.0) {
      STAPL_RUNTIME_ERROR("Maximum memory use (STAPL_RUNTIME_MSG_POOL_MAX_MEM) "
                          "has to be in the range (0.0, 1.0).");
    }
  }

  set_default_body_capacity(size);
}


// Finalizes the message pool
void message::finalize(void)
{
  STAPL_RUNTIME_ASSERT(alloc_bytes==0);
  msg_pool.purge();
  nppn = 0;
}


// Sets the default message body capacity
void message::set_default_body_capacity(const std::size_t n)
{
  if (n < message::header_size()) {
    STAPL_RUNTIME_ERROR("Increase default message size "
                        "(STAPL_RUNTIME_MSG_SIZE).");
  }

  STAPL_RUNTIME_ASSERT(alloc_bytes==0);

  // actual message capacity
  default_msg_capacity = (header_size() + n);
  STAPL_RUNTIME_STATISTICS("message default capacity", default_msg_capacity);

  if (enable_msg_pool) {
    // figure out how many messages to preallocate and their maximum number
    const auto p = allocation_sizes(default_msg_capacity);
    if (p.first==0 || p.first>p.second) {
      STAPL_RUNTIME_ERROR("Memory estimation failed or maximum available "
                          "memory (STAPL_RUNTIME_MSG_POOL_MAX_MEM) too "
                          "little.");
    }

    // clear old messages
    msg_pool.reset((sizeof_message + message::default_capacity()),
                   p.first,
                   p.second);
  }
  else {
    msg_pool.reset();
  }
}


// Default message capacity
std::size_t message::default_capacity(void) noexcept
{
  return default_msg_capacity;
}


// Default message header size
std::size_t message::header_size(void) noexcept
{
  return sizeof(aligned_storage_t<sizeof(header_type)>);
}


// Construct a new message with default size
message* message::construct(void)
{
  if (enable_msg_pool) {
    // attempt to pool allocate message
    void* p = msg_pool.allocate();
    if (p) {
      STAPL_RUNTIME_STATISTICS("message_pool # allocated messages",
                               msg_pool.size());
      return new(p) message{true};
    }
  }

  // failed to get from the pool or pool disabled
  STAPL_RUNTIME_STATISTICS("message arbitrary size",
                           message::default_capacity());
  void* p = new char[sizeof_message + message::default_capacity()];
  return new(p) message{false};
}


// Construct a new message with arbitrary size
message* message::construct(const std::size_t n, const bool exact_size)
{
  if (n <= message::default_capacity()) {
    if (!exact_size) {
      // it's ok for the buffer to be bigger than the requested capacity
      return construct();
    }

    if (enable_msg_pool) {
      // attempt to pool allocate message
      void* p = msg_pool.allocate();
      if (p) {
        STAPL_RUNTIME_STATISTICS("message_pool # allocated messages",
                                 msg_pool.size());
        return new(p) message{true};
      }
    }
  }

  // failed to get from the pool and exact message requested
  STAPL_RUNTIME_STATISTICS("message arbitrary size", n);
  void* p = new char[sizeof_message + n];
  return new(p) message{n};
}


// Releases the given message
void message::destroy(message* msg)
{
  const bool managed = msg->m_managed;
  msg->~message();
  if (managed) {
    msg_pool.release(msg);
  }
  else {
    char* p = reinterpret_cast<char*>(msg);
    delete[] p;
  }
}


// Default capacity message. Memory managed by the runtime, but it can be
// malloced if necessary.
message::message(const bool managed) noexcept
: m_body_capacity(default_body_capacity()),
  m_payload_offset(0),
  m_managed(managed),
  m_forwarded(false)
{
  header().reset();
#ifdef STAPL_RUNTIME_DEBUG
  // count bytes
  const std::size_t s = (sizeof_message + capacity());
  alloc_bytes.fetch_add(s, std::memory_order_relaxed);
  // erase message body
  std::memset(body(), 0, body_capacity());
#endif

}


// Arbitrary capacity message. Never managed by the runtime.
message::message(const std::size_t n) noexcept
: m_body_capacity(n - header_size()),
  m_payload_offset(0),
  m_managed(false),
  m_forwarded(false)
{
  header().reset();
#ifdef STAPL_RUNTIME_DEBUG
  STAPL_RUNTIME_ASSERT(n >= header_size());
  // count bytes
  const std::size_t s = (sizeof_message + capacity());
  alloc_bytes.fetch_add(s, std::memory_order_relaxed);
  // erase message body
  std::memset(body(), 0, body_capacity());
#endif
}


message::~message(void)
{
#ifdef STAPL_RUNTIME_DEBUG
  // count bytes
  const std::size_t s = (sizeof_message + capacity());
  const std::size_t v = alloc_bytes.fetch_sub(s, std::memory_order_relaxed);
  if (v<s)
    STAPL_RUNTIME_ERROR("Corruption in memory pool.");
  // erase message header and body
  std::memset(data(), 0, capacity());
#endif
}


void message::reset(void) noexcept
{
  header().reset();
  m_payload_offset = 0;
  m_forwarded      = false;
#ifdef STAPL_RUNTIME_DEBUG
  // erase message body
  std::memset(body(), 0, body_capacity());
#endif
}


char* message::data(void) noexcept
{
  return (reinterpret_cast<char*>(this) + sizeof_message);
}


char const* message::data(void) const noexcept
{
  return (reinterpret_cast<char const*>(this) + sizeof_message);
}


message_ptr message::clone(void) const
{
  const std::size_t s = body_size();
  auto m = create(s, true);
  m->reserve(s);
  STAPL_RUNTIME_ASSERT(m->size()==size() && m->body_size()==body_size());
  std::memcpy(m->data(), data(), size());
  m->m_payload_offset = m_payload_offset;
  return m;
}

} // namespace runtime

} // namespace stapl
