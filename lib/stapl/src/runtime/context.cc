/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/context.hpp>
#include <stapl/runtime/non_rmi/rpc.hpp>
#include <ostream>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Updates count for sent intergang requests.
///
/// @param init_gid Gang to update its metadata.
/// @param gid      Gang which is responsible for increasing the count.
/// @param n        Nesting level.
/// @param sent     Number of sent requests.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
void fence_md_update_sent(const gang_md::id init_gid,
                          const gang_md::id gid,
                          const nesting_level n,
                          const unsigned int sent)
{
  auto* const g = gang_md_registry::try_get(init_gid);
  if (!g)
    STAPL_RUNTIME_ERROR("Outstanding coherence traffic for destroyed gang "
                        "detected.");
  g->get_fence_md().update_sent(gid, n, sent);
}


//////////////////////////////////////////////////////////////////////
/// @brief Updates count for processed intergang requests.
///
/// @param init_gid  Gang to update its metadata.
/// @param gid       Gang which is responsible for increasing the count.
/// @param n         Nesting level.
/// @param processed Number of processed requests.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
void fence_md_update_processed(const gang_md::id init_gid,
                               const gang_md::id gid,
                               const nesting_level n,
                               const unsigned int processed)
{
  auto* const g = gang_md_registry::try_get(init_gid);
  if (!g)
    STAPL_RUNTIME_ERROR("Outstanding coherence traffic for destroyed gang "
                        "detected.");
  g->get_fence_md().update_processed(gid, n, processed);
}


//////////////////////////////////////////////////////////////////////
/// @brief Updates counts for sent and processed intergang requests.
///
/// @param init_gid  Gang to update its metadata.
/// @param gid       Gang which is responsible for increasing the count.
/// @param n         Nesting level.
/// @param sent      Number of sent requests.
/// @param processed Number of processed requests.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
void fence_md_update(const gang_md::id init_gid,
                     const gang_md::id gid,
                     const nesting_level n,
                     const unsigned int sent,
                     const unsigned int processed)
{
  auto* const g = gang_md_registry::try_get(init_gid);
  if (!g)
    STAPL_RUNTIME_ERROR("Outstanding coherence traffic for destroyed gang "
                        "detected.");
  g->get_fence_md().update(gid, n, sent, processed);
}


// Flushes fence metadata
void context::flush_fence_md(void)
{
  if (m_sent==0 && m_proc==0 && !m_intergang_md)
    return;

  if (is_intragang()) {
    get_location_md().get_fence_md().add(m_sent, m_proc);
    m_sent = 0;
    m_proc = 0;
    return;
  }

  const auto init_gid = get_initiator().get_gang_id();
  const auto gid      = get_gang_id();
  const auto n        = get_nesting();

  gang_md::fence_md* f = nullptr;
  if (init_gid==gid) {
    // intergang context, but from the same initiator gang
    STAPL_RUNTIME_ASSERT(!m_intergang_md);
    f = &(get_gang_md().get_fence_md());
  }
  else if (m_intergang_md) {
    // intergang context and metadata aggregator exists
    f = m_intergang_md.get();
  }
  else {
    // intergang context
    auto* const g    = gang_md_registry::try_get(init_gid);
    const auto owner = gang_md_registry::id_owner(init_gid);
    if (!g && (owner==runqueue::get_process_id()))
      STAPL_RUNTIME_ERROR("Outstanding coherence traffic for destroyed gang "
                          "detected.");

    if (!g) {
      // gang metadata not present; send to gang id owner
      if (m_proc==0) {
        rpc(&fence_md_update_sent, owner, init_gid, gid, n, m_sent);
        m_sent = 0;
      }
      else if (m_sent==0) {
        rpc(&fence_md_update_processed, owner, init_gid, gid, n, m_proc);
        m_proc = 0;
      }
      else {
        rpc(&fence_md_update, owner, init_gid, gid, n, m_sent, m_proc);
        m_sent = 0;
        m_proc = 0;
      }
      return;
    }

    f = &(g->get_fence_md());
  }

  if (m_proc==0) {
    f->update_sent(gid, n, m_sent);
    m_sent = 0;
  }
  else if (m_sent==0) {
    f->update_processed(gid, n, m_proc);
    m_proc = 0;
  }
  else {
    f->update(gid, n, m_sent, m_proc);
    m_sent = 0;
    m_proc = 0;
  }

  m_intergang_md.reset();
}


// Increases the count of intergang requests
void context::count_intergang_pending(const gang_md::id gid,
                                      const unsigned int N)
{
  const auto init_gid = get_initiator().get_gang_id();
  const auto n        = get_nesting();
  if (init_gid==get_gang_id()) {
    get_gang_md().get_fence_md().update_sent(gid, n, N);
  }
  else {
    if (!m_intergang_md)
      m_intergang_md = gang_md_registry::get_fence_md(init_gid);
    m_intergang_md->update_sent(gid, n, N);
  }
}


// Prints a context object
std::ostream& operator<<(std::ostream& os, context const& ctx)
{
  return os << "(init="          << ctx.get_initiator()
            << ", source="       << ctx.get_id().source
            << ", current="      << ctx.get_current_location()
            << ", magic_number=" << std::uint64_t(ctx.get_id().magic)
            << ", intragang="    << ctx.is_intragang()
            << ", nesting="      << std::uint64_t(ctx.get_nesting())
            << ", epoch="        << std::uint64_t(ctx.get_epoch()) << ')';
}

} // namespace runtime

} // namespace stapl
