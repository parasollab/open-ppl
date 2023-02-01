/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/synchronization.hpp>
#include <stapl/runtime/context.hpp>
#include <stapl/runtime/yield.hpp>
#include <stapl/runtime/collective/allreduce_object.hpp>
#include <stapl/runtime/non_rmi/rpc.hpp>
#include <boost/serialization/unordered_map.hpp>

namespace stapl {

namespace runtime {

constexpr unsigned int unequal_unregistration_count =
  std::numeric_limits<unsigned int>::max();


//////////////////////////////////////////////////////////////////////
/// @brief Reduction function used by RMI fence implementation.
/// The first element in the pair is the difference of RMIs sent and
/// processed.  The second element is used to detect unequal unregistration
/// of p_objects on locations of the gang.
//////////////////////////////////////////////////////////////////////
struct rmi_fence_reduce
{
  using reduce_type = std::pair<int, unsigned int>;

  reduce_type operator()(reduce_type const& lhs, reduce_type const& rhs) const
  {
    return lhs.second == rhs.second ?
      reduce_type(lhs.first + rhs.first, lhs.second)
      : reduce_type(lhs.first + rhs.first, unequal_unregistration_count);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief RMI fence implementation.
///
/// For one location, the fence succeeds iff
/// \f$ \sum_{lid=i}^{p} (sent - processed) + |f()| = 0 \f$.
///
/// For more than one locations, the fence succeeds iff
/// \f$ \sum_{lid=i}^{p} (sent - processed) + new_sent + f() = 0 \f$
/// for at least two iterations.
///
/// The fence ensures that all outstanding RMI requests have completed.
///
/// Post quiescence, the fence verifies that the number object
/// unregistrations that have been pending from the previous phase is
/// equivalent on all locations.  This is a cheap sanity check of the
/// stronger, required condition that all locations of a given p_object
/// unregister the object in the same program phase (i.e., during execution
/// occurring between the same successive calls to @p rmi_fence.
///
/// In the simple case, iterating is not necessary. However, two special cases
/// make iterating necessary for correctness:
///  -# if an RMI invokes a method that makes another RMI, which invokes a
///     method that makes another RMI, etc.
///  -# the communication layer does not make ordering guarantees between point-
///     to-point traffic and collective operations' traffic.
///
/// @warning This is an SPMD function.
///
/// @ingroup requestBuildingBlock
///
/// @todo The iteration restarts from the first phase, however it may be correct
///       to return to the second iteration.
///
/// @note Reducing the unregistration counts is not really necessary.
/// Broadcasting out the count from one location to all others and checking
/// for equality would suffice.  This would reduce bits and cycles up the
/// reduction tree but would require specialization of the allreduce object.
//////////////////////////////////////////////////////////////////////
template<typename Function>
void rmi_fence_impl(context& ctx, Function&& f)
{
  ctx.flush();

  auto& l = ctx.get_location_md();
  auto& g = l.get_gang_md();

  // quiescence detection
  if (g.size()==1) {
    fence_section fs{l};
    yield_until(no_context,
                [&f, &l, &ctx]
                {
                  ctx.flush();
                  return (f()                                           &&
                          l.get_gang_md().get_fence_md().none_pending() &&
                          (l.get_fence_md().pending()==0));
                });
  }
  else {
    const bool leader      = l.is_leader();
    const process_id owner = gang_md_registry::id_owner(g.get_id());
    const bool is_owner    = (runqueue::get_process_id()==owner);

    using red_var_t = rmi_fence_reduce::reduce_type;

    allreduce_object<red_var_t, rmi_fence_reduce> ar{ctx, rmi_fence_reduce()};

    auto const& l_fence_md = l.get_fence_md();

    fence_section fs{l};

    runqueue::yield();

    red_var_t result;

    for (int phase = 0, old_sent = 0; phase<2; ++phase) {
      ctx.flush();
      int n = (f() ? 0 : 1);

      if (leader) {
        if (is_owner) {
          // check if pending intergang requests completed
          yield_if_not(no_context,
                       [&g, &n, &ctx]
                       {
                         ctx.flush();
                         if (g.get_fence_md().none_pending())
                           return true;
                         // pending intergang requests
                         ++n;
                         return false;
                       });
        }
        else {
          // send counts of pending intergang requests to owner
          if (yield_until(no_context,
                          [&g, owner, &ctx]
                          {
                           ctx.flush();
                            auto c = g.get_fence_md().retrieve();
                            if (c.empty())
                              return true;
                            rpc(&gang_md::update_fence_md,
                                owner, g.get_id(), std::move(c));
                            return false;
                          })) {
            // pending intergang requests
            ++n;
          }
        }
      }

      // number of pending intragang requests
      n += l_fence_md.pending();

      // figure out if new requests were generated since last phase
      if (phase==0) {
        old_sent = l_fence_md.get_sent();
      }
      else {
        const int curr_sent = l_fence_md.get_sent();
        STAPL_RUNTIME_ASSERT(curr_sent >= old_sent);
        n += (curr_sent - old_sent);
        old_sent = curr_sent;
      }

      // calculate global number of pending requests (synchronization point)
      ar(red_var_t(n, l.num_pending_unregistrations()));

      result = ar.get();

      // decide locally next phase
      if (result.first != 0) {
        phase = -1; // inconsistent state - fall back to first phase
        // phase = 0; // inconsistent state - repeat second phase
      }
    }

    if (result.second == unequal_unregistration_count)
      STAPL_RUNTIME_ERROR("Unequal unregistration count detected during fence");
  }
}


// RMI fence that only takes care of RMIs
void rmi_fence(context& ctx)
{
  rmi_fence_impl(ctx, [] { return true; });
}


// RMI fence that takes care of RMIs and additional function object
void rmi_fence(context& ctx, std::function<bool(void)> f)
{
  rmi_fence_impl(ctx, std::move(f));
}

} // namespace runtime

} // namespace stapl
