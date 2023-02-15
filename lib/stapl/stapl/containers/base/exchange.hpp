/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_BASE_EXCHANGE_HPP
#define STAPL_CONTAINERS_BASE_EXCHANGE_HPP

#include "exchange_sends.hpp"
#include "linear_chunks.hpp"

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Dumps in turn for each location counts and displacements
/// information for alltoallv.  Solely for debugging / analysis of
/// communication pattern.
//////////////////////////////////////////////////////////////////////
template<typename Domains>
void dump_alltoallv_inputs(std::vector<int> const& send_counts,
                           std::vector<int> const& send_displs,
                           Domains const& domains)
{
  const auto myid   = get_location_id();
  const auto nprocs = get_num_locations();

  // Dump Send Receive Counts
  for (size_t j = 0; j < nprocs; ++j)
  {
    if (j == myid)
    {
      std::cout << get_affinity() << ": DUMPING MY DATA\n";

      for (size_t i = 0; i != nprocs; ++i)
        std::cout << get_affinity() << ": TO " << i
                  << " /// " <<  send_counts[i]
                  << " /// " << send_displs[i] << "\n";


      for (size_t i = 0; i != nprocs; ++i)
      {
        for (size_t chunk=0; chunk!=domains.size(); ++chunk)
          std::cout << get_affinity() << ": FROM " << i
                    << " at chunk " << chunk << " receive domains "
                    << domains[chunk][i][0] << " and "
                    << domains[chunk][i][1] << "\n";
      }
    }
    rmi_fence();
  }
}

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Perform an exchange of elements in the given view (assumed
/// to be a native view over a simple, balanced distribution), so that
/// the values are not changed by their position (i.e., gids) in the
/// container.
///
/// The implementation right now assumes a partition style exchange where
/// elements are moved around a pivot point.
///
/// @param view A view over the container whose elements are to be exchanged.
/// @param counts The counts of elements on this location in each of the
///   target partitions
/// @param global_offsets The global offsets where elements from this location
///   would be written in stable version of the exchange algorithms.
/// @param sums The global sums of elements in each of the target partitions.
/// @param b_stable Denotes whether exchange process should maintain
///   the relative order between elements on different locations headed to the
///   same target partition or if this can be relaxed to minimize communication
///   and thus increase performance.
/// @param b_copy_last Denote whether last partition is of interest to caller.
///   If false, the values of the elements in last partition are not defined.
///
/// @todo Extend implementation to support more complex distributions.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Counts, typename Offsets, typename Sums>
void exchange(View& view, Counts&& counts, Offsets&& global_offsets,
              Sums&& sums, bool b_stable = true, bool b_copy_last = true)
{
  // Non stable implementation needs better testing and performance
  // numbers to be enabled.
  b_stable = true;

  const auto myid              = get_location_id();
  const auto nprocs            = get_num_locations();
  const int num_chunks         = 8;

  using md_t        = std::vector<int>;
  using domain_t    = indexed_domain<size_t>;
  using domain_md_t = std::vector<std::array<domain_t, 2>>;

  md_t send_counts(nprocs, 0);
  md_t send_displs(nprocs, 0);

  tuple<int, int> send_ret;

  using chunked_md_t        = std::vector<md_t>;
  using chunked_domain_md_t = std::vector<domain_md_t>;

  chunked_md_t chunked_send_counts(num_chunks, md_t(nprocs, 0));
  chunked_md_t chunked_send_displs(num_chunks, md_t(nprocs, 0));
  chunked_md_t chunked_recv_counts(num_chunks, md_t(nprocs, 0));
  chunked_md_t chunked_recv_displs(num_chunks, md_t(nprocs, 0));
  chunked_domain_md_t chunked_recv_domains(num_chunks, domain_md_t(nprocs));

  using value_buffer_t        = std::vector<typename View::value_type>;
  using deferred_descriptor_t =
    tuple<domain_t, std::shared_ptr<value_buffer_t>>;
  using deferred_buffer_t     = std::vector<deferred_descriptor_t>;

  value_buffer_t    workspace;
  deferred_buffer_t deferred_queue;

  //
  // Distribute Offsets, Compute the Pivot Processor
  //
  using offsets_t = std::array<int, 2>;

  offsets_t local_offsets;
  local_offsets[0] = global_offsets[myid].first;
  local_offsets[1] = global_offsets[myid].second;

  std::vector<offsets_t> offsets(nprocs, offsets_t{{0, 0}});

  basic_allgather_rmi(local_offsets.data(), offsets.front().data(), 2);

  //
  // Compute the Send Lists.
  //
  tuple<int, int> send_rets;

  auto ret1 =
    compute_sends(counts, offsets, sums, b_stable, b_copy_last);

  send_rets   = get<0>(ret1);
  send_counts = std::move(get<1>(ret1));
  send_displs = std::move(get<2>(ret1));

  chunkify_linear_send(send_counts, send_displs,
                       chunked_send_counts, chunked_send_displs);

  //
  // Compute Receives
  //
  compute_senders(counts, offsets, sums, b_stable, b_copy_last,
                  chunked_recv_counts, chunked_recv_domains);

  for (int chunk = 0; chunk != num_chunks; ++chunk)
    compute_receives(chunked_recv_counts[chunk], chunked_recv_displs[chunk]);

  chunkify_linear_recv(chunked_recv_counts, workspace);

  auto& send_values =
    view.container().distribution().container_manager().begin()->container();

  for (int chunk=0; chunk < num_chunks; ++chunk)
  {
    alltoallv_rmi(send_values.data(), chunked_send_counts[chunk].data(),
                  chunked_send_displs[chunk].data(),
                  workspace.data(), chunked_recv_counts[chunk].data(),
                  chunked_recv_displs[chunk].data());

    linear_store(send_values,
                 workspace,
                 deferred_queue,
                 chunked_recv_counts[chunk],
                 chunked_recv_displs[chunk],
                 chunked_recv_domains[chunk],
                 chunk, num_chunks);
  }
}

} // namespace stapl

#endif // STAPL_CONTAINERS_BASE_EXCHANGE_HPP
