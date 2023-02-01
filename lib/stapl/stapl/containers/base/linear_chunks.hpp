/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_LINEAR_CHUNKS_HPP

#ifdef _STAPL
namespace stapl {
#endif


//////////////////////////////////////////////////////////////////////
/// @brief Given an input of per location send counts and offsets,
/// compute a chunked send schedule by populating the chunked send
/// count and displacement data structures.  Used for an iterative
/// use of underlying all to all primitive which has been shown
/// to sometimes improve performance.
//////////////////////////////////////////////////////////////////////
template<typename Counts, typename Displs,
         typename ChunkedCounts, typename ChunkedDispls>
void chunkify_linear_send(Counts const& send_counts,
                          Displs const& send_displs,
                          ChunkedCounts& chunked_send_counts,
                          ChunkedDispls& chunked_send_displs)
{
  const size_t num_chunks = chunked_send_counts.size();

  using descriptor_t = std::array<int, 3>;

  std::vector<descriptor_t> sends;

  int total_count = 0;

  const int counts_size = send_counts.size();

  for (int i = 0; i != counts_size; ++i)
  {
    if (send_counts[i] > 0)
    {
      sends.push_back({{int{i}, send_counts[i], send_displs[i]}});
      total_count += send_counts[i];
    }
  }

  std::sort(sends.begin(), sends.end(),
            [](descriptor_t const& lhs, descriptor_t const& rhs)
            { return lhs[2] < rhs[2]; });

  auto compute_chunk_count =
    [num_chunks, total_count](size_t chunk)
    {
      return total_count / num_chunks
       + (chunk < total_count % num_chunks ? 1 : 0);
    };

  int current_idx  = 0;
  int current_count = 0;

  for (size_t i = 0; i != num_chunks; ++i)
  {
    const int chunk_size = compute_chunk_count(i);
    int chunk_count      = 0;

    while (chunk_count != chunk_size)
    {
      const int max_chunk_count   = chunk_size - chunk_count;
      const int max_current_count = sends[current_idx][1] - current_count;
      const int current_proc      = sends[current_idx][0];
      const int current_displ     = sends[current_idx][2] + current_count;

      if (max_chunk_count >= max_current_count)
      {
        chunked_send_counts[i][current_proc] = max_current_count;
        chunked_send_displs[i][current_proc] = current_displ;

        ++current_idx;
        current_count = 0;
        chunk_count += max_current_count;
      }
      else
      {
        chunked_send_counts[i][current_proc] = max_chunk_count;
        chunked_send_displs[i][current_proc] = current_displ;

        current_count += max_chunk_count;
        chunk_count   += max_chunk_count;
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Given per location receive counts for a given location,
///  compute the appropriate receive offsets for each location and
///  As well as prefix sum of these receive counts.
//////////////////////////////////////////////////////////////////////
template<typename Counts, typename Displs>
void compute_receives(Counts const& recv_counts, Displs& recv_displs)
{
  const auto nprocs = get_num_locations();

  for (size_t i = 0; i != nprocs; ++i)
    recv_displs[i] = i == 0 ? 0 : recv_displs[i-1] + recv_counts[i-1];
}


//////////////////////////////////////////////////////////////////////
/// @brief Given per location receive counts along with receive counts and
///   offsets for a chunked send schedule, compute the offsets for the
///   workspace receive buffer and resize the buffer appropriately.
///
/// @param recv_counts The per location receive counts for this location.
/// @param chunked_recv_counts The receive counts for each step in the
///   in the chunked send schedule.
/// @param chunked_recv_displs The receive offset for each step in the
///   in the chunked send schedule.
/// @param chunked_recv_target_displs Storage for offsets in the
///   workspace buffer for each chunk.  Populated in this call.
/// @param send_ret The receive position and count for sends to self.
/// @param workspace The temporary workspace buffer used to receive
///   elements for a chunk before they are inserted in the local
///   portion of the sequence.  This is resized to the max chunk receive
///   size in this call.
///
/// @todo Verify proper order of elements from other location.
/// (1) maybe maybe myid+1 to != nprocs and then before me or
/// (2) a bit telling me whether they are less than or equal to
/// Verification likely only need on pivot location.
//////////////////////////////////////////////////////////////////////
template<typename ChunkedCounts, typename Workspace>
void chunkify_linear_recv(ChunkedCounts const& chunked_recv_counts,
                          Workspace& workspace)
{
  const auto nprocs    = get_num_locations();
  const int num_chunks = chunked_recv_counts.size();

  int max_recv_chunk_size = 0;

  for (int chunk = 0; chunk != num_chunks; ++chunk)
  {
    int recv_chunk_size = 0;

    for (size_t j = 0; j != nprocs; ++j)
      recv_chunk_size += chunked_recv_counts[chunk][j];

    max_recv_chunk_size = std::max(max_recv_chunk_size, recv_chunk_size);
  }

  workspace.resize(max_recv_chunk_size);
}


//////////////////////////////////////////////////////////////////////
/// @brief Write the results received for a chunk for the temporary workspace
///   to the local sequence if all elements that were there have been sent.
///   Elements that cannot be written yet are put in a deferred queue.
///
/// @param numbers The local portion of the sequence being exchanged.
/// @param workspace The workspace containing received elements for the
///   current chunk.
/// @param deferred Container to hold elements that cannot be written to
///   @p numbers yet.
/// @param recv_counts
/// @param recv_displs
/// @param recv_counts The receive counts for this chunk iteration.
/// @param recv_displs The offsets in @p workspace for this chunk iteration.
/// @param recv_target_domains For each partition index, the subdomain in
///   in the local storage (i.e., base container) where element received
///   from each processor in this chunk will be written.
/// @param chunk The index of the current chunk iteration.
/// @param num_chunks The total number of chunks in the iterative send schedule.
//////////////////////////////////////////////////////////////////////
template<typename Numbers, typename Workspace, typename Deferred,
         typename Counts, typename Displs, typename Domains>
void linear_store(Numbers& numbers,
                  Workspace& workspace,
                  Deferred& deferred,
                  Counts const& recv_counts,
                  Displs const& recv_displs,
                  Domains const& recv_target_domains,
                  const int chunk,
                  const int num_chunks)
{
  // could replace with compute_chunk_size usage.
  const int div        = numbers.size() / num_chunks;
  const int mod        = numbers.size() % num_chunks;
  const int next_chunk = chunk + 1;

  const int send_available_pos =
    next_chunk*div + (next_chunk <= mod ? next_chunk : mod) - 1;

  using domain_t = indexed_domain<size_t>;

  domain_t write_available_domain(0, send_available_pos);

  deferred.erase(
    std::remove_if(deferred.begin(), deferred.end(),
      [&](typename Deferred::value_type& elem)
      {
        domain_t& domain = get<0>(elem);
        auto dom_size    = domain.size();

        auto intersection = write_available_domain & domain;

        if (intersection.size() == dom_size)
        {
          auto& buffer = get<1>(elem);

          std::copy(buffer->begin(), buffer->end(),
                    numbers.begin() + domain.first());

          return true;
        }
        else
          return false;
      }),
    deferred.end()
  );

  const int recv_counts_size = recv_counts.size();

  // Process Stuff for This Chunk
  for (int recv_proc = 0; recv_proc < recv_counts_size; ++recv_proc)
  {
    if (recv_counts[recv_proc] > 0)
    {
      int displ = recv_displs[recv_proc];

      for (auto&& domain : recv_target_domains[recv_proc])
      {
        const auto dom_size     = domain.size();
        const auto intersection = write_available_domain & domain;
        const auto inter_size   = intersection.size();

        // Direct store part of message that can be written now.
        if (inter_size != 0)
        {
          stapl_assert(intersection.first() == domain.first(),
                       "unexpected intersection in linear store");

          std::copy(workspace.begin() + displ,
                    workspace.begin() + displ + inter_size,
                    numbers.begin() + domain.first());
        }

        // Buffer any remaining part that cannot be written yet.
        if (inter_size != dom_size)
        {
          const auto buf_start =
            inter_size > 0 ? intersection.last() + 1 : domain.first();

          const auto buf_size  = dom_size - inter_size;

          using value_t = typename std::decay<Workspace>::type::value_type;

          auto v = std::make_shared<std::vector<value_t>>(buf_size);

          std::copy(workspace.begin() + displ + inter_size,
                    workspace.begin() + displ + dom_size,
                    v->begin());

          deferred.push_back(
            make_tuple(domain_t(buf_start, domain.last()), std::move(v)));
        }

        displ += dom_size;
      }

      stapl_assert(recv_counts[recv_proc] == (displ - recv_displs[recv_proc]),
                   "subdomain counts != recv_count");
    }
  }
}

#ifdef _STAPL
} // namespace stapl
#endif

#endif



