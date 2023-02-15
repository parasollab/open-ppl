/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_BASE_EXCHANGE_SENDS_HPP

#ifdef _STAPL
namespace stapl {
#else

# include <tuple>

  using std::tuple;
  using std::make_tuple;
  using std::get;
  using std::tie;

  size_t myid;
  size_t nprocs;

  size_t get_affinity(void)
  { return myid; }

  size_t get_location_id(void)
  { return myid; }

  size_t get_num_locations(void)
  { return nprocs; }
#endif


//////////////////////////////////////////////////////////////////////
/// @brief Compute the send counts and input buffer read offsets
///   for a given stream of values defined by the input. This is
///   a helper function used by the other methods once the appropriate
///   write offset, etc has been computed and when stability is required.
///   Assumes balanced data distribution between locations.
///
/// @param send_counts The per location send counts of elements to be
///   sent from this location.  Initialized in this function.
/// @param send_displs The per location send read offset of elements
///   sent from this location.  Initialized in this function.
//// @param write_offset Index in the global sequence where first
///   element is written.
/// @param local_read_pos Beginning local index of elements to be sent.
/// @param local_count The number of elements on this location
///   that need to be sent.
/// @param partition The partition of elements.
//////////////////////////////////////////////////////////////////////
template<typename Partition>
void stable_compute_sends(std::vector<int>& send_counts,
                          std::vector<int>& send_displs,
                          size_t write_offset,
                          size_t local_read_pos,
                          size_t local_count,
                          Partition const& partition)
{
  size_t target_proc = partition.find(write_offset);

  while (local_count > 0)
  {
    size_t target_count =
      std::min(local_count, 1 + partition[target_proc].last() - write_offset);

    if (target_proc >= send_counts.size())
    {
#ifdef _STAPL
      abort("out of bounds detected in stable_compute_sends()");
#else
      std::cout << "out of bounds detected in stable_compute_sends()\n";
      std::abort();
#endif
    }

    // Destination Processor has the Pivot Point
    if (send_counts[target_proc] != 0)
      send_counts[target_proc] += target_count;
    else
    {
      send_counts[target_proc] = target_count;
      send_displs[target_proc] = local_read_pos;
    }

    local_read_pos += target_count;
    local_count    -= target_count;
    write_offset   += target_count;
    ++target_proc;
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute the send and read offset values for a location before
/// the pivot location in a partitioning style exchange.
///
/// @param send_counts The per location send counts of elements to be
///   sent from this location.  Initialized in this function.
/// @param send_displs The per location send read offset of elements
///   sent from this location.  Initialized in this function.
/// @param compute_match_count A functor that will return the number
///   of matches (partition 1) for a given location parameter.
/// @param compute_nmatch_count A functor that will return the number
///   of non-matches (partition 2) for a given location parameter.
/// @param local_offsets The offsets for where elements on this location
///   are destined in the global, stable partitioning exchange.
/// @param global_sums The total number of elements in the global sequence.
//    for each partition.
/// @param local_match_count Number of local elements in first partition.
/// @param local_nmatch_count Number of local elements in second partition.
/// @param pivot_proc The location where partition pivot occurs.
/// @param pivot_nmatch_deficit How many elements from second partition
///   are inbound from other locations.
/// @param b_keep_nmatches Denote whether last partition is of interest to
///   caller.
/// @param b_stable Denotes whether exchange process should maintain
///   the relative order between elements on different locations headed to the
///   same target partition or if this can be relaxed to minimize communication
///   and thus increase performance.
//////////////////////////////////////////////////////////////////////
template<typename MatchCount, typename NMatchCount,
         typename Offsets, typename Sums>
tuple<int, int>
compute_left_proc_counts(std::vector<int>& send_counts,
                         std::vector<int>& send_displs,
                         MatchCount compute_match_count,
                         NMatchCount compute_nmatch_count,
                         Offsets&& local_offsets,
                         Sums&& global_sums,
                         int local_match_count,
                         size_t pivot_proc,
                         int pivot_nmatch_deficit,
                         bool b_keep_nmatches,
                         bool b_stable)
{
  const auto myid         = get_location_id();
  const auto nprocs       = get_num_locations();

  using domain_t    = indexed_domain<size_t>;
  using partition_t = balanced_partition<domain_t>;

  partition_t partition(
    domain_t(0, global_sums.first + global_sums.second - 1));

  //
  // Handle Matches
  //

  // If stability not required, all my matches should stay local.
  if (!b_stable)
  {
    send_counts[myid] = local_match_count;
    send_displs[myid] = 0;
  }
  else
  // For stability, assume balanced distribution.  Start writing at
  // computed global offset for this processor, filling out the remainder
  // of that destination processor's block and then writing block sized
  // chunks on each processor.
  {
    stable_compute_sends(
      send_counts, send_displs, local_offsets[0], 0, local_match_count,
      partition);
  }

  //
  // Handle NonMatches
  //
  if (b_keep_nmatches)
  {
    // Compute A Beginning Processor & Offset Pair where my nonmatch
    // writing should start.
    int target_local_write_pos = 0;
    size_t current_proc        = 0;

    if (!b_stable)
    {
      // Compute number of matches before me.
      for (size_t idx = 0; idx < myid; ++idx)
        target_local_write_pos += compute_nmatch_count(idx);

      current_proc = pivot_proc;

      while (true)
      {
        // Compute how many places for incoming nonmatches on the current
        // processor.  If the pivot prc, then it is the deficit of nmatches
        // if any. If to right of pivot, the total number of matches.
        const int current_proc_matches =
          current_proc == pivot_proc ?
            pivot_nmatch_deficit : compute_match_count(current_proc);

        if (target_local_write_pos >= current_proc_matches)
        {
          target_local_write_pos -= current_proc_matches;
          ++current_proc;
        }
        else
          break;
      }
    }
    else
    {
      const size_t global_write_offset = global_sums.first + local_offsets[1];

      current_proc            = partition.find(global_write_offset);
      target_local_write_pos  =
        global_write_offset - partition[current_proc].first();
    }

    int local_send_count = compute_nmatch_count(myid);
    //
    // Using the computed start destination process and offset
    // to start computing send counts, displacements, and types.
    //
    int local_read_pos = local_match_count;
    while (local_send_count > 0)
    {
      if (current_proc >= nprocs)
      {
#ifdef _STAPL
      abort("error in proc offsetting left");
#else
      std::cout << "error in proc offsetting left\n";
      std::abort();
#endif
      }

      int current_proc_space = 0;

      if (!b_stable)
      {
        current_proc_space =
          current_proc == pivot_proc ?
            pivot_nmatch_deficit : compute_match_count(current_proc);

        current_proc_space -= target_local_write_pos;
      }
      else
      {
        current_proc_space =
          partition[current_proc].size() - target_local_write_pos;
      }

      target_local_write_pos = 0;

      if (current_proc_space > 0)
      {
        int target_count = local_send_count < current_proc_space ?
                             local_send_count : current_proc_space;

        // If matches from above already headed to destination processor,
        // don't change displ, just increment count
        if (send_counts[current_proc] != 0)
        {
          send_counts[current_proc] += target_count;
        }
        else
        {
          send_counts[current_proc] = target_count;
          send_displs[current_proc] = local_read_pos;
        }

        local_read_pos    += target_count;
        local_send_count  -= target_count;
      }

      ++current_proc;
    }
  }

  return make_tuple(0, local_match_count);
} // compute_left_proc_counts(...)


//////////////////////////////////////////////////////////////////////
/// @brief Compute the send and read offset values for a location after
/// the pivot location in a partitioning style exchange.
///
/// @param send_counts The per location send counts of elements to be
///   sent from this location.  Initialized in this function.
/// @param send_displs The per location send read offset of elements
///   sent from this location.  Initialized in this function.
/// @param compute_match_count A functor that will return the number
///   of matches (partition 1) for a given location parameter.
/// @param compute_nmatch_count A functor that will return the number
///   of non-matches (partition 2) for a given location parameter.
/// @param local_offsets The offsets for where elements on this location
///   are destined in the global, stable partitioning exchange.
/// @param global_sums The total number of elements in the global sequence.
//    for each partition.
/// @param local_match_count Number of local elements in first partition.
/// @param local_nmatch_count Number of local elements in second partition.
/// @param pivot_proc The location where partition pivot occurs.
/// @param pivot_nmatch_deficit How many elements from second partition
///   are inbound from other locations.
/// @param target_location_matches Number of partition 1 elements in
///   target sequence on the location.
/// @param b_keep_nmatches Denote whether last partition is of interest to
///   caller.
/// @param b_stable Denotes whether exchange process should maintain
///   the relative order between elements on different locations headed to the
///   same target partition or if this can be relaxed to minimize communication
///   and thus increase performance.
//////////////////////////////////////////////////////////////////////
template<typename MatchCount, typename NMatchCount,
         typename Offsets, typename Sums>
tuple<int, int>
compute_right_proc_counts(std::vector<int>& send_counts,
                          std::vector<int>& send_displs,
                          MatchCount compute_match_count,
                          NMatchCount compute_nmatch_count,
                          Offsets&& local_offsets,
                          Sums&& global_sums,
                          int local_match_count, int local_nmatch_count,
                          size_t pivot_proc, int pivot_nmatch_deficit,
                          bool b_keep_nmatches,
                          bool b_stable)
{
  const auto myid         = get_location_id();
  const auto nprocs       = get_num_locations();

  using domain_t    = indexed_domain<size_t>;
  using partition_t = balanced_partition<domain_t>;

  partition_t partition(
    domain_t(0, global_sums.first + global_sums.second - 1));

  //
  // Handle Matches
  //

  // Compute A Beginning Processor & Offset Pair where my nonmatch
  // writing should start.
  int target_local_write_pos = 0;
  size_t current_proc        = 0;

  if (!b_stable)
  {
    // How many matches are there after the pivot point on processors before me?
    target_local_write_pos = pivot_nmatch_deficit;

    for (size_t idx = pivot_proc+1; idx < myid; ++idx)
      target_local_write_pos += compute_match_count(idx);

    //
    // Compute A Beginning Processor & Offset Pair where my match
    // writing should start.
    //
    while (true)
    {
      const int current_proc_nmatches = compute_nmatch_count(current_proc);

      if (target_local_write_pos >= current_proc_nmatches)
      {
        target_local_write_pos -= current_proc_nmatches;
        ++current_proc;
      }
      else
        break;
    }
  }
  else
  {
    current_proc            = partition.find(local_offsets[0]);
    target_local_write_pos  =
      local_offsets[0] - partition[current_proc].first();
  }

  //
  // Using the computed start destination process and offset
  // to start computing send counts, displacements, and types.
  //
  int local_read_pos    = 0;
  int local_send_count = compute_match_count(myid);

  while (local_send_count > 0)
  {
    if (current_proc >= nprocs)
    {
#ifdef _STAPL
      abort("error in proc offsetting left");
#else
      std::cout << "error in proc offsetting left\n";
      std::abort();
#endif
    }

    int current_proc_space = 0;

    if (!b_stable)
    {
      current_proc_space =
        compute_nmatch_count(current_proc) - target_local_write_pos;
    }
    else
    {
      current_proc_space =
        partition[current_proc].size() - target_local_write_pos;
    }

    target_local_write_pos = 0;
    if (current_proc_space > 0)
    {
      int target_count = local_send_count < current_proc_space ?
                           local_send_count : current_proc_space;

      send_counts[current_proc] = target_count;
      send_displs[current_proc] = local_read_pos;

      local_read_pos    += target_count;
      local_send_count  -= target_count;
    }

    ++current_proc;
  }

  //
  // Handle Nonmatches
  //
  if (b_keep_nmatches)
  {
    // If stability not required, all my nonmatches should stay local.
    if (!b_stable)
    {
      send_counts[myid] = local_nmatch_count;
      send_displs[myid] = local_match_count;
    }
    else
    // For stability, assume balanced distribution.  Start writing at
    // computed global offset for this processor, filling out the remainder
    // of that destination processor's block and then writing block sized
    // chunks on each processor.
    {
      const size_t write_offset   = global_sums.first + local_offsets[1];
      const size_t local_read_pos = local_match_count;
      const size_t local_count    = local_nmatch_count;

      stable_compute_sends(
        send_counts, send_displs, write_offset, local_read_pos, local_count,
        partition);
    }
  }

  return make_tuple(0, b_keep_nmatches ? local_nmatch_count : 0);
} // compute_right_proc_counts(...)


//////////////////////////////////////////////////////////////////////
/// @brief Compute the send and read offset values for the pivot
/// location in a partitioning style exchange.
///
/// @param send_counts The per location send counts of elements to be
///   sent from this location.  Initialized in this function.
/// @param send_displs The per location send read offset of elements
///   sent from this location.  Initialized in this function.
/// @param compute_match_count A functor that will return the number
///   of matches (partition 1) for a given location parameter.
/// @param compute_nmatch_count A functor that will return the number
///   of non-matches (partition 2) for a given location parameter.
/// @param local_offsets The offsets for where elements on this location
///   are destined in the global, stable partitioning exchange.
/// @param global_sums The total number of elements in the global sequence.
//    for each partition.
/// @param local_match_count Number of local elements in first partition.
/// @param local_nmatch_count Number of local elements in second partition.
/// @param pivot_proc The location where partition pivot occurs.
/// @param pivot_nmatch_deficit How many elements from second partition
///   are inbound from other locations.
/// @param target_location_matches Number of partition 1 elements in
///   target sequence on the location.
/// @param b_keep_nmatches Denote whether last partition is of interest to
///   caller.
/// @param b_stable Denotes whether exchange process should maintain
///   the relative order between elements on different locations headed to the
///   same target partition or if this can be relaxed to minimize communication
///   and thus increase performance.
//////////////////////////////////////////////////////////////////////
template<typename MatchCount, typename NMatchCount,
         typename Offsets, typename Sums>
tuple<int, int>
compute_pivot_proc_counts(std::vector<int>& send_counts,
                          std::vector<int>& send_displs,
                          MatchCount compute_match_count,
                          NMatchCount compute_nmatch_count,
                          Offsets&& local_offsets,
                          Sums&& global_sums,
                          int local_match_count, int local_nmatch_count,
                          size_t pivot_proc, int pivot_nmatch_deficit,
                          int target_local_matches,
                          bool b_keep_nmatches,
                          bool b_stable)
{
  const auto myid         = get_location_id();
  const auto nprocs       = get_num_locations();

  using domain_t    = indexed_domain<size_t>;
  using partition_t = balanced_partition<domain_t>;

  partition_t partition(
    domain_t(0, global_sums.first + global_sums.second - 1));

  // Take the sum of elements on pivot processor and subtract the number
  // of matches in target permutation that should be there.
  const int target_local_nmatches =
    compute_match_count(myid) + compute_nmatch_count(myid)
      - target_local_matches;

  const int local_nmatch_overage =
    local_nmatch_count <= target_local_nmatches ?
      0 : local_nmatch_count - target_local_nmatches;

  int offset     = 0;
  int recv_displ = 0;

  if (!b_stable)
  {
    // I'm the pivot processor.  If I have matches, some stay here.  Some may
    // need to moved to a processor on the left (because I have more matches
    // than the pivot processor has in the target partitioning).
    //
    // Deal with Pivot Locally Kept Elements
    //
    // If we exactly match the distribution of elements we should have,
    // setup a bulk copy of all local elements to target buffer
    // (implies current_nmatchs == target_nmatches too)
    const int nmatches_to_keep =
      b_keep_nmatches ? std::min(local_nmatch_count, target_local_nmatches) : 0;

    if (local_nmatch_count == target_local_matches)
    {
      send_counts[myid] = local_match_count + nmatches_to_keep;
      send_displs[myid] = 0;
      recv_displ        = 0;

      // shouldn't matter as no one should send me anything, but can use
      // to assert later.
      offset = local_match_count + nmatches_to_keep;

    }
    else
    {
      // If more matches inbound, keep all of my matches and possibly
      // some of nmatches, but write to right of target buffer, so that
      // all locally kept elements are contiguous.
      if (local_match_count < target_local_matches)
      {
        send_counts[myid] = local_match_count + nmatches_to_keep;
        send_displs[myid] = 0;
        recv_displ        = target_local_matches - local_match_count;

        // set for clarity, incoming elements should start at
        // front of buffer...
        offset            = 0;
      }
      // Excess matches.  More nmatches inbound, unless nmatch copying disabled.
      else
      {
        send_counts[myid] = target_local_matches + nmatches_to_keep;
        send_displs[myid] = local_match_count - target_local_matches;
        recv_displ        = 0;

        // incoming nmatches should be put at end.
        offset            = target_local_matches + nmatches_to_keep;
      }
    }
    //
    //  Deal with Pivot Proc Matches to Move
    //
    size_t current_proc  = 0;
    int local_send_count = pivot_nmatch_deficit;
    int local_read_pos   = 0;

    while (local_send_count > 0)
    {
      if (current_proc >= nprocs)
      {
#ifdef _STAPL
      abort("error in proc offsetting");
#else
        std::cout << "error in proc offsetting\n";
        std::abort();
#endif
      }

      int current_proc_nmatches = compute_nmatch_count(current_proc);

      if (current_proc_nmatches > 0)
      {
        int target_count = (local_send_count < current_proc_nmatches) ?
                              local_send_count : current_proc_nmatches;

        send_counts[current_proc] = target_count;
        send_displs[current_proc] = local_read_pos;

        local_read_pos     += target_count;
        local_send_count  -= target_count;
      }
      ++current_proc;
    }
  }
  else
  {
    if (local_nmatch_count == target_local_matches)
      recv_displ = 0;
    else
    {
      // If more matches inbound, keep all of my matches and possibly
      // some of nmatches, but write to right of target buffer, so that
      // all locally kept elements are contiguous.
      if (local_match_count < target_local_matches)
        recv_displ        = target_local_matches - local_match_count;
      // Excess matches.  More nmatches inbound, unless nmatch copying disabled.
      else
        recv_displ        = 0;
    }

    stable_compute_sends(
      send_counts, send_displs, local_offsets[0], 0, local_match_count,
      partition);
  }
  //
  // Move any excess of non matches to a processor / range right of the pivot
  // with excess space where matches previously resided.
  //
  if (b_keep_nmatches)
  {
    if (!b_stable)
    {
      int local_nmatch_send_count = local_nmatch_overage;
      int local_nmatch_read_pos   = local_match_count + target_local_nmatches;

      int current_proc = nprocs-1;

      while (local_nmatch_send_count > 0)
      {
        if (current_proc <= 0)
        {
#ifdef _STAPL
          abort("error in proc offsetting for nmatch sending");
#else
          std::cout << "error in proc offsetting for nmatch sending\n";
          std::abort();
#endif
        }

        const int current_proc_matches = compute_match_count(current_proc);

        if (current_proc_matches > 0)
        {
          int target_count = (local_nmatch_send_count < current_proc_matches) ?
                                local_nmatch_send_count : current_proc_matches;

          send_counts[current_proc] = target_count;
          send_displs[current_proc] = local_nmatch_read_pos;

          local_nmatch_read_pos     += target_count;
          local_nmatch_send_count  -= target_count;
        }
        --current_proc;
      }
    }
    else
    {
      const size_t write_offset   = global_sums.first + local_offsets[1];
      const size_t local_read_pos = local_match_count;
      const size_t local_count    = local_nmatch_count;

      stable_compute_sends(
        send_counts, send_displs, write_offset, local_read_pos, local_count,
        partition);
    }
  }

  return make_tuple(recv_displ, offset);
} // compute_pivot_proc_counts(...)


//////////////////////////////////////////////////////////////////////
/// @brief Returns the local count of elements in a global sequence
/// as defined by the input parameters.
///
/// @param loc The location of interest.
/// @param nlocs The total number of locations.
/// @param global_sum The total number of elements in the global sequence.
/// @param offsets The offsets for each location's first element in the
///  global sequence.
/// @param element Used as an additional index into the offsets parameter,
///   representing the partition of interest.
//////////////////////////////////////////////////////////////////////
template<typename Offsets>
int compute_local_count(int loc, int nlocs, int global_sum,
                        Offsets const& offsets, int element)
{
  return loc != nlocs-1 ?
    offsets[loc+1][element] - offsets[loc][element]
    : global_sum - offsets[loc][element];
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute per location send counts and read displacements for
///   this location.  Also compute the receive position for sends to
///   myself to aid in optimization during the actual exchange.
///
/// This is part of the preprocessing phase of the exchange algorithm.
///
/// @param counts The number of local elements contributed to each partition.
/// @param offsets The beginning offset in the container of each
///   locations contribution to each partition.
/// @param sums The global, total number of elements in each partition.
/// @param b_stable Denotes whether exchange process should maintain
///   the relative order between elements on different locations headed to the
///   same target partition or if this can be relaxed to minimize communication
///   and thus increase performance.
/// @param b_copy_last Denote whether last partition is of interest to caller.
///   If false, the values of the elements in last partition are not defined.
/// @return The receive position and count for sends to self as well as per
///   location send counts and per location read offsets in the input view.
//////////////////////////////////////////////////////////////////////
template<typename Counts, typename Offsets, typename Sums>
std::tuple<std::tuple<int,int>, std::vector<int>, std::vector<int>>
compute_sends(Counts const& counts,
              Offsets const& offsets,
              Sums const& sums,
              bool b_stable, bool b_copy_last)
{
  const auto myid          = get_location_id();
  const auto nprocs        = get_num_locations();
  const int match_sum      = sums.first;
  const int nmatch_sum     = sums.second;
  const int local_matches  = counts[myid].first;
  const int local_nmatches = counts[myid].second;

  std::vector<int> send_counts(nprocs, 0);
  std::vector<int> send_displs(nprocs, 0);

  using offsets_t = std::array<int, 2>;

  offsets_t local_offsets;
  local_offsets[0] = offsets[myid][0];
  local_offsets[1] = offsets[myid][1];

  auto compute_match_count =
    [nprocs, match_sum, &offsets](int proc)
      { return compute_local_count(proc, nprocs, match_sum, offsets, 0); };

  auto compute_nmatch_count =
    [nprocs, nmatch_sum, &offsets](int proc)
      { return compute_local_count(proc, nprocs, nmatch_sum, offsets, 1); };

  // Compute Pivot Processor
  auto iter = std::partition_point(offsets.begin(), offsets.end(),
                                   [&offsets, match_sum](offsets_t const& val)
                                     { return (val[0] + val[1]) < match_sum; });

  const size_t pivot_proc = std::distance(offsets.begin(), iter) - 1;

  const int target_pivot_proc_matches =
    match_sum - offsets[pivot_proc][0] - offsets[pivot_proc][1];

  // Take the sum of elements on pivot processor and subtract the number
  // of matches in target permutation that should be there.
  const int target_pivot_proc_nmatches =
    compute_match_count(pivot_proc) + compute_nmatch_count(pivot_proc)
      - target_pivot_proc_matches;

  const int current_pivot_proc_nmatches = compute_nmatch_count(pivot_proc);

  // also logically equivalent to *match* overage...
  const int pivot_nmatch_deficit =
    current_pivot_proc_nmatches >= target_pivot_proc_nmatches ?
      0 : target_pivot_proc_nmatches - current_pivot_proc_nmatches;

  tuple<int, int> ret;

  if (myid < pivot_proc)
    ret = compute_left_proc_counts(
      send_counts, send_displs, compute_match_count, compute_nmatch_count,
      local_offsets, sums, local_matches, pivot_proc, pivot_nmatch_deficit,
      b_copy_last, b_stable);
  else
  {
    // If I'm a processor to the right of pivot, write all my matches
    // to the appropriate nmatch "hole" on the left side of the pivot.
    if (myid > pivot_proc)
      ret = compute_right_proc_counts(
        send_counts, send_displs, compute_match_count, compute_nmatch_count,
        local_offsets, sums,
        local_matches, local_nmatches,
        pivot_proc, pivot_nmatch_deficit,
        b_copy_last, b_stable);
    // I'm the Pivot processor.
    else
      ret = compute_pivot_proc_counts(
        send_counts, send_displs, compute_match_count, compute_nmatch_count,
        local_offsets, sums, local_matches, local_nmatches,
        pivot_proc, pivot_nmatch_deficit, target_pivot_proc_matches,
        b_copy_last, b_stable);
  }

  return std::make_tuple(ret, std::move(send_counts), std::move(send_displs));
}


template<typename Counts, typename Offsets, typename Sums,
         typename RecvCounts, typename RecvDomains>
void
compute_senders(Counts const& counts,
                Offsets const& offsets,
                Sums const& sums,
                bool b_stable, bool b_copy_last,
                RecvCounts& chunked_recv_counts,
                RecvDomains& chunked_recv_domains)
{
  using domain_t    = indexed_domain<size_t>;
  using partition_t = balanced_partition<domain_t>;

  partition_t partition(domain_t(0, sums.first + sums.second - 1));

  const auto myid      = get_location_id();
  const auto nprocs    = get_num_locations();
  const int match_sum  = sums.first;
  const int nmatch_sum = sums.second;

  std::vector<int> send_counts(nprocs, 0);
  std::vector<int> send_displs(nprocs, 0);

  using offsets_t = std::array<int, 2>;

  offsets_t local_offsets;
  local_offsets[0] = offsets[myid][0];
  local_offsets[1] = offsets[myid][1];

  auto compute_match_count =
    [nprocs, match_sum, &offsets](int proc)
      { return compute_local_count(proc, nprocs, match_sum, offsets, 0); };

  auto compute_nmatch_count =
    [nprocs, nmatch_sum, &offsets](int proc)
      { return compute_local_count(proc, nprocs, nmatch_sum, offsets, 1); };

  // Compute Pivot Processor
  auto iter = std::partition_point(offsets.begin(), offsets.end(),
                                   [&offsets, match_sum](offsets_t const& val)
                                     { return (val[0] + val[1]) < match_sum; });

  const size_t pivot_proc = std::distance(offsets.begin(), iter) - 1;

    // Source Location, Partition ID, Subdomain in Source, Subdomain in Target
  std::vector<tuple<location_type, size_t, domain_t, domain_t>> senders;

  const int target_pivot_proc_matches =
    match_sum - offsets[pivot_proc][0] - offsets[pivot_proc][1];

  const int target_pivot_proc_nmatches =
    compute_match_count(pivot_proc) + compute_nmatch_count(pivot_proc)
      - target_pivot_proc_matches;

  const bool b_receive_matches  =
    myid < pivot_proc || (myid == pivot_proc && target_pivot_proc_matches > 0);

  const bool b_receive_nmatches =
    b_copy_last
    && (myid > pivot_proc
        || (myid == pivot_proc && target_pivot_proc_nmatches > 0));

  if (b_receive_matches)
  {
    // FIXME should this be bounded by my target match count?
    domain_t my_match_domain = partition[myid];

    for (size_t i=0; i != nprocs; ++i)
    {
      const size_t proc_start = offsets[i][0];
      const size_t proc_stop  = offsets[i][0] + compute_match_count(i) - 1;

      domain_t proc_match_domain(proc_start, proc_stop);

      auto intersection = my_match_domain & proc_match_domain;

      const size_t local_start =
        partition[i].first() + intersection.first() - proc_start;

      const size_t local_end   = local_start + intersection.size() - 1;

      domain_t local_domain(local_start, local_end);

      if (!intersection.empty())
        senders.emplace_back(i, 0, intersection, local_domain);
    }
  }

  if (b_receive_nmatches)
  {
    const size_t my_start =
      myid == pivot_proc ? match_sum : partition[myid].first();

    const size_t my_target_nmatch_count =
      myid == pivot_proc ?
        target_pivot_proc_nmatches : partition[myid].size();

    domain_t my_nmatch_domain(my_start, my_start + my_target_nmatch_count-1);

    for (size_t i=0; i != nprocs; ++i)
    {
      const size_t proc_start = offsets[i][1] + match_sum;
      size_t proc_stop        = proc_start + compute_nmatch_count(i) - 1;

      // The subdomain in the target sequence that sending processor holds
      // for this partition index (i.e., nonmatches).
      domain_t proc_nmatch_domain(proc_start, proc_stop);

      // The intersection of my subdomain of the target elements in the given
      // partition index with that of the sending processor's subdomain.
      auto intersection  = my_nmatch_domain & proc_nmatch_domain;

      size_t local_start = partition[i].first() + compute_match_count(i)
                             + intersection.first() - proc_start;

      size_t local_end   = local_start + intersection.size() - 1;

      // The source subdomain the sending processor will send elements from.
      domain_t local_domain(local_start, local_end);

      if (!intersection.empty())
        senders.emplace_back(i, 1, intersection, local_domain);
    }
  }

  const size_t num_chunks  = chunked_recv_counts.size();

  auto compute_chunk_count =
    [&](location_type loc, size_t chunk)
    {
      size_t total_count = compute_match_count(loc);

      if (b_copy_last)
        total_count += compute_nmatch_count(loc);

      return total_count / num_chunks
       + (chunk < total_count % num_chunks ? 1 : 0);
    };


  // Compute the receive schedules from each sending processor.
  for (auto elem : senders)
  {
    size_t proc_start = partition[get<0>(elem)].first();

    for (size_t chunk = 0; chunk != num_chunks; ++chunk)
    {
      const location_type proc_id = get<0>(elem);
      const size_t partition_id   = get<1>(elem);

      stapl_assert(
        chunked_recv_domains[chunk][proc_id][partition_id].empty(),
        "Found already initialized domain in receive chunking");

      const size_t chunk_size = compute_chunk_count(proc_id, chunk);

      domain_t chunk_domain(proc_start, proc_start + chunk_size - 1);

      domain_t intersection = chunk_domain & get<3>(elem);

      chunked_recv_counts[chunk][proc_id] += intersection.size();

      if (intersection.size() > 0)
      {
        const size_t start_local_idx =
          get<2>(elem).first() + intersection.first() - get<3>(elem).first()
          - partition[myid].first();

        const size_t stop_local_idx = start_local_idx + intersection.size() - 1;

        chunked_recv_domains[chunk][proc_id][partition_id] =
          domain_t(start_local_idx, stop_local_idx);
      }

      proc_start += chunk_size;
    }
  }
}

#ifdef _STAPL

} // namespace stapl
#endif

#endif

