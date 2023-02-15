/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */


//////////////////////////////////////////////////////////////////////
/// @file
/// Simple list ranking implemented with RMIs.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

using namespace stapl;

class A
: public p_object
{
public:
  typedef std::size_t size_type;
private:
  struct component
  {
    std::vector<size_type> ranks;
    std::vector<size_type> prevs;
    std::vector<size_type> nexts;
    size_type              current_id;
    size_type              prev;
    size_type              next;
    size_type              rank;

    bool done(size_type round) const
    {
      return !( (prevs[round + 1] == current_id) ||
                (nexts[round + 1] == current_id) ||
                (ranks[round + 1] == invalid_location_id) );
    }
  };

  const size_type        m_nsteps;
  const size_type        m_nsteps_p1;
  std::vector<component> m_comps;
  const unsigned int     m_lid;

public:
  explicit A(const size_type k)
  : m_nsteps(std::ceil(std::log2(double(k * this->get_num_locations())))),
    m_nsteps_p1(m_nsteps + 1),
    m_comps(k),
    m_lid(this->get_location_id())
  {
    for (size_type i = 0; i < m_comps.size(); ++i) {
      m_comps[i].current_id = m_lid * m_comps.size() + i;
      m_comps[i].prev = m_comps[i].current_id - 1;
      m_comps[i].next = m_comps[i].current_id + 1;
    }

    if (m_lid == 0)
      m_comps[0].prev = invalid_location_id;

    if (m_lid == this->get_num_locations() - 1)
      m_comps[m_comps.size() - 1].next = invalid_location_id;

    for (size_type i = 0; i < m_comps.size(); ++i) {
      component& cid = m_comps[i];
      cid.prevs.clear();
      cid.nexts.clear();
      cid.ranks.clear();

      cid.prevs.resize(m_nsteps_p1, invalid_location_id);
      cid.nexts.resize(m_nsteps_p1, invalid_location_id);
      cid.ranks.resize(m_nsteps_p1, invalid_location_id);

      if (cid.prev == invalid_location_id) {
        cid.rank = 0;
        // first component has rank all 0
        std::fill(cid.ranks.begin(), cid.ranks.end(), 0);
      }
      else {
        cid.rank = 1;
        // others start with rank 1
        cid.ranks[0] = 1;
        cid.prevs[0] = cid.prev;
      }

      for (size_type i = 1; i != m_nsteps_p1; ++i) {
        cid.prevs[i] = cid.current_id;
        cid.nexts[i] = cid.current_id;
      }

      if (cid.next != invalid_location_id) {
        cid.nexts[0] = cid.next;
      }
    }

    this->advance_epoch();
  }

  void set_rank_prev(size_type round_prev,
                     size_type rank,
                     size_type cur_prev,
                     size_type prev)
  {
    size_type i_prev = cur_prev % m_comps.size();
    m_comps[i_prev].ranks[round_prev + 1] = rank;
    m_comps[i_prev].prevs[round_prev + 1] = prev;
  }

  void set_next(size_type round_next, size_type cur_next, size_type next)
  {
    size_type i_next = cur_next % m_comps.size();
    m_comps[i_next].nexts[round_next + 1] = next;
  }

  void compute_rank(void)
  {
    for (size_type round = 0; round != m_nsteps; ++round) {
      for (size_type p = 0; p < m_comps.size(); ++p) {
        component& cid = m_comps[p];

        //if next valid, send my rank and prev to next
        if (cid.nexts[round] == invalid_location_id) {
          if ((round + 1 != m_nsteps_p1) &&
              (cid.nexts[round + 1] != invalid_location_id)) {
            for (size_type j = round + 1; j != m_nsteps_p1; ++j)
              cid.nexts[j] = invalid_location_id;
          }
        }
        else {
          size_type loc_prev = cid.nexts[round] / m_comps.size();
          if (loc_prev == m_lid)
            set_rank_prev(round, cid.rank, cid.nexts[round], cid.prevs[round]);
          else
            async_rmi(loc_prev, this->get_rmi_handle(),
                      &A::set_rank_prev,
                      round, cid.rank,  cid.nexts[round], cid.prevs[round]);
        }

        // if prev valid, send my next to prev
        if (cid.prevs[round] == invalid_location_id) {
          if ((round + 1 != m_nsteps_p1) &&
              (cid.prevs[round + 1] != invalid_location_id)) {
            for (size_type i = round + 1; i != m_nsteps_p1; ++i) {
              cid.prevs[i] = invalid_location_id;
              cid.ranks[i] = 0;
            }
          }
        }
        else {
          size_type loc_next = cid.prevs[round] / m_comps.size();
          if (loc_next == m_lid)
            set_next(round, cid.prevs[round], cid.nexts[round]);
          else
            async_rmi(loc_next, this->get_rmi_handle(),
                      &A::set_next,
                      round, cid.prevs[round], cid.nexts[round]);
        }
      }

      // spin unitl the rank, prev, next for next round are all set
      for (size_type i = 0; i < m_comps.size(); ++i) { // for all components
        block_until([this, round, i] { return m_comps[i].done(round); });
      }

      // update the rank
      for (size_type i = 0; i < m_comps.size(); ++i) {
        component& cid = m_comps[i];
        cid.rank += cid.ranks[round + 1];
      }
    }

    rmi_fence(); // wait for list ranking to finish on all locations
  }
};

exit_code stapl_main(int, char*[])
{
  A a(1000);
  a.compute_rank();
#ifndef _TEST_QUIET
  std::cout << get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}
