#include "freqmine.hpp"
#include "eclat2d.hpp"

#undef STAPL_TIMER
#ifdef STAPL_TIMER
#include <stapl/runtime/counter/default_counters.hpp>
#endif
#ifdef SYS_TIMER
#include "timing.h"
#include <time.h>
#endif

#define DBG_STEP 1
//#define DBG_ASYNCH 1

///////////////////////////////////////////////////////////////////////////
// copy container sizes as specified
///////////////////////////////////////////////////////////////////////////

struct inner_copy_size_wf
{
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 v1, View2 v2) const
  {
    v1 = v2.size();
  }
};

///////////////////////////////////////////////////////////////////////////
//  1. identify equivalence classes in local L_2
//  2. for each equivalence class E_2 in local L_2
//  2. for each equivalence class E_2 in local L_2
//  3. compute_freqent(E_2)
//
//  compute_frequent(E_k-1)
//  4.  for all itemsets left and right in E_k-1
//  5.    if ( |intersect(left.tidlist,right.tidlist)| > min_supp )
//  6.      add union(left,right) to L_k
//  7.  partition L_k into equivalence classes
//  8.  for each equivalence class E_K in L_k
//  9.    compute_frequent(E_k)
//
///////////////////////////////////////////////////////////////////////////

struct asynchronous_nest_wf
{
private:
  stapl::stream<ofstream> m_dbg;
  double m_trans_cnt;
  double m_min_supp;
public:
  asynchronous_nest_wf( stapl::stream<ofstream> & dbg,
                        double tc, double ms)
   : m_dbg(dbg), m_trans_cnt(tc), m_min_supp(ms)
  { }

  typedef void result_type;
  template <typename SegView1, typename SegView2,
            typename View1, typename View2>
  result_type operator()(SegView1 vert_db_vw, SegView2 in_itemsets_vw,
                         View1 & out_itemsets_vw, View2 & out_counts_vw)
  {
#if defined(DBG_ASYNCH)
    m_dbg << "asynch_nest input: " << vert_db_vw.size() << " " <<
             in_itemsets_vw.size() << endl;
#endif

  // ----- local processing - native view

    typedef pair<int,int> stack_entry;
    stack<stack_entry> call_stack;

#if defined(DBG_ASYNCH)
    m_dbg << "asynch_nest pre phase 1: " << vert_db_vw.size() << " " <<
             in_itemsets_vw.size() << endl;
#endif

#ifdef STAPL_TIMER
  counter_t timer;
#endif
#ifdef SYS_TIMER
  double time_start, time_end;
#endif

#ifdef STAPL_TIMER
  timer.reset();
  timer.start();
#endif
#ifdef SYS_TIMER
  seconds(time_start);
#endif

  //  1. compute equivalence classes in local L_2 (within this segment)

    vector<pair_ul_tp> seg_equiv_class;
    build_equiv_class<SegView1,SegView2>
      ( vert_db_vw, in_itemsets_vw, seg_equiv_class, m_dbg );

    // push each equivalence class on the call stack
    ulong size = seg_equiv_class.size();
    for ( ulong i = 0; i<size; i++ ) {
      ulong lo = seg_equiv_class[i].first;
      ulong hi = seg_equiv_class[i].second;
      call_stack.push( make_pair(lo, hi) );
    }

#if defined(DBG_ASYNCH)
    m_dbg << "asynch_nest Step 1 finished: " << call_stack.size() << endl;
#endif

    pmap_ul_pset_ul_tp local_trans_sets;
    size = in_itemsets_vw.end() - in_itemsets_vw.begin(); // CHECK
    pvec_vec_item_tp local_itemsets(size);

  //  2. for each equivalence class E_2 in local L_2

    // copy the segment transaction ids sets onto local_trans_sets
    // copy the segment item ids sets on local_itemsets

    ulong trans_set_ndx = 0;
    auto db_it = vert_db_vw.begin();
    while ( db_it != vert_db_vw.end() ) {
      auto proxy_pair = *db_it++;
      // SIMULATE: local_trans_sets[trans_set_ndx++] = proxy_pair.second;
      auto src = proxy_pair.second;
      auto dest = local_trans_sets[trans_set_ndx];
      dest.resize(src.size());
      ulong i = 0;
      for ( ; i<src.size(); i++ ) {
        dest[i] = src[i];
      }
      assert( 0 != local_trans_sets[trans_set_ndx].size() );
      trans_set_ndx++;
    }

    pmap_ul_pset_ul_vw_tp local_trans_sets_vw(local_trans_sets);

    // copy the segment item ids sets onto local_itemsets

    ulong itemset_ndx = 0;
    auto item_it = in_itemsets_vw.begin();
    while ( item_it != in_itemsets_vw.end() ) {
      // SIMULATE: local_itemsets[itemset_ndx++] = *item_it++;
      auto src = *item_it++;
      auto dest = local_itemsets[itemset_ndx];
      dest.resize(src.size());
      ulong i = 0;
      for ( ; i<src.size(); i++ ) {
        dest[i] = src[i];
      }
      assert( 0 != local_itemsets[itemset_ndx].size() );
      itemset_ndx++;
    }

    pvec_vec_item_vw_tp local_itemsets_vw(local_itemsets);

#if defined(DBG_ASYNCH)
    m_dbg << "asynch_nest Step 2 finished: " << endl;
#endif

#ifdef STAPL_TIMER
  double phase1_time = timer.stop();
  m_dbg << "Step 4, phase 1-2: " << phase1_time << endl;
#endif
#ifdef SYS_TIMER
  seconds(time_end);
  double phase1_time = time_end - time_start;
  m_dbg << "Step 4, phase 1-2: " << phase1_time << endl;
#endif

  //  3. compute_freqent(E_2)

    vec_vec_ul_tp invoc_trans_sets;
    vec_vec_item_tp invoc_itemsets;
    do {

#ifdef STAPL_TIMER
  timer.reset();
  timer.start();
#endif
#ifdef SYS_TIMER
  seconds(time_start);
#endif

      stack_entry entry = call_stack.top();
      call_stack.pop();
      ulong eqv_class_lo = entry.first;
      ulong eqv_class_hi = entry.second;
      ulong itemset_base = local_itemsets_vw.size();

#if defined(DBG_ASYNCH)
    m_dbg << "STACK pop: " << entry.first << ","
                           << entry.second << endl;
#endif

      invoc_trans_sets.clear(); // results for this invocation
      invoc_itemsets.clear();

      if ( eqv_class_lo == eqv_class_hi ) {
        continue; // ignore singletons
      }

#if defined(DBG_ASYNCH)
      m_dbg << "asynch_nest go "  <<
               " : " << eqv_class_lo << "," << eqv_class_hi << endl;
#endif

  //  4.  for all itemsets I_1 and I_2 in E_k-1

      for (ulong left_ctr=eqv_class_lo; left_ctr<eqv_class_hi; left_ctr++ ) {
        for (ulong right_ctr=left_ctr+1; right_ctr<=eqv_class_hi; right_ctr++ ) {

          auto left_trans_set = local_trans_sets_vw[left_ctr];
          auto right_trans_set = local_trans_sets_vw[right_ctr];
          vec_ul_tp result_trans_set;

#if defined(DBG_ASYNCH)
           m_dbg << "asynch_nest phase 4: " << left_trans_set.size() <<
                    " " << right_trans_set.size() << endl;
#endif

  //  5.    if ( |intersect(left.tidlist,right.tidlist)| > min_supp )

          set_intersection(left_trans_set.begin(), left_trans_set.end(),
                           right_trans_set.begin(), right_trans_set.end(),
                           inserter(result_trans_set,result_trans_set.begin()));

#if defined(DBG_ASYNCH)
          m_dbg << "asynch_nest Intersection" << endl;
          m_dbg << "L[" << left_trans_set.size() << "]: ";
          for ( ulong i=0; i<left_trans_set.size(); i++ ) {
            m_dbg << left_trans_set[i] << " ";
          }
          m_dbg << endl;
          m_dbg << "R[" << right_trans_set.size() << "]: ";
          for ( ulong i=0; i<right_trans_set.size(); i++ ) {
            m_dbg << right_trans_set[i] << " ";
          }
          m_dbg << endl;
          m_dbg << "Z[" << result_trans_set.size() << "]: ";
          for ( ulong i=0; i<result_trans_set.size(); i++ ) {
            m_dbg << result_trans_set[i] << " ";
          }
          m_dbg << endl << endl;
#endif

          double avg_size = ((double) result_trans_set.size()) / m_trans_cnt;
          if ( avg_size >= m_min_supp )  {

#if defined(DBG_ASYNCH)
            m_dbg << "asynch_nest avg supp: " << avg_size << " " <<
                     m_min_supp << endl;
            m_dbg << "U[" << result_trans_set.size() << "]: ";
            for ( ulong i=0; i<result_trans_set.size(); i++ ) {
              m_dbg << result_trans_set[i] << " ";
            }
            m_dbg << endl << endl;
#endif

            invoc_trans_sets.push_back( result_trans_set ); // stl -no flush

  //  6.      add union(left,right) to L_k
            vec_item_tp new_itemset;
            auto left_itemset = local_itemsets_vw[left_ctr];
            auto right_itemset = local_itemsets_vw[right_ctr];

            ulong prefix = left_itemset.size() - 1;
            new_itemset.insert( new_itemset.begin(),
                                 left_itemset.begin(), left_itemset.end() );

            auto iter = right_itemset.begin();
            new_itemset.insert( new_itemset.end(),
                                 iter+prefix, right_itemset.end() );

#if defined(DBG_ASYNCH)
            m_dbg << "asynch_nest Union to pushback: ";
            for ( ulong i=0; i<new_itemset.size(); i++ ) {
              m_dbg << new_itemset[i] << ":";
            }
            m_dbg << endl;
#endif

            invoc_itemsets.push_back(new_itemset); // stl -no flush

#if defined(DBG_ASYNCH)
            m_dbg << "asynch_nest phase 6: " << invoc_itemsets.size() << endl;
#endif
          }
        }
      }

#if defined(DBG_ASYNCH)
      m_dbg << "asynch_nest invoc_sets: " << endl;
      for ( ulong i=0; i<invoc_itemsets.size(); i++ ) {
        auto itemset = invoc_itemsets[i];
        for ( ulong j=0; j<itemset.size(); j++ ) {
          m_dbg << itemset[j] << ":";
        }
        m_dbg << endl;
      }
      m_dbg << endl;
#endif

      if ( invoc_itemsets.size() > 0 ) {

  //  7.  partition L_k into equivalence classes

        vector<pair_ul_tp> invoc_equiv_class;
        build_equiv_class( invoc_itemsets, invoc_equiv_class ); // # ?

#if defined(DBG_ASYNCH)
        m_dbg << "asynch_nest phase 7: " <<
                  invoc_equiv_class.size() << endl;
#endif

  //  8.  for each equivalence class E_k in L_k

        for ( ulong i=0; i<invoc_equiv_class.size(); i++ ) {
          pair_ul_tp eqv_cl = invoc_equiv_class[i];
          eqv_cl.first += itemset_base;
          eqv_cl.second += itemset_base;

  //  9.    compute_frequent(E_k)
 
#if defined(DBG_ASYNCH)
          m_dbg << "STACK push " << eqv_cl.first << ","
                                 << eqv_cl.second << endl;
#endif

          call_stack.push( eqv_cl );
        }

        // accumulate the results of this invocation for this processor

#if defined(DBG_ASYNCH)
m_dbg << "asynch_nest INVOC " << invoc_itemsets.size() << endl;
#endif

        for ( ulong i=0; i<invoc_itemsets.size(); i++ ) {
          vec_item_tp itemset = invoc_itemsets[i];
          vec_ul_tp trans_set = invoc_trans_sets[i];
          ulong key = local_itemsets_vw.size();
          local_itemsets_vw.push_back( itemset ); // stapl - flush needed

          // SIMULATE: local_trans_sets_vw[key] = trans_set;
          vec_ul_tp::iterator item_it = trans_set.begin();
          auto dest = local_trans_sets[key];
          dest.resize(trans_set.size());
          for ( ulong j = 0; j<trans_set.size(); j++ ) {
            dest[j] = trans_set[j];
          }
          assert( 0 != local_trans_sets[key].size() );
          local_itemsets_vw.flush();
        }
      }
      //local_itemsets_vw.flush();

#if defined(DBG_ASYNCH)
      m_dbg << "asynch_nest LOCAL " << local_itemsets_vw.size() << endl;
#endif

#ifdef STAPL_TIMER
  double phase3_time = timer.stop();
  m_dbg << "Step 4, iteration: " << phase3_time << endl;
#endif
#ifdef SYS_TIMER
  seconds(time_end);
  double phase3_time = time_end - time_start;
  m_dbg << "Step 4, iteration: " << phase3_time << endl;
#endif

#if defined(CALL_STACK)
      m_dbg << "STACK size " << call_stack.size() << endl;
#endif

    } while ( call_stack.size() > 0 );

#ifdef STAPL_TIMER
  timer.reset();
  timer.start();
#endif
#ifdef SYS_TIMER
  seconds(time_start);
#endif

  // ----- global processing - repeat view

#if defined(DBG_STEP)||defined(DBG_ASYNCH)
    m_dbg << "asynch_nest local_sets on " << stapl::get_location_id() <<
             endl;
    for ( ulong i=0; i<local_itemsets_vw.size(); i++ ) {
      vec_item_tp itemset = local_itemsets_vw[i];
      for ( ulong j=0; j<itemset.size(); j++ ) {
        m_dbg << itemset[j] << ":";
      }
      m_dbg << "  ";
      auto trans_set = local_trans_sets[i];
      for ( ulong j=0; j<trans_set.size(); j++ ) {
        m_dbg << trans_set[j] << ",";
      }
      m_dbg << endl;
    }
#endif

    // insert local results in global data structure

    ulong num_locs = vert_db_vw.get_num_locations();
    ulong my_loc = vert_db_vw.get_location_id();
    auto loc_itemsets = out_itemsets_vw[my_loc];

    // assemble the sizes of the middle vectors

    ary_sz_tp outer_len(num_locs);
    ary_sz_vw_tp outer_len_vw(outer_len);
    stapl::map_func( inner_copy_size_wf(), outer_len_vw, local_itemsets_vw );

    // resize the elements of the middle vectors

    stapl::map_func( inner_resize_wf(), loc_itemsets, outer_len_vw );

    // assemble the sizes of the inner vectors
  
    ary_2_sz_tp middle_len(outer_len);
    ary_2_sz_vw_tp middle_len_vw(middle_len);

#ifdef AGGREGATE
    stapl::map_func( outer_copy_size_wf(), middle_len_vw, local_itemsets_vw );

    // resize the elements of the inner vectors
  
    //stapl::map_func( outer_resize_wf(), middle_len_vw, middle_len_vw );

    // copy the values of the inner vectors

    for ( ulong i=0; i<local_itemsets_vw.size(); i++ ) {
      vec_ul_tp itemset = local_itemsets_vw[i];
      for ( ulong j=0; j<itemset.size(); j++ ) {
         itemset[j]
      }
    }
#endif

#ifdef STAPL_TIMER
  double phase10_time = timer.stop();
  m_dbg << "Step 4, phase 10: " << phase10_time << endl;
#endif
#ifdef SYS_TIMER
  seconds(time_end);
  double phase10_time = time_end - time_start;
  m_dbg << "Step 4, phase 10: " << phase10_time << endl;
#endif

  }

  void define_type(stapl::typer& t)
  {
    t.member(m_dbg);
    t.member(m_trans_cnt);
    t.member(m_min_supp);
  }
};

///////////////////////////////////////////////////////////////////////////
// perform asynchronous step
///////////////////////////////////////////////////////////////////////////

void asynchronous_nest( double min_supp, ulong trans_cnt, ulong item_cnt,
                        pmap_ul_pset_ul_vw_tp & vert_db_vw,
                        pvec_vec_item_vw_tp & in_itemsets_vw,
                        pvec_pvec_pvec_ul_vw_tp & out_itemsets_vw,
                        pvec_ul_vw_tp & out_set_counts_vw,
                        stapl::stream<ofstream> & dbg_out ) {

  double trans_cnt_dbl = (double) trans_cnt;

  pvec_pvec_pvec_ul_tp & out_itemsets_ct = out_itemsets_vw.container();

  stapl::map_func( asynchronous_nest_wf(dbg_out, trans_cnt_dbl, min_supp),
                   stapl::native_view(vert_db_vw),
                   stapl::native_view(in_itemsets_vw),
                   stapl::make_repeat_view(out_itemsets_vw),
                   stapl::make_repeat_view(out_set_counts_vw) );
  out_itemsets_ct.flush();
}

