#include "freqmine.hpp"
#include "eclat1d.hpp"

//#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>

#if defined(STAPL_TIMER)
#include <stapl/runtime/counter/default_counters.hpp>
#endif

#define SYS_TIMER 1
#if defined(SYS_TIMER)
#include "timing.h"
#endif

#define DBG_STEP 1

//#define DBG_STACK 1
//#define DBG_ASYNCH 1

///////////////////////////////////////////////////////////////////////////
//  1. identify equivalence classes in local L_2
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
//  vert_db:      native_view<pmap_ul_set_ul_vw_tp>
//      key is index into in_itemsets, values are transaction ids
//  in_itemsets:  native_view<pvec_vec_ul_vw_tp>
//      each vector is a sorted set of item ids
//  out_itemsets: repeat_view<pvec_vec_ul_vw_tp>
///////////////////////////////////////////////////////////////////////////

struct asynchronous_flat_wf
{
private:
  stapl::stream<ofstream> m_dbg;
  double m_trans_cnt;
  double m_min_supp;
public:
  asynchronous_flat_wf( stapl::stream<ofstream> & out, double tc, double ms)
   : m_dbg(out), m_trans_cnt(tc), m_min_supp(ms)
  { }

  typedef void result_type;
  template <typename SegView1, typename RepView1, typename View1>
  result_type operator()(SegView1 vert_db_vw, RepView1 in_itemsets_vw,
                         View1 & out_itemsets_vw)
  {

// ----- local processing - native view

    typedef pair<int,int> stack_entry;
    stack<stack_entry> call_stack;

#if defined(DBG_STACK) || defined(DBG_ASYNCH)
    m_dbg << "asynch_flat pre phase 1: " << vert_db_vw.size() << " " <<
             in_itemsets_vw.size() << endl;
#endif

#if defined(STAPL_TIMER)
  counter_t timer;
#endif
#if defined(DBG_TRACE) && defined(SYS_TIMER)
  double time_start, time_end;
#endif

#if defined(STAPL_TIMER)
  timer.reset();
  timer.start();
#endif
#if defined(DBG_TRACE) && defined(SYS_TIMER)
  seconds(time_start);
#endif

  //  1. compute equivalence classes in local L_2 (within this segment)

    vector<pair_ul_tp> seg_equiv_class;
    build_equiv_class<SegView1,RepView1>
      ( vert_db_vw, in_itemsets_vw, seg_equiv_class, m_dbg ); // #4

#if defined(DBG_STACK) || defined(DBG_ASYNCH)
{
    ulong size = seg_equiv_class.size();
    for ( ulong i = 0; i<size; i++ ) {
      ulong lo = seg_equiv_class[i].first;
      ulong hi = seg_equiv_class[i].second;
      m_dbg << "Eqvcl " << lo << "," << hi << endl;
    }
}
#endif

    // push each equivalence class on the call stack
    ulong size = seg_equiv_class.size();
    for ( ulong i = 0; i<size; i++ ) {
      ulong lo = seg_equiv_class[i].first;
      ulong hi = seg_equiv_class[i].second;
#if defined(DBG_STACK) || defined(DBG_ASYNCH)
    m_dbg << "STACK Push " << lo << "," << hi << endl;
#endif
      call_stack.push( make_pair(lo, hi) );
    }

#ifdef USE_SET
    map_ul_set_ul_tp local_trans_sets;
#else
    map_ul_vec_ul_tp local_trans_sets;
#endif

#ifdef USE_UNORD_ARB_MAP
    size = vert_db_vw.size();
#else
    size = vert_db_vw.end() - vert_db_vw.begin();
#endif

    vec_vec_item_tp local_itemsets(size);

  //  2. for each equivalence class E_2 in local L_2

    // copy the segment transaction ids sets onto local_trans_sets
    // copy the segment item ids sets onto local_itemsets

    ulong ndx = 0;
    auto db_it = vert_db_vw.begin();
    while ( db_it != vert_db_vw.end() ) {
      auto proxy_pair = *db_it++;
      auto link = proxy_pair.first;
#ifdef USE_SET
      set_ul_tp tid_set = proxy_pair.second;
#else
      vec_ul_tp tid_set = proxy_pair.second;
#endif
      local_trans_sets[ndx] = tid_set;
      local_itemsets[ndx] = in_itemsets_vw[link];
      ndx++;
    }

#if defined(DBG_ASYNCH)
    m_dbg << "asynch_flat local_sets BEFORE" << endl;
    for ( ulong i=0; i<local_itemsets.size(); i++ ) {
      auto itemset = local_itemsets[i];
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

#if defined(STAPL_TIMER)
  double phase1_time = timer.stop();
  m_dbg << "Step 4, phase 1-2: " << phase1_time << endl;
#endif
#if defined(DBG_TRACE) && defined(SYS_TIMER)
  seconds(time_end);
  double phase1_time = time_end - time_start;
  m_dbg << "Step 4, phase 1-2: " << phase1_time << endl;
#endif

  //  3. compute_freqent(E_2)

    ulong iter_ctr = 0;
#ifdef USE_SET
    vec_set_ul_tp invoc_trans_sets;
#else
    vec_vec_ul_tp invoc_trans_sets;
#endif
    vec_vec_item_tp invoc_itemsets;
    do {

#if defined(ITERATIONS) && defined(STAPL_TIMER)
  timer.reset();
  timer.start();
#endif
#if defined(DBG_TRACE) && defined(SYS_TIMER)
  seconds(time_start);
#endif

      stack_entry entry = call_stack.top();
      call_stack.pop();
      ulong eqv_class_lo = entry.first;
      ulong eqv_class_hi = entry.second;
      ulong itemset_base = local_itemsets.size();


#if defined(DBG_STACK) || defined(DBG_ASYNCH)
      m_dbg << "STACK pop " << entry.first << ","
                                 << entry.second <<  endl;
#endif

      invoc_trans_sets.clear(); // results for this invocation
      invoc_itemsets.clear();

      if ( eqv_class_lo == eqv_class_hi ) {
        continue; // ignore singletons
      }

  //  4.  for all itemsets I_1 and I_2 in E_k-1

      ulong local_size = local_trans_sets.size();
      assert( eqv_class_lo < local_size && eqv_class_hi < local_size );

      for (ulong left_ctr=eqv_class_lo; left_ctr<eqv_class_hi; left_ctr++ ) {
        for (ulong right_ctr=left_ctr+1; right_ctr<=eqv_class_hi; right_ctr++ ) {

          if ( left_ctr >= local_trans_sets.size() ) {
            m_dbg << "Invalid left_ctr " << left_ctr << " >= " <<
                     local_trans_sets.size() << endl;
            assert(0);
          }
#ifdef USE_SET
          set_ul_tp left_trans_set = local_trans_sets[left_ctr];
#else
          vec_ul_tp left_trans_set = local_trans_sets[left_ctr];
#endif

          assert( 0 < left_trans_set.size() );

          if ( right_ctr >= local_trans_sets.size() ) {
            m_dbg << "Invalid right_ctr " << right_ctr <<  " >= " <<
                     local_trans_sets.size() << endl;
            assert(0);
          }
#ifdef USE_SET
          set_ul_tp right_trans_set = local_trans_sets[right_ctr];
#else
          vec_ul_tp right_trans_set = local_trans_sets[right_ctr];
#endif
          assert( 0 < right_trans_set.size() );

#ifdef USE_SET
          set_ul_tp result_trans_set;
#else
          vec_ul_tp result_trans_set;
#endif

  //  5.    if ( |intersect(left.tidlist,right.tidlist)| > min_supp )

          set_intersection(left_trans_set.begin(), left_trans_set.end(),
                           right_trans_set.begin(), right_trans_set.end(),
                           inserter(result_trans_set,result_trans_set.begin()));

#if defined(DBG_ASYNCH)
          m_dbg << "asynch_flat Intersection" << endl;
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
            m_dbg << "asynch_flat avg supp: " << avg_size << " " <<
                     m_min_supp << endl;
            m_dbg << "U[" << result_trans_set.size() << "]: ";
            for ( ulong i=0; i<result_trans_set.size(); i++ ) {
              m_dbg << result_trans_set[i] << " ";
            }
            m_dbg << endl << endl;
#endif

            invoc_trans_sets.push_back( result_trans_set );

  //  6.      add union(left,right) to L_k
            vec_item_tp new_itemset;
            auto left_itemset = local_itemsets[left_ctr];
            auto right_itemset = local_itemsets[right_ctr];

            ulong prefix = left_itemset.size() - 1;
            new_itemset.insert( new_itemset.begin(),
                                 left_itemset.begin(), left_itemset.end() );

            auto iter = right_itemset.begin();

            new_itemset.insert( new_itemset.end(),
                                 iter+prefix, right_itemset.end() );

#if defined(DBG_ASYNCH)
            m_dbg << "asynch_flat Union to pushback: ";
            for ( ulong i=0; i<new_itemset.size(); i++ ) {
              m_dbg << new_itemset[i] << ":";
            }
            m_dbg << endl;
#endif

            invoc_itemsets.push_back(new_itemset);

          }
        }
      }

#if defined(DBG_ASYNCH)
      m_dbg << "asynch_flat invoc_itemsets: " << endl;
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
        build_equiv_class( invoc_itemsets, invoc_equiv_class );

  //  8.  for each equivalence class E_k in L_k

        for ( ulong i=0; i<invoc_equiv_class.size(); i++ ) {
          pair_ul_tp eqv_cl = invoc_equiv_class[i];
          eqv_cl.first += itemset_base;
          eqv_cl.second += itemset_base;

  //  9.    compute_frequent(E_k)
  //
          call_stack.push( eqv_cl );

#if defined(DBG_STACK) || defined(DBG_ASYNCH)
          m_dbg << "STACK push " << eqv_cl.first << "," <<
                   eqv_cl.second << " [" << call_stack.size() << "]" << endl;
#endif
        }

        // accumulate the results of this invocation for this processor

#if defined(DBG_ASYNCH)
m_dbg << "asynch_flat post phase 9 " << invoc_itemsets.size() << endl;
#endif

        for ( ulong i=0; i<invoc_itemsets.size(); i++ ) {
          auto itemset = invoc_itemsets[i];
#ifdef USE_SET
          set_ul_tp trans_set = invoc_trans_sets[i];
#else
          vec_ul_tp trans_set = invoc_trans_sets[i];
#endif
          ulong key = local_itemsets.size();
          local_itemsets.push_back( itemset  );

          local_trans_sets[key] = trans_set;
        }
      }

#if defined(ITERATIONS) && defined(STAPL_TIMER)
  double phase3_time = timer.stop();
  m_dbg << "Step 4, iteration: " << phase3_time << endl;
#endif
#if defined(ITERATIONS) && defined(SYS_TIMER)
  seconds(time_end);
  double phase3_time = time_end - time_start;
  m_dbg << "Step 4, iteration: " << phase3_time << endl;
#endif

#if defined(DBG_STACK) || defined(DBG_ASYNCH)
      m_dbg << "STACK size " << call_stack.size() << endl;
#endif
    
      iter_ctr++;

    } while ( call_stack.size() > 0 );

#if defined(DBG_STEP)
      m_dbg << "Asynch_flat iterations " << iter_ctr-1 <<
               ", #local_itemsets == " << local_itemsets.size() << endl;
#endif

#if defined(STAPL_TIMER)
  timer.reset();
  timer.start();
#endif
#if defined(DBG_TRACE) && defined(SYS_TIMER)
  seconds(time_start);
#endif

// ----- global processing - repeat view

#if defined(DBG_ASYNCH) 
    m_dbg << "asynch_flat local_sets AFTER" << endl;
    for ( ulong i=0; i<local_itemsets.size(); i++ ) {
      auto itemset = local_itemsets[i];
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

    size = local_itemsets.size();
    for ( ulong i=0; i<local_itemsets.size(); i++ ) {
      auto itemset = local_itemsets[i];
      if ( 2 < itemset.size() ) { // pairs were repeated to all locations
        out_itemsets_vw.container().add( itemset );
      }
    }

#if defined(STAPL_TIMER)
  double phase10_time = timer.stop();
  m_dbg << "Step 4, phase 10: " << phase10_time << endl;
#endif
#if defined(DBG_TRACE) && defined(SYS_TIMER)
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

void asynchronous_flat( double min_supp, ulong trans_cnt, ulong item_cnt,
#ifdef USE_UNORD_ARB_MAP
                        phash_ul_set_ul_arb_vw_tp & vert_db_vw,
#else
                        pmap_ul_set_ul_arb_vw_tp & vert_db_vw,
#endif
                        pvec_vec_item_vw_tp & in_itemsets_vw,
                        pvec_vec_item_vw_tp & out_itemsets_vw,
                        stapl::stream<ofstream> & dbg_out ) {

  double trans_cnt_dbl = (double) trans_cnt;

  pvec_vec_item_tp & out_itemsets_ct = out_itemsets_vw.container();

  stapl::map_func( asynchronous_flat_wf(dbg_out, trans_cnt_dbl, min_supp),
                   stapl::native_view(vert_db_vw),
                   stapl::make_repeat_view(in_itemsets_vw),
                   stapl::make_repeat_view(out_itemsets_vw) );

  out_itemsets_ct.flush();
}
