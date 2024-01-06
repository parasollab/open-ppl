#include "freqmine.hpp"
#include "eclat1d.hpp"

//#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>

#include <stapl/algorithms/sorting.hpp>
#include <climits>

#undef STAPL_TIMER
#if defined(STAPL_TIMER)
#include <stapl/runtime/counter/default_counters.hpp>
#endif

#define SYS_TIMER 1
#if defined(SYS_TIMER)
#include "timing.h"
#endif

#define DBG_TRACE 1
#define DBG_VERT 1
//#define DBG_SRCH 1
#define DETAIL_TIMER 1
//#define DBG_DATA 1

#define USE_UNORD_STL_MAP 1
// turning this on causes some inner data structures to be unordered maps
// instead of ordered maps

///////////////////////////////////////////////////////////////////////////
// copy a STAPL vector of STL pairs to a STAPL vector of STL vectors
///////////////////////////////////////////////////////////////////////////

struct copy_itemset2_vec_wf
{
private:
  stapl::stream<ofstream> m_dbg;
public:
  copy_itemset2_vec_wf(stapl::stream<ofstream> & out)
   : m_dbg(out)
  { }
  typedef void result_type;
  template <typename PairView, typename RepView>
  result_type operator()(PairView proxy1, RepView & proxy2 )
  {
    pair_item_tp item_pair = proxy1;
    vec_item_tp item_vec(2);
    item_vec[0] = item_pair.first;
    item_vec[1] = item_pair.second;
    proxy2.push_back(item_vec);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_dbg);
  }
};

///////////////////////////////////////////////////////////////////////////
// copy a STAPL vector of STL pairs to a STAPL map of encoded pairs
// which map to a unsigned int (TID)
///////////////////////////////////////////////////////////////////////////

struct copy_itemset2_map_wf
{
private:
  stapl::stream<ofstream> m_dbg;
public:
  copy_itemset2_map_wf(stapl::stream<ofstream> & out)
   : m_dbg(out)
  { }
  typedef void result_type;
  template <typename PairView, typename RepView, typename CountView>
  result_type operator()(PairView proxy1, RepView & proxy2, CountView ndx )
  {
    pair_item_tp itempair = proxy1;
    ulong first = itempair.first;
    ulong second = itempair.second;

#ifdef FIXED_MAP_PAIR
#ifdef USE_UNORD_MAP
    phash_duo_ul_vw_tp in_item_pairs_vw = proxy2;
#else
    pmap_duo_ul_vw_tp in_item_pairs_vw = proxy2;
#endif
    duo_ul_tp key = duo<ulong>(first,second);
    in_item_pairs_vw.insert( key, 1+ndx);

#else // FIXED_MAP_PAIR
#ifdef USE_UNORD_MAP
    phash_encpair_ul_vw_tp in_item_pairs_vw = proxy2;
#else
    pmap_encpair_ul_vw_tp in_item_pairs_vw = proxy2;
#endif
    // encode
    encpair_tp enckey = (first << encode_shift ) | second;

    // add 1 to ndx so that map retrieval of non-existent key
    // is NOT confused with retrieval of first (0-th) index
    // must subtract 1 when values are fetched from this map

    in_item_pairs_vw.insert( enckey, 1+ndx);

#endif // FIXED_MAP_PAIR

  }
  void define_type(stapl::typer& t)
  {
    t.member(m_dbg);
  }
};

///////////////////////////////////////////////////////////////////////////
// on one location (native view):
// - process each transaction
//   - process each item
//     - add the transaction id to the TID list of each item pair
//       that has this item as the first value of the pair
///////////////////////////////////////////////////////////////////////////

struct build_vert_flat_db_wf
{
private:
  ulong m_loc_id;
  stapl::stream<ofstream> m_dbg;
public:
  build_vert_flat_db_wf(ulong loc, stapl::stream<ofstream> & out)
   : m_loc_id(loc), m_dbg(out)
  { }

  struct insert_tidset_wf
  {

    typedef void result_type;
    template <typename MapElement, typename NewElement>
    result_type operator()(MapElement& val, NewElement const& new_val) const
    {
#ifdef USE_SET
      set_ul_tp result_tidset;

      set_ul_tp local_tidset = new_val.second;
      set_ul_tp global_tidset = val.second;
      set_union( local_tidset.begin(), local_tidset.end(),
                 global_tidset.begin(), global_tidset.end(),
                 inserter(result_tidset,result_tidset.begin()) );
      val.second = result_tidset;
#else
      vec_ul_tp result_tidset;

      vec_ul_tp local_tidset = new_val.second;
      vec_ul_tp global_tidset = val.second;

      set_union( local_tidset.begin(), local_tidset.end(),
                 global_tidset.begin(), global_tidset.end(),
                 inserter(result_tidset,result_tidset.begin()) );
      val.second = result_tidset;
#endif
    }
  };

  //---------------------------------------------------------------
  // stapl::map_func(build_vert_flat_db_wf(dbg_out),
  //                 stapl::native_view(horz_db_vw),
  //                 stapl::make_repeat_view(vert_db_vw),
  //                 stapl::make_repeat_view(equiv_class),
  //                 stapl::native_view(in_item_pairs_vw),
  //                 stapl::make_repeat_view(in_item_pairs_vw) );
  //---------------------------------------------------------------

  typedef void result_type;
  template <typename SegView, typename RepView1,
            typename RepView2, typename RepView3, typename RepView4>
  result_type operator()(SegView seg, RepView1 vert_db_vw,
                         RepView2 equiv_class,
                         RepView3 global_in_item_pairs_vw,
                         RepView4 local_results_vw)
  {

    // local processing, no communication
    // for the transactions on this location,
    // create lists of transactions associated with each item
  
#if defined(DBG_VERT)
    m_dbg << "ENTER build_vert_flat_db_wf, horz_db_vw.size()[SEGMENT] = " <<
              seg.size() << endl;
#endif

#if defined(STAPL_TIMER)
  counter_t timer;
  timer.reset();
  timer.start();
#endif
#if (!defined(DETAIL_TIMER)) && defined(SYS_TIMER)
  double time2a_start, time2a_end;
  seconds(time2a_start);
#endif

#ifdef USE_UNORD_STL_MAP
    hash_ul_set_ul_tp local_tid_map(6400);
#else
    map_ul_set_ul_tp local_tid_map;
#endif

#ifdef FIXED_MAP_PAIR
    map_duo_ul_tp local_item_pairs;
#else // FIXED_MAP_PAIR
    map_pair_ul_tp local_item_pairs;
#endif

    ulong trans_ctr = 0, lookups = 0;
    ulong trans_id = seg.domain().first();
    typename SegView::iterator seg_it;
    for ( seg_it = seg.begin(); seg_it != seg.end(); ++seg_it ) {
      vec_item_tp trans = *seg_it;

#if defined(DBG_VERT)
      m_dbg << "build_vert_flat_db_wf, trans.size()= " << trans.size() << endl;
      auto trans_it = trans.begin();
      for ( ; trans_it != trans.end(); ++trans_it ) {
        ulong elem = *trans_it;
        m_dbg << elem << ",";
      }
      m_dbg << endl;
#endif

      auto outer_it = trans.begin();
      ulong outer_ctr = 0;
      for ( ; outer_it != trans.end(); ++outer_it ) {
        ulong outer = *outer_it;
        outer_ctr++;
        ulong inner_ctr = 0;
        vec_item_tp::iterator inner_it = trans.begin();
        // triangularization
        for ( ulong i=0; i<outer_ctr; i++ ) {
          inner_it++;
        }

#if defined(DETAIL_TIMER) && defined(SYS_TIMER)
  double time2I_start, time2I_end;
  seconds(time2I_start);
#endif
        for ( ; inner_it != trans.end(); ++inner_it ) {
          ulong inner = *inner_it;
          inner_ctr++;

          ulong lo = outer;
          ulong hi = inner;
          if ( outer > inner ) {
            hi = outer;
            lo = inner;
          }

#ifdef FIXED_MAP_PAIR
          auto try_pair = duo<ulong>(lo,hi);
#else
          auto try_pair = make_pair(lo,hi);
#endif

          // is the pair in the local cache of item pairs?

          ulong local_entry = local_item_pairs[try_pair];

          if ( local_entry > 0 ) {
              // subtract 1 from ndx so that map retrieval of non-existent key
              // is NOT confused with retrieval of first (0-th) index
              ulong ndx = local_entry - 1;
#ifdef USE_SET
              local_tid_map[ndx].insert(trans_id);
#else
              local_tid_map[ndx].push_back(trans_id);
#endif

#if defined(DBG_SRCH)
              m_dbg << "[ " << lo << "," << hi << "] LOCAL at " << ndx << endl;
#endif

          } else {

            // is the pair in the global map of item pairs?

#ifdef FIXED_MAP_PAIR
            ulong global_entry = global_in_item_pairs_vw[try_pair];
#else
            // encode
            encpair_tp enckey = (lo << encode_shift ) | hi;
            ulong global_entry = global_in_item_pairs_vw[enckey];
#endif

            // entry == 0 means not found
            if ( global_entry > 0 ) {
              // subtract 1 from ndx so that map retrieval of non-existent key
              // is NOT confused with retrieval of first (0-th) index
              ulong ndx = global_entry - 1;
#ifdef USE_SET
              local_tid_map[ndx].insert(trans_id);
#else
              local_tid_map[ndx].push_back(trans_id);
#endif

#if defined(DBG_SRCH)
              m_dbg << "[ " << lo << "," << hi << "] GLOBAL at " << ndx << endl;
#endif

              local_item_pairs[try_pair] = global_entry;

            } else {
#if defined(DBG_SRCH)
              m_dbg << "( " << lo << "," << hi << ") NOT FOUND" << endl;
#endif
            }
          }

        }
        lookups += inner_ctr;

#if defined(DETAIL_TIMER) && defined(SYS_TIMER)
  seconds(time2I_end);
  double phase2I_time = time2I_end - time2I_start;
  m_dbg << "Step 3, phase 2a_inner: " << trans_id <<
           "  (" << outer_ctr << "," << inner_ctr << ") ==" <<
            phase2I_time << endl;
#endif

      }

      trans_ctr++;
      trans_id++;
    }
#if defined(DBG_PHASE)
    m_dbg << "transactions " << trans_ctr << " lookups " << lookups << endl;
#endif

    map_ul_set_ul_tp::iterator map_it;

#if defined(STAPL_TIMER)
  double phase2a_time = timer.stop();
  m_dbg << "Step 3, phase 2a: " << phase2a_time << endl;
#endif
#if (!defined(DETAIL_TIMER)) && defined(SYS_TIMER)
  seconds(time2a_end);
  double phase2a_time = time2a_end - time2a_start;
  m_dbg << "Step 3, phase 2a: " << phase2a_time << endl;
#endif

    // - - - - - - - - - - - - - - - - - - - - -

#if defined(DBG_VERT)
    stapl::do_once([&]() {
      m_dbg << "build_vert_flat_db_wf, local_tid_map.size()= " <<
               local_tid_map.size() << endl;
    });
    for ( auto map_it = local_tid_map.begin();
          map_it != local_tid_map.end(); ++map_it )  {

      ulong ndx = map_it->first;
#ifdef USE_SET
      set_ul_tp local_tidset = map_it->second;
#else
      vec_ul_tp local_tidset = map_it->second;
#endif

      stapl::do_once([&]() {
        m_dbg << " [" << local_tidset.size() << "] ";

#ifdef USE_SET
        set_ul_tp::iterator set_it = local_tidset.begin();
#else
        vec_ul_tp::iterator set_it = local_tidset.begin();
#endif

        for ( ; set_it != local_tidset.end(); ++set_it )  {
          m_dbg << *set_it << ",";
        }
        m_dbg << endl;
      });
  }
#endif

    // - - - - - - - - - - - - - - - - - - - - -

#if defined(SYS_TIMER)
  double time2b_start, time2b_end;
  seconds(time2b_start);
#endif

#define USE_LOCAL_MERGE 1

#ifdef USE_LOCAL_MERGE // -------------------------------------------------

    // save the results to be merged later
    local_results_vw[m_loc_id] = local_tid_map;

#else // ------------------------------------------------------------------

    // global processing, all processors exchange data
    // update and insert global_tidset back into map
    
#ifdef USE_UNORD_STL_MAP
    hash_ul_set_ul_tp::iterator local_it = local_tid_map.begin();
#else
    map_ul_set_ul_tp::iterator local_it = local_tid_map.begin();
#endif
    for ( ; local_it != local_tid_map.end(); ++local_it )  {
      ulong itemset_ndx = (*local_it).first;

#ifdef USE_SET
      set_ul_tp local_tidset = (*local_it).second;
#else
      vec_ul_tp local_tidset = (*local_it).second;
#endif

#if defined(DBG_VERT)
      stapl::do_once([&]() {
        m_dbg << "Global insert @  " << itemset_ndx << 
                 " # " << local_tidset.size() << endl;
      });
#endif

      vert_db_vw.insert(itemset_ndx, local_tidset, insert_tidset_wf());

#if defined(DBG_VERT)
      stapl::do_once([&]() {
        m_dbg << "Global insert @@ " << itemset_ndx << 
                 " ##" << vert_db_vw[itemset_ndx].size() << endl;
      });
#endif
    }

#endif // USE_LOCAL_MERGE -------------------------------------------------

#if defined(STAPL_TIMER)
  double phase2b_time = timer.stop();
  m_dbg << "Step 3, phase 2b: " << phase2b_time << endl;
#endif
#if defined(SYS_TIMER)
  seconds(time2b_end);
  double phase2b_time = time2b_end - time2b_start;
  m_dbg << "Step 3, phase 2b: " << phase2b_time << endl;
#endif

  }
  void define_type(stapl::typer& t)
  {
    t.member(m_dbg);
  }
};

///////////////////////////////////////////////////////////////////////////
// merge tidsets for 2 locations that are on the same shared-memory node
///////////////////////////////////////////////////////////////////////////

struct merge_local_wf
{
private:
  ulong m_loc_id;
  ulong m_step;
  stapl::stream<ofstream> m_dbg;
public:
  merge_local_wf(ulong loc_id, ulong step, stapl::stream<ofstream> & out)
   : m_loc_id(loc_id), m_step(step), m_dbg(out)
  { }

  typedef void result_type;
  template < typename View1, typename RepView2 >
  result_type operator()( View1 proxy, RepView2 &all_results_vw )
  {
    if ( 0 == m_loc_id % m_step ) {
#ifdef USE_UNORD_STL_MAP
      hash_ul_set_ul_tp src_tid_map = all_results_vw[ m_loc_id + (m_step >> 1) ];
#else
      map_ul_set_ul_tp src_tid_map = all_results_vw[ m_loc_id + (m_step >> 1) ];
#endif
      auto it_begin = src_tid_map.begin();
      auto it_end = src_tid_map.end();

#ifdef USE_UNORD_STL_MAP
      hash_ul_set_ul_tp dest_tid_map = proxy;
#else
      map_ul_set_ul_tp dest_tid_map = proxy;
#endif
      dest_tid_map.insert( it_begin, it_end );

      for ( auto src_it = src_tid_map.begin(); 
           src_it != src_tid_map.end(); ++src_it ) {

        auto src_key = (*src_it).first;
        set_ul_tp src_tidset = (*src_it).second;
        set_ul_tp dest_tidset = dest_tid_map[src_key];
        set_ul_tp merge_tidset;

        set_union( src_tidset.begin(), src_tidset.end(),
                   dest_tidset.begin(), dest_tidset.end(),
                   inserter(merge_tidset,merge_tidset.begin()) );
        dest_tid_map[src_key] = merge_tidset;
      }
      all_results_vw[ m_loc_id ] = dest_tid_map;
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_loc_id);
    t.member(m_step);
    t.member(m_dbg);
  }
};

///////////////////////////////////////////////////////////////////////////
// compact maps residing in different STAPL locations on the same
// shared memory node into one map in one location
///////////////////////////////////////////////////////////////////////////

struct compact_local_wf
{
private:
  ulong m_loc_id;
  ulong m_step;
  stapl::stream<ofstream> m_dbg;
public:
  compact_local_wf(ulong loc_id, ulong step, stapl::stream<ofstream> & out)
   : m_loc_id(loc_id), m_step(step), m_dbg(out)
  { }

  typedef void result_type;
  template < typename View1, typename RepView2 >
  result_type operator()( View1 proxy, 
                          RepView2 & invert_vw )
  {
    if ( 0 == m_loc_id % m_step ) {

#ifdef USE_UNORD_STL_MAP 
      hash_ul_set_ul_tp local_tid_map = proxy;
#else
      auto local_tid_map = proxy;
#endif
      auto local_it = local_tid_map.begin();
      for ( ; local_it != local_tid_map.end(); ++local_it )  {
        ulong itemset_ndx = (*local_it).first;

#ifdef USE_SET
        set_ul_tp local_tidset = (*local_it).second;
#else
        vec_ul_tp local_tidset = (*local_it).second;
#endif
        invert_vw.add( make_pair(itemset_ndx,local_tidset) );
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_loc_id);
    t.member(m_step);
    t.member(m_dbg);
  }
};

template <typename T>
struct comp_first_wf
{
  typedef T argument_type;
  typedef bool result_type;
  result_type operator() (const T& x, const T& y) const
  {
    return x.first < y.first;
  }
};

///////////////////////////////////////////////////////////////////////////
// for all transaction ID sets distributed to a location,
// merge the TID sets for groups of adjacent identical keys
///////////////////////////////////////////////////////////////////////////

struct merge_invert_wf
{
private:
  stapl::stream<ofstream> m_dbg;
public:
  merge_invert_wf(stapl::stream<ofstream> & out)
   : m_dbg(out)
  { }

  struct merge_tidset_wf
  {

    typedef void result_type;
    template <typename MapElement, typename NewElement>
    result_type operator()(MapElement& val, NewElement const& new_val) const
    {
#ifdef USE_SET
      set_ul_tp result_tidset;

      set_ul_tp local_tidset = new_val.second;
      set_ul_tp global_tidset = val.second;
      set_union( local_tidset.begin(), local_tidset.end(),
                 global_tidset.begin(), global_tidset.end(),
                 inserter(result_tidset,result_tidset.begin()) );
      val.second = result_tidset;
#else
      vec_ul_tp result_tidset;

      vec_ul_tp local_tidset = new_val.second;
      vec_ul_tp global_tidset = val.second;

      set_union( local_tidset.begin(), local_tidset.end(),
                 global_tidset.begin(), global_tidset.end(),
                 inserter(result_tidset,result_tidset.begin()) );
      val.second = result_tidset;
#endif
    }
  };

  typedef void result_type;
  template < typename SegView1, typename RepView2 >
  result_type operator()( SegView1 seg, 
                          RepView2 & vert_db_vw )
  {
    vec_ul_tp * merge_tidset = new vec_ul_tp();
    vec_ul_tp * result_tidset = new vec_ul_tp();
    vec_ul_tp * swap_tidset = nullptr;

    auto seg_it = seg.begin(); 
    ulong prev_key = (*seg_it).first;
    vec_ul_tp prev_tidset = (*seg_it).second;
    seg_it++;

    while ( seg_it != seg.end() ) {
      auto rec = *seg_it;
      ulong curr_key = rec.first;
      vec_ul_tp curr_tidset = rec.second;
      seg_it++;

      set_union( merge_tidset->begin(), merge_tidset->end(),
                 curr_tidset.begin(), curr_tidset.end(),
                 inserter(*result_tidset, result_tidset->begin()) );

      if ( prev_key != curr_key ) {
        vert_db_vw.insert( curr_key, *result_tidset, merge_tidset_wf());
        result_tidset = new vec_ul_tp();
        merge_tidset->clear();
      } else {
        swap_tidset = merge_tidset;
        merge_tidset = result_tidset;
        result_tidset = swap_tidset;
      }

      prev_key = curr_key;
      prev_tidset = curr_tidset;
    }
    set_union( merge_tidset->begin(), merge_tidset->end(),
               prev_tidset.begin(), prev_tidset.end(),
               inserter(*result_tidset, result_tidset->begin()) );
    vert_db_vw.insert( prev_key, *result_tidset, merge_tidset_wf());
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_dbg);
  }
};

///////////////////////////////////////////////////////////////////////////
// merge tidsets for 2 locations that are on different shared-memory nodes
///////////////////////////////////////////////////////////////////////////

struct merge_global_wf
{
private:
  ulong m_loc_id;
  ulong m_step;
  stapl::stream<ofstream> m_dbg;
public:
  merge_global_wf(ulong loc_id, ulong step, stapl::stream<ofstream> & out)
   : m_loc_id(loc_id), m_step(step), m_dbg(out)
  { }

  struct merge_tidset_wf
  {

    typedef void result_type;
    template <typename MapElement, typename NewElement>
    result_type operator()(MapElement& val, NewElement const& new_val) const
    {
#ifdef USE_SET
      set_ul_tp result_tidset;

      set_ul_tp local_tidset = new_val.second;
      set_ul_tp global_tidset = val.second;
      set_union( local_tidset.begin(), local_tidset.end(),
                 global_tidset.begin(), global_tidset.end(),
                 inserter(result_tidset,result_tidset.begin()) );
      val.second = result_tidset;
#else
      vec_ul_tp result_tidset;

      vec_ul_tp local_tidset = new_val.second;
      vec_ul_tp global_tidset = val.second;

      set_union( local_tidset.begin(), local_tidset.end(),
                 global_tidset.begin(), global_tidset.end(),
                 inserter(result_tidset,result_tidset.begin()) );
      val.second = result_tidset;
#endif
    }
  };

  typedef void result_type;
  template < typename View1, typename RepView2 >
  result_type operator()( View1 proxy, 
                          RepView2 & vert_db_vw )
  {

    if ( 0 == m_loc_id % m_step ) {

#ifdef USE_UNORD_STL_MAP 
      hash_ul_set_ul_tp local_tid_map = proxy;
#else
      auto local_tid_map = proxy;
#endif
      auto local_it = local_tid_map.begin();
      for ( ; local_it != local_tid_map.end(); ++local_it )  {
        ulong itemset_ndx = (*local_it).first;

#ifdef USE_SET
        set_ul_tp local_tidset = (*local_it).second;
#else
        vec_ul_tp local_tidset = (*local_it).second;
#endif
        vert_db_vw.insert(itemset_ndx, local_tidset, merge_tidset_wf());

      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_loc_id);
    t.member(m_step);
    t.member(m_dbg);
  }
};

///////////////////////////////////////////////////////////////////////////
// - create sorted vector of equivalence classes of 2-itemsets
// - schedule 2-itemsets over the parallel locations
// - insert 2-itemsets into TID-list map
///////////////////////////////////////////////////////////////////////////

void prepare_vertical( double min_supp, ulong trans_cnt, ulong item_cnt,
                   pvec_vec_item_vw_tp & horz_db_vw,
                   pvec_pair_item_vw_tp & itemset_2_vw,
                   pvec_vec_item_vw_tp & in_itemsets_vw,
#ifdef FIXED_MAP_PAIR
                   phash_duo_ul_vw_tp & in_item_pairs_vw,
#ifdef USE_UNORD_MAP
#else
                   pmap_duo_ul_vw_tp & in_item_pairs_vw,
#endif
#else // FIXED_MAP_PAIR
#ifdef USE_UNORD_MAP
                   phash_encpair_ul_vw_tp & in_item_pairs_vw,
#else
                   pmap_encpair_ul_vw_tp & in_item_pairs_vw,
#endif
#endif // FIXED_MAP_PAIR
                   vector<eqv_cl_tp *> & equiv_class,
                   stapl::stream<ofstream> & dbg_out ) {

#if defined(STAPL_TIMER)
  counter_t timer;
#endif
#if defined(SYS_TIMER)
  double time_start, time_end;
#endif

#if defined(STAPL_TIMER)
  timer.reset();
  timer.start();
#endif
#if defined(SYS_TIMER)
  seconds(time_start);
#endif

  build_equiv_class( itemset_2_vw, equiv_class, dbg_out ); // #2

  std::sort( equiv_class.begin(), equiv_class.end(), eqv_cl_weight_lt );

#if defined(DBG_EQUIV)
  stapl::do_once([&]() {
    dbg_out << "prepare_vertical: finished build_equiv_class_pairs" << endl;
  });

  stapl::do_once([&]() {
    show_equiv_class( itemset_2_vw, equiv_class, dbg_out );
  });
#endif

  schedule_classes( equiv_class, horz_db_vw.get_num_locations(), dbg_out );

  std::sort( equiv_class.begin(), equiv_class.end(), eqv_cl_begin_lt );

#if defined(DBG_EQUIV)
  stapl::do_once([&]() {
    dbg_out << "prepare_vertical: finished schedule_classes" << endl;
  });

  stapl::do_once([&]() {
    show_equiv_class( itemset_2_vw, equiv_class, dbg_out );
  });
#endif

  stapl::map_func( copy_itemset2_vec_wf(dbg_out),
                   itemset_2_vw,
                   stapl::make_repeat_view(in_itemsets_vw) );
  in_itemsets_vw.flush();

#if defined(DBG_DATA)
  stapl::do_once([&]() {
    dbg_out << "copy_itemset2_vec DONE: " << in_itemsets_vw.size() << endl;
    for ( ulong i=0; i< in_itemsets_vw.size(); i++ ) {
      auto item_vec = in_itemsets_vw[i];
      dbg_out << "[" << i << "]: " <<
                 item_vec[0] << " " << item_vec[1] << endl;
    }
  });
#endif

  ulong size = itemset_2_vw.size();

  stapl::map_func( copy_itemset2_map_wf(dbg_out),
                   itemset_2_vw,
                   stapl::make_repeat_view(in_item_pairs_vw),
                   stapl::counting_view<int>(size) );

#if defined(DBG_DATA)
  stapl::do_once([&]() {
    dbg_out << "copy_itemset2_map DONE: " << in_item_pairs_vw.size() << endl;
#ifdef FIXED_MAP_PAIR
    for ( auto map_it = in_item_pairs_vw.begin();
      map_it != in_item_pairs_vw.end(); ++map_it ) {
      auto key = (*map_it).first;
      ulong first = key.first;
      ulong second = key.second;
      ulong count = (*map_it).second;
      dbg_out << first << " " << second << " " << count << endl;
    }
#else
    for ( auto map_it = in_item_pairs_vw.begin();
      map_it != in_item_pairs_vw.end(); ++map_it ) {
      auto enckey = (*map_it).first;
      ulong count = (*map_it).second;
      // decode
      ulong first = (first_mask&enckey)>>encode_shift;
      ulong second = second_mask&enckey;
      dbg_out << first << " " << second << " " << count << endl;
    }
#endif
  });
#endif

#if defined(STAPL_TIMER)
  double phase1_time = timer.stop();
  dbg_out << "Step 3, Phase 1: " << phase1_time << endl;
#endif
#if defined(SYS_TIMER)
  seconds(time_end);
  double phase1_time = time_end - time_start;
  dbg_out << "Step 3, Phase 1: " << phase1_time << endl;
#endif

}

///////////////////////////////////////////////////////////////////////////
// perform transform step
///////////////////////////////////////////////////////////////////////////

void transform_flat( pvec_vec_item_vw_tp & horz_db_vw,
                   pvec_pair_ul_set_ul_vw_tp & invert_vw,

#ifdef USE_UNORD_ARB_MAP
                   phash_ul_set_ul_arb_vw_tp & vert_db_vw,
#else
                   pmap_ul_set_ul_arb_vw_tp & vert_db_vw,
#endif
                   vector<eqv_cl_tp *> & equiv_class,

#ifdef FIXED_MAP_PAIR
                   phash_duo_ul_vw_tp & in_item_pairs_vw,
#ifdef USE_UNORD_MAP
#else
                   pmap_duo_ul_vw_tp & in_item_pairs_vw,
#endif
#else // FIXED_MAP_PAIR
#ifdef USE_UNORD_MAP
                   phash_encpair_ul_vw_tp & in_item_pairs_vw,
#else
                   pmap_encpair_ul_vw_tp & in_item_pairs_vw,
#endif
#endif // FIXED_MAP_PAIR

                   stapl::stream<ofstream> & dbg_out ) {


#if defined(DBG_VERT)
  stapl::do_once([&]() {
    dbg_out << "BEFORE build_vert_flat_db_wf, horz_db_vw.size() = " <<
               horz_db_vw.size() << endl;
  });
#endif

#if defined(STAPL_TIMER)
  counter_t timer;
#endif
#if defined(SYS_TIMER)
  double time_start, time_end;
#endif

#if defined(STAPL_TIMER)
  timer.reset();
  timer.start();
#endif
#if defined(SYS_TIMER)
  seconds(time_start);
#endif

  ulong num_locs = horz_db_vw.get_num_locations();
  ulong loc_id = horz_db_vw.get_location_id();

#ifdef USE_UNORD_STL_MAP
  pvec_hash_ul_set_ul_tp local_results(num_locs);
  pvec_hash_ul_set_ul_vw_tp local_results_vw(local_results);
#else
  pvec_map_ul_set_ul_tp local_results(num_locs);
  pvec_map_ul_set_ul_vw_tp local_results_vw(local_results);
#endif

  stapl::map_func(build_vert_flat_db_wf(loc_id,dbg_out),
                  stapl::native_view(horz_db_vw),
                  stapl::make_repeat_view(vert_db_vw),
                  stapl::make_repeat_view(equiv_class),
                  stapl::make_repeat_view(in_item_pairs_vw),
                  stapl::make_repeat_view(local_results_vw)
                  );

#if defined(STAPL_TIMER)
  double phase2_time = timer.stop();
  dbg_out << "Step 3, Phase 2: " << phase2_time << endl;
#endif
#if defined(SYS_TIMER)
  seconds(time_end);
  double phase2_time = time_end - time_start;
  dbg_out << "Step 3, Phase 2: " << phase2_time << endl;
#endif

#if defined(STAPL_TIMER)
  timer.reset();
  timer.start();
#endif
#if defined(SYS_TIMER)
  seconds(time_start);
#endif

// - - - - - - - - - -

#ifdef USE_LOCAL_MERGE

#if defined(DETAIL_TIMER) || defined(SYS_TIMER)
  double time3a_start, time3a_end;
  seconds(time3a_start);
#endif

  ulong local_locs = (num_locs < 32) ? num_locs : 32;

  // use a binary tree for merging, rather than linear pass
  // same number of merge operations, but less temp storage required
 
  for ( ulong step = 1; step < local_locs; step <<= 1 ) {
    stapl::map_func(merge_local_wf(loc_id, 2*step, dbg_out),
                    local_results_vw, 
                    stapl::make_repeat_view(local_results_vw) );
  }

#if defined(DEBUG)
  stapl::do_once([&]() {
    dbg_out << "TIDSET sizes after local merge" << endl;
    for ( int i=0; i<local_results_vw.size(); i++ ) {
      dbg_out << "[" << i << "]= " << local_results_vw[i].size() << endl;
    }
  });
#endif

#ifdef DBG_DUMP
  stapl::do_once([&]() {
    dbg_out << "#### TIDSET after local merge" << endl;
    for ( int i=0; i<local_results_vw.size(); i++ ) {
      auto proxy = local_results_vw[i];
      hash_ul_vec_ul_tp local_res = proxy;
      auto size = local_res.size();
      dbg_out << "#" << i << "#= " << size << endl;
      auto loc_iter = local_res.begin(); 
      for ( ; loc_iter != local_res.end(); loc_iter++ ) {
        dbg_out << (*loc_iter).first << ": " << endl;
        vec_ul_tp tidset = (*loc_iter).second;
        auto item_iter = tidset.begin();
        int ctr = 0;
        for ( ; item_iter != tidset.end(); item_iter++ ) {
          dbg_out << *item_iter << " ";
          ctr++;
          if ( 0 == ctr%15 ) {
              dbg_out << endl;
          }
        }
        dbg_out << endl;
      }
      dbg_out << endl;
    }
  });
#endif

#ifdef DBG_DUMP
//  pvec_hash_ul_set_ul_tp local_results(num_locs);
  stapl::do_once([&]() {
    dbg_out << "// TIDSET after local merge" << endl;
    ulong count = 0;
    for ( int tab=0; tab<local_results_vw.size(); tab++ ) {
      auto proxy = local_results_vw[tab];
      hash_ul_vec_ul_tp local_res = proxy;
      auto size = local_res.size();
      auto loc_iter = local_res.begin(); 
      count += local_res.size();
    }
    dbg_out << "#define HASH_CNT " << count << ";" << endl;
    dbg_out << "unsigned long pvec_hash_ulset_ul_cnt = HASH_CNT;" << endl;

    dbg_out << "unsigned long pvec_hash_ulset_ul_dir[HASH_CNT][4] = {" << endl;
    ulong base = 0;
    for ( int tab=0; tab<local_results_vw.size(); tab++ ) {
      auto proxy = local_results_vw[tab];
      hash_ul_vec_ul_tp local_res = proxy;
      auto size = local_res.size();
      auto loc_iter = local_res.begin(); 
      for ( ; loc_iter != local_res.end(); loc_iter++ ) {
        ulong key = (*loc_iter).first;
        vec_ul_tp tidset = (*loc_iter).second;
        dbg_out << "{ " << tab << ", " << key << ", " <<
                           base << ", " << tidset.size() << "}," << endl;
        base += tidset.size();
      }
    }
    dbg_out << "};" << endl;

    dbg_out << "unsigned long pvec_hash_ulset_ul_pool[] = {" << endl;
    for ( int tab=0; tab<local_results_vw.size(); tab++ ) {
      auto proxy = local_results_vw[tab];
      hash_ul_vec_ul_tp local_res = proxy;
      auto size = local_res.size();
      auto loc_iter = local_res.begin(); 
      for ( ; loc_iter != local_res.end(); loc_iter++ ) {
        vec_ul_tp tidset = (*loc_iter).second;
        auto item_iter = tidset.begin();
        int ctr = 0;
        for ( ; item_iter != tidset.end(); item_iter++ ) {
          dbg_out << *item_iter << ",";
          ctr++;
          if ( 0 == ctr%15 ) {
              dbg_out << endl;
          }
        }
        dbg_out << endl;
      }
      dbg_out << "// end table " << tab << endl;
    }
    dbg_out << "};" << endl;

  });
#endif

#ifdef DBG_DUMP
//  pvec_hash_ul_set_ul_tp local_results(num_locs);
  unsigned long pvec_hash_ul_set_ul_cnt;
  unsigned long ** pvec_hash_ulset_ul_dir;
  unsigned long *pvec_hash_ulset_ul_pool;
  pvec_hash_ulset_ul_dir = new unsigned long *[pvec_hash_ul_set_ul_cnt];
  for ( int ctr = 0; ctr < pvec_hash_ul_set_ul_cnt; ctr++ ) {
    pvec_hash_ulset_ul_dir[ctr] = new unsigned long[4];
  }

  int tab_id = pvec_hash_ulset_ul_dir[0][0];
  hash_ul_set_ul_tp hash_tab = local_results[tab_id];
  int prev = tab_id;
  for ( int ctr = 0; ctr < pvec_hash_ul_set_ul_cnt; ctr++ ) {
    tab_id = pvec_hash_ulset_ul_dir[ctr][0];
    if( prev != tab_id ) {
      local_results[prev] = hash_tab;
      prev = tab_id;
      hash_tab = local_results[tab_id];
    }
    ulong key = pvec_hash_ulset_ul_dir[ctr][1];
    ulong set_start = pvec_hash_ulset_ul_dir[ctr][2];
    ulong set_len = pvec_hash_ulset_ul_dir[ctr][3];
    
    set_ul_tp tidset;
    for( int ndx = set_start; ndx < set_start + set_len; ndx++ ) {
      tidset.push_back( pvec_hash_ulset_ul_pool[ndx] );
    }
    hash_tab[key] =tidset;
  }
  local_results[prev] = hash_tab;
#endif

// - - - - - - - - - -

#if defined(STAPL_TIMER)
  double phase3a_time = timer.stop();
  dbg_out << "Step 3, phase 3a: " << phase3a_time << endl;
#endif
#if defined(SYS_TIMER)
  seconds(time3a_end);
  double phase3a_time = time3a_end - time3a_start;
  dbg_out << "Step 3, phase 3a: " << phase3a_time << endl;
#endif

#if defined(SYS_TIMER)
  double time3b_start, time3b_end;
  seconds(time3b_start);
#endif

// - - - - - - - - - -

#define SORT_BEFORE_MERGE 1
#ifdef SORT_BEFORE_MERGE

#if defined(DBG_TRACE)
    stapl::do_once([&]() {
      dbg_out << "before compact_local_wf" << endl;
    });
#endif

  stapl::map_func( compact_local_wf(loc_id, local_locs, dbg_out),
                   local_results_vw, 
                   stapl::make_repeat_view(invert_vw) ); 
  invert_vw.flush();
  pvec_pair_ul_set_ul_vw_tp temp_vw(invert_vw.container());

// TIMMIE: P=64 never gets here

#if defined(DBG_TRACE)
    stapl::do_once([&]() {
      dbg_out << "after compact_local_wf" << endl;
    });
#endif

  stapl::sort( temp_vw, comp_first_wf<pair_ul_set_ul_tp>() );

#if defined(DBG_TRACE)
    stapl::do_once([&]() {
      dbg_out << "after sort invert_vw" << endl;
    });
#endif

  stapl::map_func( merge_invert_wf(dbg_out),
                   stapl::native_view(temp_vw), 
                   stapl::make_repeat_view(vert_db_vw) ); 

#if defined(DBG_TRACE)
    stapl::do_once([&]() {
      dbg_out << "after merge_invert_wf" << endl;
    });
#endif

#else // SORT_BEFORE_MERGE

  stapl::map_func( merge_global_wf(loc_id, local_locs, dbg_out),
                   local_results_vw, 
                   stapl::make_repeat_view(vert_db_vw) ); 

#endif // SORT_BEFORE_MERGE

// - - - - - - - - - -

#if defined(STAPL_TIMER)
  double phase3b_time = timer.stop();
  dbg_out << "Step 3, phase 3b: " << phase3b_time << endl;
#endif
#if defined(SYS_TIMER)
  seconds(time3b_end);
  double phase3b_time = time3b_end - time3b_start;
  dbg_out << "Step 3, phase 3b: " << phase3b_time << endl;
#endif

#endif // USE_LOCAL_MERGE

#if defined(STAPL_TIMER)
  double phase3_time = timer.stop();
  dbg_out << "Step 3, Phase 3: " << phase3_time << endl;
#endif
#if defined(SYS_TIMER)
  seconds(time_end);
  double phase3_time = time_end - time_start;
  dbg_out << "Step 3, Phase 3: " << phase3_time << endl;
#endif

}
