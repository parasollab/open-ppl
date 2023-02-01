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

#define DEBUG 1

//#define DBG_TRACE 1
//#define DBG_VERT 1

//#define DBG_DATA 1

///////////////////////////////////////////////////////////////////////////
// are the corresponding inner parts of two nested structures the same?
///////////////////////////////////////////////////////////////////////////
//    result_type result = std::numeric_limits<unsigned long int>::max();
//    if ( elem1.size() == elem2.size() ) {
//        if (  stapl::equal(elem1,elem2) ) {
//            result = stapl::index_of(elem1);
//        }
//    }


struct inner_same_wf
{
private:
  stapl::stream<ofstream> m_dbg;
  ulong m_trans_id;
public:
  inner_same_wf(stapl::stream<ofstream> & dbg)
   :  m_dbg(dbg)
  { }

  typedef ulong result_type;
  template<typename Elem1, typename Elem2>
  result_type operator()(Elem1 elem1, Elem2 elem2)
  {
    //ulong ndx = std::numeric_limits<unsigned long int>::max();
    //for ( int i=0; i<in_itemsets_vw.size(); i++ ) {
    //auto elem1 = in_itemsets_vw[i];
    //for ( int j=0; j<elem1.size(); j++ ) {
    //  if ( elem1[j] != elem2[j] ) {
    //    result = i;
 
#ifdef DEBUG
m_dbg << "inner_same GO " << endl;
#endif
    result_type result = std::numeric_limits<unsigned long int>::max();
    bool same = false;
#ifdef DEBUG
m_dbg << "inner_same A " << endl;
#endif
    if ( elem1.size() == elem2.size() ) {
      same = true;
#ifdef DEBUG
m_dbg << "inner_same B " << endl;
#endif
      for ( int j=0; j<elem1.size(); j++ ) {
        if ( elem1[j] != elem2[j] ) {
#ifdef DEBUG
m_dbg << "inner_same C " << elem1[j] << " " << elem2[j] << endl;
#endif
          same = false;
          break;
        }
#ifdef DEBUG
m_dbg << "inner_same D " << endl;
#endif
      }
#ifdef DEBUG
m_dbg << "inner_same E " << same << endl;
#endif
      if ( same == true ) {
#ifdef DEBUG
m_dbg << "inner_same F " << endl;
#endif
        result = stapl::index_of(elem1);
      }
    }
#if defined(DEBUG) || defined(DBG_VERT)
m_dbg << "inner_same " << elem1.size() << " " << elem2.size() << " " << result << endl;
#endif
    return result;
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

struct build_vert_nest_pairs_wf
{
private:
  stapl::stream<ofstream> m_dbg;
  ulong m_trans_id;
public:
  build_vert_nest_pairs_wf(ulong trans_id, stapl::stream<ofstream> & dbg)
   : m_trans_id(trans_id), m_dbg(dbg)
  { }

  typedef void result_type;
  template <typename Elem, typename RepView1, typename Index,
            typename RepView2, typename RepView3>
  result_type operator()(Elem outer, RepView1 trans_vw, Index ndx,
                         RepView2 in_itemsets_vw, RepView3 local_tid_map_vw)
  {
    pvec_item_vw_tp::iterator inner_it = trans_vw.begin();
    //triangularization
    for ( ulong i=0; i<=ndx; i++ ) {
      inner_it++;
    }
    for ( ; inner_it != trans_vw.end(); ++inner_it ) {
      ulong inner = *inner_it;

      pvec_ul_tp itemset(2);
      pvec_ul_vw_tp itemset_vw(itemset);
      if ( outer <= inner ) {
        itemset_vw[0] = outer;
        itemset_vw[1] = inner;
      } else {
        itemset_vw[1] = outer;
        itemset_vw[0] = inner;
      }
      stapl::rmi_fence();

#if defined(DEBUG) || defined(DBG_VERT)
    m_dbg << "build_vert_nest_pairs before inner_same_wf: " <<
              itemset_vw[0] << " " << itemset_vw[1] << endl;
#endif

#if 1
      auto ndx = stapl::map_reduce( inner_same_wf(m_dbg), stapl::min<ulong>(),
                                    in_itemsets_vw,
                                    stapl::make_repeat_view(itemset_vw) );
#else
      ulong ndx = std::numeric_limits<unsigned long int>::max();
      bool same = false;
      for ( int i=0; i<in_itemsets_vw.size(); i++ ) {
m_dbg << "inner_same A " << endl;
        auto elem1 = in_itemsets_vw[i];
        if ( elem1.size() == itemset_vw.size() ) {
          same = true;
m_dbg << "inner_same B " << endl;

          for ( int j=0; j<elem1.size(); j++ ) {
            if ( elem1[j] != itemset_vw[j] ) {
m_dbg << "inner_same C " << elem1[j] << " " << itemset_vw[j] << endl;
              same = false;
              break;
            }
          }
          if ( same == true ) {
            ndx = i;
            break;
          }
        } else {
m_dbg << "inner_same C " << elem1.size() << " " << itemset_vw.size() << endl;
          same = false;
        }
      }
#endif

#if defined(DBG_VERT)
    m_dbg << "build_vert_nest_pairs T3: " << ndx << endl;
#endif

      if (ndx != std::numeric_limits<unsigned long int>::max()) {
        local_tid_map_vw[ndx].push_back(m_trans_id); // flush below
#if defined(DBG_VERT)
        m_dbg << "find " << outer << "," << inner <<" in [" <<
                 in_itemsets_vw.size() << "] items: " << ndx << endl;
#endif
      } else {
          // nothing
#if defined(DBG_FAIL)
        m_dbg << "find " << outer << "," << inner <<" in [" <<
                 in_itemsets_vw.size() << "] items: NOT FOUND" << endl;
#endif
      }
      local_tid_map_vw[ndx].flush(); // push_back above
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_dbg);
    t.member(m_trans_id);
  }
};

///////////////////////////////////////////////////////////////////////////
// construct vertical database
///////////////////////////////////////////////////////////////////////////

struct build_vert_nest_db_wf
{
private:
  stapl::stream<ofstream> m_dbg;
public:
  build_vert_nest_db_wf(stapl::stream<ofstream> & dbg)
   : m_dbg(dbg)
  { }

  typedef void result_type;
  template <typename SegView, typename RepView1,
            typename RepView2, typename RepView3>
  result_type operator()(SegView seg, RepView1 &vert_db_vw,
                         RepView2 &equiv_class, RepView3 &in_itemsets_vw)
  {

    // local processing, no communication
    // for the transactions on this location,
    // create lists of transactions associated with each item

#if defined(DBG_VERT)
    m_dbg << "ENTER build_vert_nest_db_wf, horz_db_vw.size()[SEGMENT] = " <<
             seg.size() << endl;
#endif

    int seg_ctr = 0;
    pmap_ul_pset_ul_tp local_tid_map;
    pmap_ul_pset_ul_vw_tp local_tid_map_vw(local_tid_map);
    ulong trans_id = seg.domain().first();
    typename SegView::iterator seg_it;
    for ( seg_it = seg.begin(); seg_it != seg.end(); ++seg_it ) {
      auto trans_vw = *seg_it;

#if defined(DEBUG) || defined(DBG_VERT)
    m_dbg << "prior to map_func(build_vert_nest_pairs_wf) " << endl;
#endif

// FAILURE: stapl/containers/iterators/local_accessor.hpp
// stapl_assert(false, "This should not be called");

      stapl::map_func( build_vert_nest_pairs_wf(trans_id, m_dbg),
                       trans_vw, stapl::make_repeat_view(trans_vw),
                       stapl::counting_view<ulong>(trans_vw.size()),
                       stapl::make_repeat_view(in_itemsets_vw),
                       stapl::make_repeat_view(local_tid_map_vw) );
      trans_id++;
      seg_ctr++;
    }

    pmap_ul_pset_ul_vw_tp::iterator map_it;

#if defined(DBG_VERT)
    stapl::do_once([&]() {
      m_dbg << "build_vert_nest_db_wf: finished local " <<
               local_tid_map.size() << endl;
    });

    for ( map_it = local_tid_map.begin();
          map_it != local_tid_map.end(); ++map_it )  {
      ulong itemset_ndx = (*map_it).first;
      auto local_tidset_vw= (*map_it).second;

      stapl::do_once([&]() {
        auto itemset = in_itemsets_vw[itemset_ndx];
        for ( typename decltype(itemset)::iterator vec_it = itemset.begin();
             vec_it != itemset.end(); ++vec_it )  {
          if ( vec_it != itemset.begin() ) {
            m_dbg << ":";
          }
          m_dbg << *vec_it;
        }


        m_dbg << " [" << local_tidset_vw.size() << "] ";
        typename decltype(local_tidset_vw)::iterator set_it =
                          local_tidset_vw.begin();
        for ( ; set_it != local_tidset_vw.end(); ++set_it )  {
          m_dbg << *set_it << ",";
        }
        m_dbg << endl;
      });
    }
#endif

    // global processing, all processors exchange data
    // update and insert global_tidset back into map

    for ( map_it = local_tid_map.begin();
         map_it != local_tid_map.end(); ++map_it )  {
      ulong itemset_ndx = (*map_it).first;
      auto proxy= (*map_it).second;
      auto local_tidset_vw = (*map_it).second;
      auto global_tidset_vw = vert_db_vw[itemset_ndx];

      pvec_ul_tp result_tidset;
      pvec_ul_vw_tp temp1_tidset_vw(result_tidset);
      pvec_ul_vw_tp temp2_tidset_vw(result_tidset);

      // result_tidset is union of local_tidset and global_tidset)
      stapl::map_func( set_copy_wf(),
             local_tidset_vw,
             stapl::make_repeat_view<pvec_ul_vw_tp>(temp1_tidset_vw) );
      stapl::map_func( set_union_wf(),
             global_tidset_vw,
             stapl::make_repeat_view<pvec_ul_vw_tp>(temp2_tidset_vw) );
      pvec_ul_vw_tp result_tidset_vw(result_tidset);

      // SIMULATE: vert_db_vw[itemset_ndx] = result_tidset_vw;
      auto src = result_tidset_vw;
      auto dest = vert_db_vw[itemset_ndx];
      dest.resize(src.size());
      for ( ulong i=0; i<src.size(); i++ ) {
        dest[i] = src[i];
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_dbg);
  }
};

///////////////////////////////////////////////////////////////////////////
// - create sorted vector of equivalence classes of 2-itemsets
// - schedule 2-itemsets over the parallel locations
// - insert 2-itemsets into TID-list map
// - transform the local database into vertical form
///////////////////////////////////////////////////////////////////////////

struct copy_itemset2_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 item_pair, View2 item_vec)
  {
    item_vec[0] = item_pair.first;
    item_vec[1] = item_pair.second;
  }
};

///////////////////////////////////////////////////////////////////////////
// - create sorted vector of equivalence classes of 2-itemsets
// - schedule 2-itemsets over parallel locations
// - insert 2-itemsets into TID-list map
///////////////////////////////////////////////////////////////////////////

void prepare_vertical( double min_supp, ulong trans_cnt, ulong item_cnt,
                       pvec_pvec_item_vw_tp & horz_db_vw,
                       pvec_pair_item_vw_tp & itemset_2_vw,
                       pvec_vec_item_vw_tp & in_itemsets_vw,
                       vector<eqv_cl_tp *> & equiv_class,
                       stapl::stream<ofstream> & dbg_out ) {

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

  build_equiv_class( itemset_2_vw, equiv_class, dbg_out );

#ifdef STAPL_TIMER
  double phase1_time = timer.stop();
  dbg_out << "Step 3, phase 1: " << phase1_time << endl;
#endif
#ifdef SYS_TIMER
  seconds(time_end);
  double phase1_time = time_end - time_start;
  dbg_out << "Step 3, phase 1: " << phase1_time << endl;
#endif

#ifdef STAPL_TIMER
  timer.reset();
  timer.start();
#endif
#ifdef SYS_TIMER
  seconds(time_start);
#endif

#if defined(DBG_DATA)
  stapl::do_once([&]() {
    dbg_out << "prepare_vertical: build_equiv_class finished" << endl;
    for ( ulong i=0; i<itemset_2_vw.size(); i++ ) {
      auto pair = itemset_2_vw[i];
      dbg_out << "[" << i << "] " << pair.first << "," << pair.second << endl;
    }
    dbg_out << endl;
  });
#endif

#if defined(DBG_DATA)
  stapl::do_once([&]() {
    dbg_out << "prepare_vertical: build_equiv_class " << equiv_class.size() << endl;
  });

  stapl::do_once([&]() {
    show_equiv_class( itemset_2_vw, equiv_class, dbg_out );
  });
#endif

  std::sort( equiv_class.begin(), equiv_class.end(), eqv_cl_weight_lt );

  schedule_classes( equiv_class, horz_db_vw.get_num_locations(), dbg_out );

#ifdef STAPL_TIMER
  double phase2_time = timer.stop();
  dbg_out << "Step 3, phase 2: " << phase2_time << endl;
#endif
#ifdef SYS_TIMER
  seconds(time_end);
  double phase2_time = time_end - time_start;
  dbg_out << "Step 3, phase 2: " << phase2_time << endl;
#endif

#ifdef STAPL_TIMER
  timer.reset();
  timer.start();
#endif
#ifdef SYS_TIMER
  seconds(time_start);
#endif

#if defined(DBG_TRACE)
  stapl::do_once([&]() {
    dbg_out << "prepare_vertical: schedule classes finished" << endl;
  });
#endif

  std::sort( equiv_class.begin(), equiv_class.end(), eqv_cl_begin_lt );

#if defined(DBG_DATA)
  stapl::do_once([&]() {
    dbg_out << "prepare_vertical: schedule_classes" << endl;
  });
  stapl::do_once([&]() {
    show_equiv_class( itemset_2_vw, equiv_class, dbg_out );
  });
#endif

  stapl::map_func( resize_vec_wf(),
                   in_itemsets_vw, stapl::make_repeat_view<int>(2) );

  stapl::map_func( copy_itemset2_wf(),
                   itemset_2_vw, in_itemsets_vw );

#ifdef STAPL_TIMER
  double phase3_time = timer.stop();
  dbg_out << "Step 3, phase 3: " << phase3_time << endl;
#endif
#ifdef SYS_TIMER
  seconds(time_end);
  double phase3_time = time_end - time_start;
  dbg_out << "Step 3, phase 3: " << phase3_time << endl;
#endif
}

///////////////////////////////////////////////////////////////////////////
// perform transform step
///////////////////////////////////////////////////////////////////////////

void transform_nest( pvec_pvec_item_vw_tp & horz_db_vw,
                     pmap_ul_pset_ul_vw_tp & vert_db_vw,
                     pvec_pair_item_vw_tp & itemset_2_vw,
                     vector<eqv_cl_tp *> & equiv_class,
                     pvec_vec_item_vw_tp & in_itemsets_vw,
                     stapl::stream<ofstream> & dbg_out ) {

#if defined(DBG_TRACE)
  stapl::do_once([&]() {
    dbg_out << "BEFORE build_vert_nest_db_wf, horz_db_vw.size() = " <<
               horz_db_vw.size() << endl;
  });
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

  stapl::map_func(build_vert_nest_db_wf(dbg_out),
                  stapl::native_view(horz_db_vw),
                  stapl::make_repeat_view(vert_db_vw),
                  stapl::make_repeat_view(equiv_class),
                  stapl::make_repeat_view(in_itemsets_vw)
                  );

#ifdef STAPL_TIMER
  double phase4_time = timer.stop();
  dbg_out << "Step 3, phase 4: " << phase4_time << endl;
#endif
#ifdef SYS_TIMER
  seconds(time_end);
  double phase4_time = time_end - time_start;
  dbg_out << "Step 3, phase 4: " << phase4_time << endl;
#endif

#if defined(DBG_TRACE)
  stapl::do_once([&]() {
    dbg_out << "transform_nest: build vert_db finished" << endl;
  });
#endif

}

