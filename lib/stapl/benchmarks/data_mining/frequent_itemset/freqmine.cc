#include "freqmine.hpp"
#include <math.h>

#define SYS_TIMER 1
#if defined(SYS_TIMER)
#include "timing.h"
struct timeval tp;
#endif

//#define DBG_EQUIV 1
//#define DBG_ARB_DIST 1

///////////////////////////////////////////////////////////////////////////

void merge_counts(map_pair_ul_tp &src, map_pair_ul_tp &dest)
{
  map_pair_ul_tp::iterator map_it;
  for ( map_it = src.begin(); map_it != src.end(); ++map_it ) {
    pair_ul_tp key = map_it->first;
    ulong arg_cnt = map_it->second;

    ulong temp_cnt = dest[key];
    dest[key] = temp_cnt + arg_cnt;
  }
}

///////////////////////////////////////////////////////////////////////////

void show_itemset_2(stapl::stream<ofstream> & out,
                    vec_pair_item_tp itemset_2 ) {

    out << "Itemset 2: " << itemset_2.size() << endl;
    vec_pair_item_tp::iterator iter;
    for ( iter = itemset_2.begin(); iter != itemset_2.end(); ++iter )  {
      pair<ulong,ulong> pair = *iter;
      out << pair.first << ", " << pair.second << endl;
    }
    out << endl;
}

/*=========================================================================
 * EQUIVALENCE CLASS PROCESSING
 *=========================================================================*/

///////////////////////////////////////////////////////////////////////////
// - create sorted vector of equivalence classes of 2-itemsets
// - schedule 2-itemsets over the parallel locations
// - insert 2-itemsets into TID-list map
// - transform the local database into vertical form
///////////////////////////////////////////////////////////////////////////

bool eqv_cl_weight_lt( eqv_cl_tp const *e1, eqv_cl_tp const *e2 ) {
  return e1->weight < e2->weight;
}

bool eqv_cl_begin_lt( eqv_cl_tp const *e1, eqv_cl_tp const *e2 ) {
  return e1->begin < e2->begin;
}

///////////////////////////////////////////////////////////////////////////
// - partition 2-itemsets into equivalence classes using common prefix
// - assign weight based on the number of items in the class
///////////////////////////////////////////////////////////////////////////

void build_equiv_class( vec_pair_item_tp & itemset_2,
                        vector<eqv_cl_tp *> & equiv_class ) {

#if defined(DBG_EQUIV)
  cerr << "build_equiv_class : freqmine.cc #1" << endl;
#endif

  if ( 0 == itemset_2.size() ) {
    cerr << "build_equiv_class 1 guard zero input" << endl;
    return;
  }

  auto iter= itemset_2.begin();
  ulong prev_ndx = 0, pair_ndx = 0;
  pair_ul_tp prev = *iter++;
  ulong count = 1;
  for ( ; iter != itemset_2.end(); ++iter ) {
    pair_ul_tp pair = *iter;
    pair_ndx++;
    if ( pair.first != prev.first ) {
      eqv_cl_tp *eqv_cl = new eqv_cl_tp;
      eqv_cl->begin = prev_ndx;
      eqv_cl->end = pair_ndx;
      eqv_cl->proc = 2147483647;
      if ( count < MAX_WEIGHT ) {
        double weight_dbl = (double) weights[count];
        eqv_cl->weight = (ulong) pow( weight_dbl, 1.5 );
      } else {
        eqv_cl->weight = weights[MAX_WEIGHT-1];
      }
      equiv_class.push_back(eqv_cl);

      prev_ndx = pair_ndx;
      prev = pair;
      count = 1;
    } else {
      count++;
    }
  }
  eqv_cl_tp *eqv_cl = new eqv_cl_tp;
  eqv_cl->begin = prev_ndx;
  eqv_cl->end = pair_ndx+1;
  eqv_cl->weight = weights[MAX_WEIGHT-1];
  eqv_cl->proc = 0;
  if ( count < MAX_WEIGHT ) {
    eqv_cl->weight = weights[count];
  }
  equiv_class.push_back(eqv_cl);

}

///////////////////////////////////////////////////////////////////////////

struct neq_first_asg_wf
{
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left x, Right y, Result z)
  {
    z = x.first != y.first;
  }
};

struct neq_first_elem_asg_wf
{
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left x, Right y, Result z)
  {
    z = x[0] != y[0];
  }
};

struct neq_weight_asg_wf
{
  typedef void result_type;
  template <typename Left, typename Right, typename Result>
  result_type operator()(Left l_proxy, Right r_proxy, Result z)
  {
    eqv_cl_tp l = l_proxy;
    eqv_cl_tp r = r_proxy;
    z = l.weight != r.weight;
  }
};

typedef stapl::not_equal_to<ulong> ne_ul_wf;

///////////////////////////////////////////////////////////////////////////
// - partition vector itemsets into equivalence classes using common prefix
///////////////////////////////////////////////////////////////////////////

struct build_equiv_wf
{
private:
  stapl::stream<ofstream> m_dbg;
public:
  build_equiv_wf(stapl::stream<ofstream> &out)
    : m_dbg(out)
  { }
  typedef void result_type;
  template<typename Elem1, typename Elem2, typename RepView3>
  result_type operator()(Elem1 begin, Elem2 end, RepView3 equiv_class)
  {
    eqv_cl_tp *eqv_cl = new eqv_cl_tp;
    eqv_cl->begin = begin;
    eqv_cl->end = end;
#if defined(DBG_EQUIV)
    m_dbg << eqv_cl->begin << "," << eqv_cl->end << endl;
#endif
    ulong count = end - begin;
    if ( count < MAX_WEIGHT ) {
      double weight_dbl = (double) weights[count];
      eqv_cl->weight = (ulong) pow( weight_dbl, 1.5 );
    }

    eqv_cl->proc = 2147483647;
    equiv_class.push_back(eqv_cl);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_dbg);
  }
};

void build_equiv_class( pvec_pair_item_vw_tp & itemset_2_vw,
                        vector<eqv_cl_tp *> & equiv_class,
                        stapl::stream<ofstream> & dbg_out

) {

#if defined(DBG_EQUIV)
  dbg_out << "build_equiv_class : freqmine.cc #2" << endl;
#endif

  ulong size = itemset_2_vw.size();
  if ( 0 == size ) {
    cerr << "build_equiv_class guard zero input" << endl;
    return;
  }

  typedef pvec_pair_ul_vw_tp::domain_type arg_dom_tp;
  typedef pvec_ul_vw_tp::domain_type res_dom_tp;
  arg_dom_tp left_dom(0, size-2);
  arg_dom_tp right_dom(1, size-1);
  pvec_pair_item_vw_tp left_vw(itemset_2_vw.container(), left_dom);
  pvec_pair_item_vw_tp right_vw(itemset_2_vw.container(), right_dom);

  res_dom_tp first_dom(1, size-1);
  pvec_ul_tp ctl_brk_first(size);
  pvec_ul_vw_tp ctl_brk_first_vw(ctl_brk_first);
  pvec_ul_vw_tp first_item_vw(ctl_brk_first_vw.container(), first_dom);
  ctl_brk_first_vw[0] = 1;

  stapl::map_func( neq_first_asg_wf(),
                   left_vw, right_vw, first_item_vw );

  res_dom_tp last_dom(0, size-2);
  pvec_ul_tp ctl_brk_last(size);
  pvec_ul_vw_tp ctl_brk_last_vw(ctl_brk_last);

  pvec_ul_vw_tp last_item_vw(ctl_brk_last_vw.container(), last_dom);
  ctl_brk_last_vw[size-1] = 1;
  stapl::map_func( neq_first_asg_wf(),
                   left_vw, right_vw, last_item_vw );

#if defined(DBG_EQUIV)
  stapl::do_once([&]() {
    dbg_out << "control break" << endl;
    pvec_pair_ul_vw_tp::iterator iter= itemset_2_vw.begin();
    for ( int i=0; i< itemset_2_vw.size(); i++ ) {
      pair_ul_tp pair = itemset_2_vw[i];
      dbg_out << ctl_brk_first_vw[i] << ":" << ctl_brk_last_vw[i] << "  "
              << pair.first << "," << pair.second << endl;
    }
    dbg_out << endl;
  });
#endif

  ulong brk_cnt = stapl::map_reduce( id_ul_wf(), add_ul_wf(),
                               ctl_brk_first_vw );
  assert( brk_cnt > 0 );
  pvec_ul_tp zero(size);
  pvec_ul_vw_tp zero_vw(zero);
  stapl::fill( zero_vw, 0 );

  typedef pvec_ul_vw_tp::domain_type pvec_dom_tp;

  pvec_ul_tp step_val(size);
  pvec_ul_vw_tp step_val_vw(step_val);
  pvec_ul_vw_tp short_vw(step_val);

  // collect all the begin values which are on control breaks of the first value

  stapl::iota( step_val_vw, 0 );
  short_vw = stapl::collect_if ( step_val_vw, zero_vw, ctl_brk_first_vw,
                                 ne_ul_wf() );

  // pick just the collected values

  pvec_ul_tp begin_ndx(brk_cnt);
  pvec_ul_vw_tp begin_ndx_vw(begin_ndx);

  pvec_dom_tp prefix_dom(0,brk_cnt-1);
  pvec_ul_vw_tp first_prefix_vw(step_val, prefix_dom);
  assert( short_vw.size() == begin_ndx_vw.size() );
  stapl::copy(short_vw, begin_ndx_vw);

  // collect all the end values which are on control breaks of the first value

  assert( step_val_vw.size() == zero_vw.size() );
  assert( zero_vw.size() == ctl_brk_last_vw.size() );

  stapl::iota( step_val_vw, 0 );
  short_vw = stapl::collect_if ( step_val_vw, zero_vw, ctl_brk_last_vw,
                                 ne_ul_wf() );

  // pick just the collected values

  pvec_ul_tp end_ndx(brk_cnt);
  pvec_ul_vw_tp end_ndx_vw(end_ndx);

  pvec_ul_vw_tp second_prefix_vw(step_val, prefix_dom);
  assert( short_vw.size() == end_ndx_vw.size() );
  stapl::copy(short_vw, end_ndx_vw);

  // loss of results if this is used - investigate
  stapl::map_func( build_equiv_wf(dbg_out),
                   begin_ndx_vw, end_ndx_vw,
                   stapl::make_repeat_view(equiv_class)
                 );

#if defined(DBG_EQUIV)
  stapl::do_once([&]() {
    dbg_out << "Equivalence Classes: " << endl;
    pvec_ul_vw_tp::iterator iter_beg= begin_ndx_vw.begin();
    pvec_ul_vw_tp::iterator iter_end= end_ndx_vw.begin();
    while ( iter_beg != begin_ndx_vw.end() ) {
      eqv_cl_tp *eqv_cl = new eqv_cl_tp;
      eqv_cl->begin = *iter_beg;
      eqv_cl->end = *iter_end;
      dbg_out << eqv_cl->begin << "," << eqv_cl->end << endl;
      iter_beg++;
      iter_end++;
    }
    dbg_out << endl;
  });
#endif

}

///////////////////////////////////////////////////////////////////////////
// - partition vector itemsets into equivalence classes using common prefix
///////////////////////////////////////////////////////////////////////////

void build_equiv_class( vec_vec_item_tp & vec_list,
                        vec_pair_ul_tp & equiv_class ) {

#if defined(DBG_EQUIV)
  cerr << "build_equiv_class : freqmine.cc #3" << endl;
#endif

  ulong prev_ndx = 0, curr_ndx = 0;

  if ( 0 == vec_list.size() ) {
    cerr << "build_equiv_class 3 guard zero input" << endl;
    return;
  }

  auto iter= vec_list.begin();
  vec_item_tp prev = *iter++;
  ulong prefix = prev.size() - 1;
  while ( iter != vec_list.end() ) {
    vec_item_tp curr = *iter++;

    bool ctl_break = false;
    for ( ulong i=0; i<prefix; i++ ) {
      if ( curr[i] != prev[i] ) {
        ctl_break = true;
      }
    }
    if ( ctl_break ) {
      pair_ul_tp eqv_cl;
      eqv_cl.first = prev_ndx;
      eqv_cl.second = curr_ndx;
      equiv_class.push_back(eqv_cl);

      prev_ndx = curr_ndx + 1;
      std::copy( curr.begin(), curr.end(), prev.begin() );
    }
    curr_ndx++;
  }
  pair_ul_tp eqv_cl;
  eqv_cl.first = prev_ndx;
  eqv_cl.second = curr_ndx;
  equiv_class.push_back(eqv_cl);
}

///////////////////////////////////////////////////////////////////////////

void show_equiv_class( vec_pair_item_tp & itemset_2,
                       vector<eqv_cl_tp *> & equiv_class,
                       stapl::stream<ofstream> & dbg_out ) {

  stapl::do_once([&]() {
    dbg_out << "show_equiv_class: " << itemset_2.size() << " " <<
                                      equiv_class.size() << endl;
    for ( size_t i=0; i<equiv_class.size(); i++ ) {
      eqv_cl_tp *ec = equiv_class[i];
      dbg_out << ec->weight << " " << ec->begin << " "
              << ec->end << " " << ec->proc << endl;
    }
    dbg_out << endl;
  });

  for ( size_t i=0; i<equiv_class.size(); i++ ) {
    eqv_cl_tp *ec = equiv_class[i];
    for ( ulong i= ec->begin; i < ec->end; i++ ) {
      pair_ul_tp item = itemset_2[i];
      dbg_out << item.first << ":" << item.second << endl;
    }
    dbg_out << endl;
  }
}

void show_equiv_class( pvec_pair_item_vw_tp & itemset_2,
                       vector<eqv_cl_tp *> & equiv_class,
                       stapl::stream<ofstream> & dbg_out ) {

  stapl::do_once([&]() {
    dbg_out << "show_equiv_class: " << itemset_2.size() << " " <<
                                      equiv_class.size() << endl;
    for ( size_t i=0; i<equiv_class.size(); i++ ) {

      eqv_cl_tp *ec = equiv_class[i];
      if ( ec->weight == weights[MAX_WEIGHT-1] ) {
        dbg_out << ec->begin << "," << ec->end << endl;
      } else {
        dbg_out << ec->weight << " " << ec->begin << ","
                << ec->end << " " << ec->proc << endl;
      }
    }
    dbg_out << endl;
  });

  stapl::do_once([&]() {
    for ( size_t i=0; i<equiv_class.size(); i++ ) {
      eqv_cl_tp *ec = equiv_class[i];
      for ( ulong i= ec->begin; i <= ec->end; i++ ) {
        pair_item_tp item = itemset_2[i];
        dbg_out << item.first << ":" << item.second << endl;
      }
      dbg_out << endl;
    }
  });
}

///////////////////////////////////////////////////////////////////////////
// assign each class to the least loaded processor available
///////////////////////////////////////////////////////////////////////////

void schedule_classes( vector<eqv_cl_tp *> & equiv_class,
                       ulong loc_cnt,
                       stapl::stream<ofstream> & dbg_out ) {

  ulong  proc_load[loc_cnt], proc_class_cnt[loc_cnt];
  for ( ulong i=0; i<loc_cnt; i++ ) {
    proc_load[i] = 0;
    proc_class_cnt[i] = 0;
  }

  // input is sorted in ascending order, so start with the biggest
  int cl_cnt = equiv_class.size();
  for ( int cl_ctr = cl_cnt - 1; cl_ctr >= 0; --cl_ctr ) {

    // find the least loaded processor
    int best_proc = 0;
    ulong min_load = 2147483647;
    int min_ndx = 0;
    for ( ulong i = 0; i < loc_cnt; i++ ) {
      if ( proc_load[i] <= min_load ) {
        min_load = proc_load[i];
        min_ndx = i;
      }
    }
    best_proc = min_ndx;

    // add the next heaviest equivalence class
    ulong weight = equiv_class[cl_ctr]->weight;
    proc_load[best_proc] += weight;
    proc_class_cnt[best_proc] += 1;
    equiv_class[cl_ctr]->proc = best_proc;
  }

#if defined(DBG_EQUIV)
    dbg_out << "PROC LOAD COUNT" << endl;
    for ( ulong i=0; i<loc_cnt; i++ ) {
      dbg_out << "loading " << i << " " << proc_load[i] << " " <<
                 proc_class_cnt[i] << endl;
    }
    dbg_out << endl;
#endif

}

///////////////////////////////////////////////////////////////////////////
// assign each class to the least loaded processor available
///////////////////////////////////////////////////////////////////////////

void schedule_classes( pvec_eqv_cl_vw_tp& eqv_cl_vw,
                       ulong loc_cnt,
                       stapl::stream<ofstream> & dbg_out ) {

#if defined(DBG_EQUIV)
  ulong proc_load[loc_cnt], proc_class_cnt[loc_cnt];
  for ( ulong i=0; i<loc_cnt; i++ ) {
    proc_load[i] = 0;
    proc_class_cnt[i] = 0;
  }
#endif

  ulong size = eqv_cl_vw.size();
  typedef pvec_eqv_cl_vw_tp::domain_type arg_dom_tp;
  typedef pvec_ul_vw_tp::domain_type res_dom_tp;

  arg_dom_tp left_dom(0, size-2);
  arg_dom_tp right_dom(1, size-1);
  pvec_eqv_cl_vw_tp left_vw(eqv_cl_vw.container(), left_dom);
  pvec_eqv_cl_vw_tp right_vw(eqv_cl_vw.container(), right_dom);

  res_dom_tp last_dom(0, size-2);
  pvec_ul_tp ctl_brk_last(size);
  pvec_ul_vw_tp ctl_brk_last_vw(ctl_brk_last);
  pvec_ul_vw_tp last_item_vw(ctl_brk_last_vw.container(), last_dom);
  ctl_brk_last_vw[size-1] = 1;
  stapl::map_func( neq_weight_asg_wf(),
                   left_vw, right_vw, last_item_vw );

  ulong brk_cnt = stapl::map_reduce( id_ul_wf(), add_ul_wf(),
                               ctl_brk_last_vw );
  assert( brk_cnt > 0 );
  pvec_ul_tp zero(size);
  pvec_ul_vw_tp zero_vw(zero);
  stapl::fill( zero_vw, 0 );

  typedef pvec_ul_vw_tp::domain_type pvec_dom_tp;

  pvec_ul_tp step_val(size);
  pvec_ul_vw_tp step_val_vw(step_val);
  pvec_ul_vw_tp short_vw(step_val);

  // collect all the end values which are on control breaks

  assert( step_val_vw.size() == zero_vw.size() );
  assert( zero_vw.size() == ctl_brk_last_vw.size() );

  stapl::iota( step_val_vw, 0 );
  short_vw = stapl::collect_if ( step_val_vw, zero_vw, ctl_brk_last_vw,
                                 ne_ul_wf() );

  // pick just the collected values

  pvec_ul_tp end_ndx(brk_cnt);
  pvec_ul_vw_tp end_ndx_vw(end_ndx);
  assert( short_vw.size() == end_ndx_vw.size() );
  stapl::copy(short_vw, end_ndx_vw);

#if defined(DBG_EQUIV)
    dbg_out << "## PROC ## LOAD" << endl;
    for ( ulong i=0; i<loc_cnt; i++ ) {
      dbg_out << "loading " << i << " " << proc_load[i] << " " <<
                 proc_class_cnt[i] << endl;
    }
    dbg_out << endl;
#endif

}

/*=========================================================================
 * ARBITRARY DISTRIBUTION PROCESSING
 *=========================================================================*/

///////////////////////////////////////////////////////////////////////////
// ensure that the distribution of the vertical data base
// cleanly separates the equivalence classes
// this method is a set of heuristics.
// it will fail if the input is indivisible, or if
// the number of possible divisions is less than the number of locations
///////////////////////////////////////////////////////////////////////////

ulong gen_arb_dist( pvec_vec_item_vw_tp & in_itemsets_vw,
                   stapl::array<stapl::arbitrary_partition_info> & part_info,
                   stapl::stream<ofstream> & dbg_out ) {

  ulong size = in_itemsets_vw.size();
  ulong num_locs = in_itemsets_vw.get_num_locations();
  ulong split = size / num_locs;

#if defined(DBG_ARB_DIST)
  stapl::do_once([&]() {
    dbg_out << "gen_arb_dist_flat: " << split << " " <<
               in_itemsets_vw.size() << " " << num_locs<< endl;
  });
#endif

  ulong candidates = 0;
  bool bad_dist = false;
  do {
    bad_dist = false;

    ulong loc = 0;
    ulong begin = 0;
    ulong end = split - 1; // 0-origin

    while ( end < size ) {
      vec_item_tp curr = in_itemsets_vw[end-1];
      vec_item_tp next = in_itemsets_vw[end];

      // find the next control break
      while ( curr[0] == next[0] ) {
        ++end;
        curr = in_itemsets_vw[end-1];
        next = in_itemsets_vw[end];
      }
      if ( loc == num_locs - 1 ) {
        end = size - 1;
      } else {
        --end;
      }

#if defined(DBG_ARB_DIST)
      stapl::do_once([&]() {
        dbg_out << "control break: " <<
                   end   << ": " << curr[0] << "," << curr[1] << " __ " <<
                   end+1 << ": " << next[0] << "," << next[1] << endl;
        dbg_out << "part_info[" << loc << "]= arb_part_info( " <<
                   begin << "," << end << "," << loc << ")" << endl;
      });
#endif

      part_info[loc] = stapl::arbitrary_partition_info( begin, end, loc );

      loc++;
      begin = end+1;
      end += split;
    }

    // did every location get an assignment?
    if ( loc != num_locs ) {
      bad_dist = true;
    } else {
      for ( ulong i=0; i<part_info.size(); i++ ) {
        stapl::arbitrary_partition_info info = part_info[i];
        pair<ulong,ulong> dom_pair = info.domain();
        ulong begin = dom_pair.first;
        ulong end = dom_pair.second;

        if ( !( begin <= size && end <= size) ) {
          bad_dist = true;
        }
      }
    }

    // in case we need to try again
    split *= 0.9;
    candidates++;

  } while ( bad_dist && candidates < num_locs );

#if defined(DBG_ARB_DIST)
  stapl::do_once([&]() {
    dbg_out << "gen_arb_dist_flat  after: " << split << " " <<
               in_itemsets_vw.size() << " " << num_locs<< endl;
    dbg_out << "Partition Bounds: " << part_info.size() << endl;
    for ( ulong i=0; i<part_info.size(); i++ ) {
      stapl::arbitrary_partition_info info = part_info[i];
      pair<ulong,ulong> dom_pair = info.domain();
      ulong begin = dom_pair.first;
      ulong end = dom_pair.second;

      dbg_out << "[" << i << "] " << begin << "," << end << endl;
    }
    dbg_out << endl;
  });
#endif

  // did we succeed?

  ulong part_cnt = num_locs;
  ulong in_size = in_itemsets_vw.size();
  ulong max_lo = 0, max_hi = 0;
  bool first = true;
  for ( ulong i=0; i<part_info.size(); i++ ) {
    stapl::arbitrary_partition_info info = part_info[i];
    pair<ulong,ulong> dom_pair = info.domain();
    ulong begin = dom_pair.first;
    ulong end = dom_pair.second;

    if ( !( begin <= in_size && end <= in_size) ) {
      stapl::do_once([&]() {
        dbg_out << "INVALID PARTITION [" << i << "]  " <<
                   begin << " " << end << " " << in_size << endl;
      });
      --part_cnt;
    }
    if ( !first ) {
      if ( max_lo >= begin || max_hi >= end ) {
        stapl::do_once([&]() {
          dbg_out << "OVERLAP PARTITION [" << i << "] " <<
                     begin << " " << end << endl;
        });
        --part_cnt;
      }
    } else {
      first = false;
    }
    max_lo = begin;
    max_hi = end;
  }

  stapl::rmi_fence();
  return part_cnt;
}

///////////////////////////////////////////////////////////////////////////

ulong count_control_breaks( pvec_vec_item_vw_tp & itemsets_vw,
                        stapl::stream<ofstream> & dbg_out ) {

  ulong size = itemsets_vw.size();

  typedef pvec_vec_ul_vw_tp::domain_type arg_dom_tp;
  typedef pvec_ul_vw_tp::domain_type res_dom_tp;
  arg_dom_tp left_dom(0, size-2);
  arg_dom_tp right_dom(1, size-1);
  pvec_vec_item_vw_tp left_vw(itemsets_vw.container(), left_dom);
  pvec_vec_item_vw_tp right_vw(itemsets_vw.container(), right_dom);

  res_dom_tp first_dom(1, size-1);
  pvec_ul_tp ctl_brk_first(size);
  pvec_ul_vw_tp ctl_brk_first_vw(ctl_brk_first);

  pvec_ul_vw_tp first_item_vw(ctl_brk_first_vw.container(), first_dom);
  ctl_brk_first_vw[0] = 1;
  stapl::map_func( neq_first_elem_asg_wf(),
                   left_vw, right_vw, first_item_vw );

  res_dom_tp last_dom(0, size-2);
  pvec_ul_tp ctl_brk_last(size);
  pvec_ul_vw_tp ctl_brk_last_vw(ctl_brk_last);

  pvec_ul_vw_tp last_item_vw(ctl_brk_last_vw.container(), last_dom);
  ctl_brk_last_vw[size-1] = 1;
  stapl::map_func( neq_first_elem_asg_wf(),
                   left_vw, right_vw, last_item_vw );

#if defined(DBG_EQUIV)
  stapl::do_once([&]() {
    dbg_out << "control break" << endl;
    pvec_vec_ul_vw_tp::iterator iter= itemsets_vw.begin();
    for ( int i=0; i< itemsets_vw.size(); i++ ) {
      vec_ul_tp vec = itemsets_vw[i];
      dbg_out << ctl_brk_first_vw[i] << ":" << ctl_brk_last_vw[i] << "  "
              << vec[0] << "," << vec[1] << endl;
    }
    dbg_out << endl;
  });
#endif

  ulong brk_cnt = stapl::map_reduce( id_ul_wf(), add_ul_wf(),
                               ctl_brk_first_vw );

  return brk_cnt;
}
