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

//#define DBG_PHASE 1
//#define DBG_DATA 1
//#define DBG_ASYNCH 1
//#define DBG_COUNT 1

///////////////////////////////////////////////////////////////////////////
// perform the ECLAT algorithm with parallelism depth=1
///////////////////////////////////////////////////////////////////////////

ulong eclat_nest( double min_supp, double min_conf, double load_bal,
               ulong trans_cnt, ulong max_sz, ulong tot_items, char in_type,
               pary_ul_tp & trans_len, pary_ul_tp & item_cnt,
               stapl::stream<ifstream> & trans_data,
               stapl::stream<ofstream> & freq_out,
               stapl::stream<ofstream> & dbg_out ) {

#ifdef STAPL_TIMER
  stapl::counter_t timer;
#endif
#ifdef SYS_TIMER
  double time_start, time_end;
#endif

  // Input Step

#ifdef STAPL_TIMER
  timer.reset();
  timer.start();
#endif
#ifdef SYS_TIMER
  seconds(time_start);
#endif

  pary_ul_vw_tp trans_len_vw(trans_len);

  // Horizontal database

  pvec_pvec_item_tp horz_db(trans_cnt); // 2-d parallelism
  pvec_pvec_item_vw_tp horz_db_vw(horz_db);

  stapl::map_func( inner_resize_wf(),
                   horz_db_vw, trans_len_vw );

  // read the transactions database

  if ( in_type == 't' ) {
    stapl::serial_io(read_trans_txt_wf(trans_data), horz_db_vw);
  } else {
    stapl::serial_io(read_trans_bin_wf(trans_data), horz_db_vw);
  }
  horz_db_vw.flush();
  stapl::rmi_fence();

#if defined(DBG_PHASE)
  stapl::do_once([&]() {
    dbg_out << "End phase:: read_input " <<endl;
  });
#endif

#ifdef STAPL_TIMER
  double step1_time = timer.stop();
  dbg_out << "Step 1: " << step1_time << endl;
#endif
#ifdef SYS_TIMER
  seconds(time_end);
  double step1_time = time_end - time_start;
  dbg_out << "Step 1: " << step1_time << endl;
#endif

#if defined(DBG_STEP)
  stapl::do_once([&]() { // only shows 1 location
    dbg_out << "Input step -> horizontal database: " <<
               horz_db_vw.size() << endl;
    auto outer_it = horz_db_vw.begin();
    for ( ; outer_it != horz_db_vw.end(); outer_it++ ) {
      auto trans = *outer_it;
      dbg_out << trans.size() << " ";
#if defined(DBG_DATA)
      auto inner_it = trans.begin();
      for ( ; inner_it != trans.end(); inner_it++ ) {
        auto elem = *inner_it;
        dbg_out << elem << " ";
      }
      dbg_out << endl;
#endif
    }
    dbg_out << endl;
  });
#endif

  // Initialize Step

#ifdef STAPL_TIMER
  timer.reset();
  timer.start();
#endif
#ifdef SYS_TIMER
  seconds(time_start);
#endif

  pvec_pair_item_tp itemset_2;
  pvec_pair_item_vw_tp temp_itemset_2_vw(itemset_2);

  initialize_nest(min_supp, trans_cnt, tot_items, horz_db_vw,
                  temp_itemset_2_vw, dbg_out);
  pvec_pair_item_vw_tp itemset_2_vw(itemset_2);

  if ( itemset_2_vw.size() == 0 ) {
    stapl::do_once([&]() {
      dbg_out << "Initialize step failed: " << endl;
    });
    return 0;
  }

#if defined(DBG_PHASE)
  stapl::do_once([&]() {
    dbg_out << "End phase:: initialize_nest " <<endl;
  });
#endif

#if defined(DBG_STEP)
  stapl::do_once([&]() {
    dbg_out << "Initialize step -> counts, itemsets(length==2): " <<
               itemset_2_vw.size() << endl;
  });
  stapl::do_once([&]() {
    for ( ulong i=0; i<itemset_2.size(); i++ ) {
      pair_item_tp key = itemset_2[i];
      dbg_out << key.first << " " << key.second << endl;
    }
  });
#endif

#ifdef REMOVE_THIS
  ulong size = itemset_2_vw.size();
  ary_sz_tp vec_len(size);
  ary_sz_vw_tp vec_len_vw(vec_len);
  stapl::fill_n(vec_len_vw, 2, vec_len_vw.size());
#endif

#ifdef STAPL_TIMER
  double step2_time = timer.stop();
  dbg_out << "Step 2: " << step2_time << endl;
#endif
#ifdef SYS_TIMER
  seconds(time_end);
  double step2_time = time_end - time_start;
  dbg_out << "Step 2: " << step2_time << endl;
#endif

  // Transform Step - create vertical database

#ifdef STAPL_TIMER
  timer.reset();
  timer.start();
#endif
#ifdef SYS_TIMER
  seconds(time_start);
#endif

#ifdef REMOVE_THIS
  pvec_vec_item_tp in_itemsets(vec_len_vw);
  pvec_vec_item_vw_tp arg_itemsets_vw(in_itemsets);
#else
  ulong size = itemset_2_vw.size();
  pvec_vec_item_tp in_itemsets(size);
  pvec_vec_item_vw_tp arg_itemsets_vw(in_itemsets);
#endif

  vector<eqv_cl_tp *> equiv_class;
  prepare_vertical(min_supp, trans_cnt, tot_items,
                   horz_db_vw,
                   itemset_2_vw,
                   arg_itemsets_vw,
                   equiv_class, dbg_out );
  in_itemsets.flush();
  pvec_vec_item_vw_tp in_itemsets_vw(in_itemsets);

#if defined(DBG_PHASE)
  stapl::do_once([&]() {
    dbg_out << "End phase:: prepare_vertical: " <<
               "#equiv_class= " << equiv_class.size() << " " <<
               "#in_itemsets= " << in_itemsets_vw.size() << endl;
  });
#endif

#if defined(DBG_DATA)
  stapl::do_once([&]() {
    auto set_it = in_itemsets_vw.begin();
    for ( ; set_it != in_itemsets_vw.end(); ++set_it ) {
      auto duo = *set_it;
      dbg_out << duo[0] << "," << duo[1] << endl;
    }
  });
#endif

  // ensure there is enough work to distribute to all locations
  ulong num_locs = horz_db_vw.get_num_locations();
  ulong ctl_breaks = count_control_breaks(in_itemsets_vw, dbg_out);
  if ( ctl_breaks < num_locs ) {
    return 0;
  }

  // create the distribution of the candidates across the locations
  stapl::array<stapl::arbitrary_partition_info> part_info(
                                         horz_db_vw.get_num_locations());
  ulong part_cnt = gen_arb_dist( in_itemsets_vw, part_info, dbg_out );
  if ( part_cnt < num_locs ) {
    return 0;
  }

  stapl::array_view<stapl::array<stapl::arbitrary_partition_info>>
         part_view(part_info);

  int dom_lmt = in_itemsets_vw.size();
  ndx_dom_tp map_dom(0, dom_lmt-1);
  pmap_ul_pset_ul_tp vert_db(map_dom);
  pmap_ul_pset_ul_vw_tp vert_db_vw(vert_db);

#if defined(DBG_PHASE)
  stapl::do_once([&]() {
    dbg_out << "End phase:: gen_arb_dist: " <<
               "part_cnt= " << part_cnt << endl;
  });
#endif

  transform_nest( horz_db_vw, vert_db_vw, itemset_2_vw,
                 equiv_class, in_itemsets_vw, dbg_out);

#if defined(DBG_PHASE)
  stapl::do_once([&]() {
    dbg_out << "End phase:: transform_nest: " << endl;
  });
#endif

#ifdef STAPL_TIMER
  double step3_time = timer.stop();
  dbg_out << "Step 3: " << step3_time << endl;
#endif
#ifdef SYS_TIMER
  seconds(time_end);
  double step3_time = time_end - time_start;
  dbg_out << "Step 3: " << step3_time << endl;
#endif

#if defined(DBG_STEP)
  stapl::do_once([&]() {
    dbg_out << "Transform step -> vertical database: " <<
               vert_db_vw.size() << endl;
  });
#endif

#if defined(DBG_DATA)
  stapl::serial_io(show_vert_stapl_wf(dbg_out), vert_db_vw,
                   stapl::make_repeat_view(in_itemsets_vw),
                   stapl::counting_view<int>(vert_db_vw.size()) );
  stapl::do_once([&]() {
    dbg_out << "" << endl;
  });
#endif

  // Asynchronous Step - find itemsets length > 2

#ifdef STAPL_TIMER
  timer.reset();
  timer.start();
#endif
#ifdef SYS_TIMER
  seconds(time_start);
#endif

  ary_sz_tp outer_len(num_locs);
  ary_sz_vw_tp outer_len_vw(outer_len);
  stapl::map_func( inner_fill_wf(), outer_len_vw,
                   stapl::make_repeat_view<int>(1) );

  ary_2_sz_tp middle_len(outer_len);
  ary_2_sz_vw_tp middle_len_vw(middle_len);
  stapl::map_func( outer_fill_wf(), middle_len_vw,
                   stapl::make_repeat_view<int>(1) );

  pvec_pvec_pvec_ul_tp out_itemsets(middle_len_vw);
  pvec_pvec_pvec_ul_vw_tp temp_itemsets_vw(out_itemsets);

  pvec_ul_tp out_set_counts;
  pvec_ul_vw_tp out_set_counts_vw(out_set_counts);

  asynchronous_nest( min_supp, trans_cnt, tot_items,
                     vert_db_vw, in_itemsets_vw, temp_itemsets_vw,
                     out_set_counts_vw, dbg_out);
  out_itemsets.flush();
  out_set_counts.flush();

  pvec_pvec_pvec_ul_vw_tp out_itemsets_vw(out_itemsets);

#if defined(DBG_PHASE)
  stapl::do_once([&]() {
    dbg_out << "End phase:: asynchronous_nest: " << endl;
  });
#endif

#ifdef STAPL_TIMER
  double step4_time = timer.stop();
  dbg_out << "Step 4: " << step4_time << endl;
#endif
#ifdef SYS_TIMER
  seconds(time_end);
  double step4_time = time_end - time_start;
  dbg_out << "Step 4: " << step4_time << endl;
#endif

#if defined(DBG_STEP)
  // out_itemsets dumped locally
  stapl::do_once([&]() {
    dbg_out << "Asynchronous step -> " <<
               out_itemsets_vw.size() << endl;
  });
#endif

  // Aggregate Step - save the results

#ifdef STAPL_TIMER
  timer.reset();
  timer.start();
#endif
#ifdef SYS_TIMER
  seconds(time_start);
#endif

#if defined(DBG_STEP)
  stapl::do_once([&]() {
    dbg_out << "Aggregate step -> FILE" << endl;
  });
#endif

  //stapl::serial_io(write_freq_std_wf(freq_out), out_itemsets_vw );

#ifdef STAPL_TIMER
  double step5_time = timer.stop();
  dbg_out << "Step 2: " << step5_time << endl;
#endif
#ifdef SYS_TIMER
  seconds(time_end);
  double step5_time = time_end - time_start;
  dbg_out << "Step 5: " << step5_time << endl;
#endif

#ifdef AGGREGATE
  return set_count;
#else
  return 0;
#endif

}
