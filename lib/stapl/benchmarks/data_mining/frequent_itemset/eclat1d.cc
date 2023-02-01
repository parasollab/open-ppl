#include "freqmine.hpp"
#include "eclat1d.hpp"
#include "mappers.hpp"

#include <stapl/containers/distribution/specifications.hpp>
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

//#define DBG_PHASE 1
//#define DBG_DATA 1

#define USE_AGGREG_OUTPUT 1
// this will cause aggregation of output item sets from a pvec<vec<item>>
// to a simple vector of unsigned ints.  drastically reduces I/O time

///////////////////////////////////////////////////////////////////////////
// put each frequent itemset in ascending order
///////////////////////////////////////////////////////////////////////////

struct finalize_wf
{
  typedef void result_type;
  template <typename Cont>
  result_type operator()(Cont cont) {
    std::sort( cont.begin(), cont.end() );
  }
};

///////////////////////////////////////////////////////////////////////////
// perform the ECLAT algorithm with parallelism depth=1
///////////////////////////////////////////////////////////////////////////

ulong eclat_flat( double min_supp, double min_conf, double load_bal,
               ulong trans_cnt, ulong max_sz, ulong tot_items, char in_type,
               pary_ul_tp & trans_len, pary_ul_tp & item_cnt,
               stapl::stream<ifstream> & trans_data,
               stapl::stream<ofstream> & freq_out,
               stapl::stream<ofstream> & dbg_out ) {

#if defined(STAPL_TIMER)
  counter_t timer;
#endif
#if defined(DBG_PHASE) && defined(SYS_TIMER)
  double time_start, time_end;
#endif

  // Input step

#if defined(STAPL_TIMER)
  timer.reset();
  timer.start();
#endif
#if defined(DBG_PHASE) && defined(SYS_TIMER)
  seconds(time_start);
#endif

  pary_ul_vw_tp trans_len_vw(trans_len);

  // Horizontal database

  pvec_vec_item_tp horz_db(trans_cnt);
  pvec_vec_item_vw_tp horz_db_vw(horz_db);

  stapl::map_func( resize_vec_wf(),
                   horz_db_vw, trans_len_vw );

  // read the transactions database

  if ( in_type == 't' ) {
    stapl::serial_io(read_trans_txt_wf(trans_data), horz_db_vw);
  } else {
    stapl::serial_io(read_trans_bin_wf(trans_data), horz_db_vw);
  }

#if defined(DBG_PHASE)
  stapl::do_once([&]() {
    dbg_out << "End phase:: read_input" << endl;
  });
#endif

#if defined(STAPL_TIMER)
  double step1_time = timer.stop();
  dbg_out << "Step 1: " << step1_time << endl;
#endif
#if defined(DBG_PHASE) && defined(SYS_TIMER)
  seconds(time_end);
  double step1_time = time_end - time_start;
  dbg_out << "Step 1: " << step1_time << endl;
#endif

#if defined(DBG_STEP)
  stapl::do_once([&]() {
    dbg_out << "Input step -> horizontal database " <<
               horz_db_vw.size() << endl;
  });
#endif

#if defined(DBG_DATA)
  stapl::do_once([&]() {
     dbg_out << "unsigned long trans_len_vals[]={" << endl;
  });
  stapl::serial_io(show_horz_len_wf(dbg_out), horz_db_vw,
                   stapl::counting_view<int>(horz_db_vw.size() ) );
  stapl::do_once([&]() {
     dbg_out << "};" << endl;
     dbg_out << "unsigned long horz_db_val[]={" << endl;
  });
  stapl::serial_io(show_horz_val_wf(dbg_out), horz_db_vw );
  stapl::do_once([&]() {
     dbg_out << "};" <<  endl;
  });
#endif

  pvec_pair_item_tp itemset_2;
  pvec_pair_item_vw_tp arg_itemset_2_vw(itemset_2);

  // Initialize Step

#if defined(STAPL_TIMER)
  timer.reset();
  timer.start();
#endif
#if defined(DBG_PHASE) && defined(SYS_TIMER)
  seconds(time_start);
#endif

  initialize_flat(min_supp, trans_cnt, tot_items, horz_db_vw,
                  arg_itemset_2_vw, dbg_out);

  pvec_pair_item_vw_tp itemset_2_vw(itemset_2);

#if defined(DBG_PHASE)
  stapl::do_once([&]() {
    dbg_out << "End phase:: initialize_flat" << endl;
  });
#endif

#if defined(STAPL_TIMER)
  double step2_time = timer.stop();
  dbg_out << "Step 2: " << step1_time << endl;
#endif
#if defined(DBG_PHASE) && defined(SYS_TIMER)
  seconds(time_end);
  double step2_time = time_end - time_start;
  dbg_out << "Step 2: " << step2_time << endl;
#endif

#if defined(DBG_STEP)
  stapl::do_once([&]() {
    dbg_out << "Initialize step -> counts, itemsets(length==2) " <<
               itemset_2.size() << endl;
  });
#endif

  // ensure there is enough work to distribute to all locations
  ulong num_locs = horz_db_vw.get_num_locations();
  if ( itemset_2.size() < num_locs ) {
    dbg_out << "itemset-2 " << itemset_2.size() <<
               ", num_locs " << num_locs << endl;
    stapl::rmi_fence();
    return 0;
  }

#if defined(DBG_DATA)
  stapl::do_once([&]() {
    dbg_out << "unsigned long itemset_2_count = " << itemset_2_vw.size() <<
               ";" << endl;
    dbg_out << "unsigned long itemset_2_vals[][2] = {" << endl;
    for ( ulong i=0; i<itemset_2_vw.size(); i++ ) {
      pair_item_tp key = itemset_2_vw[i];
      dbg_out << "{" <<  key.first << ", " << key.second << "},"  << endl;
    }
    dbg_out << "};" << endl << endl;
  });
#endif

  // Transform Step - create vertical database

#if defined(STAPL_TIMER)
  timer.reset();
  timer.start();
#endif
#if defined(DBG_PHASE) && defined(SYS_TIMER)
  seconds(time_start);
#endif

  pvec_vec_item_tp in_itemsets;
  pvec_vec_item_vw_tp arg_itemsets_vw(in_itemsets);

#ifdef FIXED_MAP_PAIR
#ifdef USE_UNORD_MAP
  phash_duo_ul_tp in_item_pairs;
  phash_duo_ul_vw_tp in_item_pairs_vw(in_item_pairs);
#else
  pmap_duo_ul_tp in_item_pairs;
  pmap_duo_ul_vw_tp in_item_pairs_vw(in_item_pairs);
#endif
#else // FIXED_MAP_PAIR
#ifdef USE_UNORD_MAP
  phash_encpair_ul_tp in_item_pairs;
  phash_encpair_ul_vw_tp in_item_pairs_vw(in_item_pairs); // HERE
#else
  ndx_dom_ul_tp map_dom(0,max_key);
  pmap_encpair_ul_tp in_item_pairs(map_dom);
  pmap_encpair_ul_vw_tp in_item_pairs_vw(in_item_pairs);
#endif
#endif // FIXED_MAP_PAIR

  vector<eqv_cl_tp *> equiv_class;
  prepare_vertical(min_supp, trans_cnt, tot_items, horz_db_vw,
                   itemset_2_vw,
                   arg_itemsets_vw,
                   in_item_pairs_vw,
                   equiv_class, dbg_out);
  in_itemsets.flush();
  pvec_vec_item_vw_tp in_itemsets_vw(in_itemsets);

#if defined(DBG_PHASE) || defined(DBG_STEP)
  stapl::do_once([&]() {
    dbg_out << "End phase:: prepare_vertical: " <<
               "#equiv_class= " << equiv_class.size() << " " <<
               "#in_item_pairs= " << in_item_pairs_vw.size() <<  endl;
  });
#endif

#if defined(DBG_DATA)
  stapl::do_once([&]() {
    dbg_out << "in_itemsets" << endl;
    int ctr = 0;
    pvec_vec_item_vw_tp::iterator set_it= in_itemsets_vw.begin();
    for ( ; set_it != in_itemsets_vw.end(); ++set_it ) {
      auto duo = *set_it;
      dbg_out << "[" << ctr << "] " << duo[0] << "," << duo[1] << endl;
      ctr++;
    }
  });
#endif

  // ensure there is enough work to distribute to all locations
  ulong ctl_breaks = count_control_breaks(in_itemsets_vw, dbg_out);
  if ( ctl_breaks < num_locs ) {
    dbg_out << "ctl_breaks " << ctl_breaks << ", num_locs " << num_locs << endl;
    stapl::rmi_fence();
    return 0;
  }

  // create the distribution of the candidates across the locations
  stapl::array<stapl::arbitrary_partition_info> part_info(num_locs);
  ulong part_cnt = gen_arb_dist( in_itemsets_vw, part_info, dbg_out );
  if ( part_cnt != num_locs ) {
    dbg_out << "part_cnt " << part_cnt << ", num_locs " << num_locs << endl;
    stapl::rmi_fence();
    return 0;
  }

#if defined(DBG_PHASE)
  stapl::do_once([&]() {
    dbg_out << "End phase:: screen in_itemsets " <<  endl;
  });
#endif

//#define HYBRID_MAP 1
#ifdef HYBRID_MAP
  GID_to_PID_tp gid_to_pid(part_info);
  PID_to_LID_tp pid_to_lid;
  dist_spec_tp arb_spec = stapl::arbitrary(
    trans_cnt, stapl::get_num_locations(),
    gid_to_pid, pid_to_lid );
#else
  stapl::array_view<stapl::array<stapl::arbitrary_partition_info>>
         part_view(part_info);
  dist_spec_tp arb_spec = stapl::arbitrary( part_view );
#endif

  // vertical database MUST be ordered for build_equiv_class to work
#ifdef USE_UNORD_ARB_MAP
  phash_ul_set_ul_arb_tp vert_db(part_view);
  phash_ul_set_ul_arb_vw_tp vert_temp_vw(vert_db);
#else
  pmap_ul_set_ul_arb_tp vert_db(part_view);
  pmap_ul_set_ul_arb_vw_tp vert_temp_vw(vert_db);
#endif

  pvec_pair_ul_set_ul_tp invert;
  pvec_pair_ul_set_ul_vw_tp invert_vw(invert);

#if defined(DBG_PHASE)
  stapl::do_once([&]() {
    dbg_out << "End phase:: gen_arb_dist " <<  endl;
  });
#endif

  transform_flat( horz_db_vw, invert_vw, vert_temp_vw, equiv_class,
                 in_item_pairs_vw, dbg_out );

#ifdef USE_UNORD_ARB_MAP
  phash_ul_set_ul_arb_vw_tp vert_db_vw(vert_db);
#else
  pmap_ul_set_ul_arb_vw_tp vert_db_vw(vert_db);
#endif

#if defined(DBG_PHASE)
  stapl::do_once([&]() {
    dbg_out << "End phase:: transform_flat " <<  endl;
  });
#endif

// vert_db is built - can release horz_db and in_item_pairs

#if defined(STAPL_TIMER)
  double step3_time = timer.stop();
  dbg_out << "Step 3: " << step1_time << endl;
#endif
#if defined(DBG_PHASE) && defined(SYS_TIMER)
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
  stapl::do_once([&]() {
    dbg_out << "[Index] itemset @ { tidset }" << endl;
  });
  stapl::serial_io(show_vert_std_wf(dbg_out), vert_db_vw,
                   stapl::make_repeat_view(in_itemsets_vw),
                   stapl::counting_view<int>(vert_db_vw.size()) );
  stapl::do_once([&]() {
    dbg_out << "" << endl;
  });
#endif

  // Asynchronous Step - find itemsets length > 2

#if defined(STAPL_TIMER)
  timer.reset();
  timer.start();
#endif
#if defined(DBG_PHASE) && defined(SYS_TIMER)
  seconds(time_start);
#endif

  pvec_vec_item_tp out_itemsets;
  pvec_vec_item_vw_tp temp_itemsets_vw(out_itemsets);

  asynchronous_flat( min_supp, trans_cnt, tot_items,
                     vert_db_vw, in_itemsets_vw, temp_itemsets_vw, dbg_out);
  out_itemsets.flush();
  stapl::rmi_fence();

// results are computed - can release vert_db

#if defined(DBG_PHASE)
  stapl::do_once([&]() {
    dbg_out << "End phase:: asynchronous_flat " <<  endl;
  });
#endif

#if defined(STAPL_TIMER)
  double step4_time = timer.stop();
  dbg_out << "Step 4: " << step1_time << endl;
#endif
#if defined(DBG_PHASE) && defined(SYS_TIMER)
  seconds(time_end);
  double step4_time = time_end - time_start;
  dbg_out << "Step 4: " << step4_time << endl;
#endif

  // Aggregate Step - save the results

#if defined(STAPL_TIMER)
  timer.reset();
  timer.start();
#endif
#if defined(DBG_PHASE) && defined(SYS_TIMER)
  seconds(time_start);
#endif

  // it is just as easy to perform a separate write to the file
  // in_itemsets are already sorted, so this reduces the cost of the sort
  stapl::serial_io(write_freq_std_wf(freq_out), in_itemsets_vw );

  pvec_vec_item_vw_tp out_itemsets_vw(out_itemsets);
  ulong size = out_itemsets_vw.size();

#ifdef FIXED_HANG_OUTSETS
  stapl::map_func( finalize_wf(),
                   out_itemsets_vw );
#else // FIXED_HANG_OUTSETS
  pvec_vec_item_tp tuple_itemsets(size);
  pvec_vec_item_vw_tp tuple_itemsets_vw(tuple_itemsets);
  for ( ulong i=0; i<size; ++i) {
    tuple_itemsets_vw[i] = out_itemsets_vw[i];
  }
  stapl::rmi_fence();
  stapl::map_func( finalize_wf(),
                   tuple_itemsets_vw );
#endif // FIXED_HANG_OUTSETS

#if defined(STAPL_TIMER)
  double step5_time = timer.stop();
  dbg_out << "Step 5: " << step5_time << endl;
#endif
#if defined(DBG_PHASE) && defined(SYS_TIMER)
  seconds(time_end);
  double step5_time = time_end - time_start;
  dbg_out << "Step 5: " << step5_time << endl;
#endif

#ifdef FIXED_HANG_OUTSETS
#ifndef USE_AGGREG_OUTPUT
  stapl::serial_io(write_freq_std_wf(freq_out), out_itemsets_vw );
  freq_out.close();
#else // USE_AGGREG_OUTPUT
  pvec_vec_item_tp bin_itemsets(num_locs);
  pvec_vec_item_vw_tp bin_itemsets_vw(bin_itemsets);
  aggregate_flat( out_itemsets_vw, bin_itemsets_vw, dbg_out );
  stapl::serial_io(write_freq_bin_wf(freq_out), bin_itemsets_vw );
#endif // USE_AGGREG_OUTPUT

#else // FIXED_HANG_OUTSETS
  pvec_vec_item_tp bin_itemsets(num_locs);
  pvec_vec_item_vw_tp bin_itemsets_vw(bin_itemsets);
  aggregate_flat( tuple_itemsets_vw, bin_itemsets_vw, dbg_out );
  stapl::serial_io(write_freq_bin_wf(freq_out), bin_itemsets_vw );
#endif// FIXED_HANG_OUTSETS

#if defined(DBG_STEP)
  stapl::do_once([&]() {
    dbg_out << "Aggregate step -> FILE" << endl;
  });
#endif

#if defined(STAPL_TIMER)
  double step6_time = timer.stop();
  dbg_out << "Step 6: " << step6_time << endl;
#endif
#if defined(DBG_PHASE) && defined(SYS_TIMER)
  seconds(time_end);
  double step6_time = time_end - time_start;
  dbg_out << "Step 6: " << step6_time << endl;
#endif

#ifdef FIXED_HANG_OUTSETS
  return in_itemsets_vw.size() + out_itemsets_vw.size();
#else // FIXED_HANG_OUTSETS
  return in_itemsets_vw.size() + tuple_itemsets_vw.size();
#endif // FIXED_HANG_OUTSETS
}
