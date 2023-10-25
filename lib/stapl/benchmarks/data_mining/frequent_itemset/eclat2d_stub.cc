#include "freqmine.hpp"

ulong 
eclat_nest( double min_supp, double min_conf,
            ulong trans_cnt, ulong max_sz, ulong tot_items, 
            char in_type, pary_ul_tp & trans_len, 
            pary_ul_tp & item_cnt,
            stapl::stream<ifstream> & trans_data,
            stapl::stream<ofstream> & local_out,
            stapl::stream<ofstream> & freq_out,
            stapl::stream<ofstream> & dbg_out ) {
  return 0; 
}

void 
initialize_nest( double min_supp, ulong trans_cnt, 
                 ulong item_cnt, pvec_pvec_item_vw_tp & horz_db_vw,
                 pvec_pair_ul_vw_tp & itemset_2,
                 stapl::stream<ofstream> & dbg_out ) { }

void 
transform_nest( double min_supp, ulong trans_cnt, ulong item_cnt,
                pvec_pvec_item_vw_tp & horz_db_vw,
                pmap_ul_pset_ul_vw_tp & vert_db_vw,
                pvec_pair_ul_vw_tp & itemset_2_vw,
                pvec_pvec_ul_vw_tp & in_item_sets_vw,
                stapl::stream<ofstream> & dbg_out ) { }

void 
asynchronous_nest( double min_supp, ulong trans_cnt, 
                   ulong item_cnt,
                   pmap_ul_pset_ul_vw_tp & vert_db_vw,
                   pvec_pvec_ul_vw_tp & in_item_sets_vw,
                   pvec_pvec_ul_vw_tp & out_item_sets_vw,
                   pvec_ul_vw_tp & out_set_counts_vw,
                   stapl::stream<ofstream> & local_out,
                   stapl::stream<ofstream> & dbg_out ) { }

void 
aggregate_nest( ulong num_locs,
                pvec_pvec_ul_vw_tp & out_item_sets_vw,
                stapl::stream<ofstream> & dbg_out ) { }

