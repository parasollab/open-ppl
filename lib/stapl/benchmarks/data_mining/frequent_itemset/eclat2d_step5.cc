#include "freqmine.hpp"
#include "eclat2d.hpp"

#define DBG_STEP 1

//#define DBG_TRACE 1
//#define DBG_DATA 1

///////////////////////////////////////////////////////////////////////////
//    aggregate_nest( stapl::get_num_locations(), out_itemsets_vw, dbg_out );
///////////////////////////////////////////////////////////////////////////

void aggregate_nest( ulong num_locs,
                     pvec_pvec_pvec_ul_vw_tp & out_itemsets_vw,
                     stapl::stream<ofstream> & dbg_out ) {

#if 0
  ary_sz_tp outer_len(num_locs);
  ary_sz_vw_tp outer_len_vw(outer_len);
  stapl::map_func( inner_fill_wf(), outer_len_vw,
                   stapl::make_repeat_view<int>(1) );

  ary_2_sz_tp middle_len(outer_len);
  ary_2_sz_vw_tp middle_len_vw(middle_len);
  stapl::map_func( outer_fill_wf(), middle_len_vw,
                   stapl::make_repeat_view<int>(1) );

  pvec_pvec_pvec_ul_tp x_itemsets(middle_len_vw);
#endif
 
}

#if 0
///////////////////////////////////////////////////////////////////////////
// until we can insert a stapl::vector into a nested stapl::vector
// we have to write the results of local computations to separate files,
// and then read them into a global structure
///////////////////////////////////////////////////////////////////////////

void aggregate_nest( ulong num_locs,
                     pvec_pvec_ul_vw_tp & out_itemsets_vw,
                     stapl::stream<ofstream> & dbg_out ) {

  ulong set_ctr = 0;
  pvec_ul_tp out_sets_counts;
  for ( ulong loc=0; loc<num_locs; loc++ ) {
    string loc_str = std::to_string(loc);
    string itemset_path = "eclatlocal" + loc_str;
    ifstream local_in;
    local_in.open(itemset_path.c_str(), ios::in|ios::binary );
    if ( local_in.is_open() ) {
      // number of sets
      ulong set_counts;
      local_in.read( (char *)&set_counts, sizeof(ulong) );
      // number of items in each set
      ulong max_items = 0;
      for ( ulong i=0; i<set_counts; i++ ) {
        ulong item_count;
        local_in.read( (char *)&item_count, sizeof(ulong) );
        out_sets_counts[i] = item_count;
        out_itemsets_vw[set_ctr].resize(item_count);
        max_items = std::max(max_items,item_count);
      }
      vector<ulong> buffer(max_items);
      // contents of each set
      for ( ulong i=0; i<set_counts; i++ ) {
        ulong item_count = out_sets_counts[i];
        local_in.read( (char *) buffer.data(), item_count*sizeof(ulong));
        auto item_set = out_itemsets_vw[set_ctr];
        for ( ulong j=0; j<item_count; j++ ) {
          item_set[j] = buffer[j];
        }
      }
      set_ctr++;
    } else {
      cerr << "Unable to write output file on location: " <<
              stapl::get_location_id() << endl;
    }
  }
}
#endif
