#include "freqmine.hpp"
#include "eclat1d.hpp"

#define SYS_TIMER 1
#ifdef SYS_TIMER
#include "timing.h"
#endif

//#define DBG_DATA 1

///////////////////////////////////////////////////////////////////////////
// compute the per-location counts of each pair
///////////////////////////////////////////////////////////////////////////

struct partial_map_counts_wf
{
private:
  ulong m_loc_id;
  stapl::stream<ofstream> m_dbg;
public:
  partial_map_counts_wf(ulong loc, stapl::stream<ofstream> & out)
   : m_loc_id(loc), m_dbg(out)
  { }
  typedef void result_type;
  template <typename SegView, typename RepView>
  result_type operator()(SegView seg, RepView & vec_map)
  {
    map_pair_ul_tp local_counts;
    typename SegView::iterator seg_it;
    for ( seg_it = seg.begin(); seg_it != seg.end(); ++seg_it ) {
      auto trans = *seg_it;

      for ( ulong i=0; i<trans.size(); i++ ) {
        for ( ulong j=i+1; j<trans.size(); j++ ) {
          pair_ul_tp key = make_pair(trans[i],trans[j]);
          ulong count = local_counts[key];
          local_counts[key] = count + 1;
        }
      }
    }
    vec_map[m_loc_id] = local_counts;

#if defined(DBG_DATA)
    stapl::do_once([&]() {
      m_dbg << "map_counts_wf: " << local_counts.size() << endl;
      map_pair_ul_tp::iterator map_it;
      for ( map_it = local_counts.begin();
         map_it != local_counts.end(); ++map_it ) {
        pair_ul_tp key = map_it->first;
        ulong local_cnt = map_it->second;
        m_dbg << key.first << " " << key.second << " " << local_cnt << endl;
      }
    });
#endif
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_dbg);
  }
};

///////////////////////////////////////////////////////////////////////////
// reduce the per-location counts of each pair to a single global count
///////////////////////////////////////////////////////////////////////////

struct partial_reduce_counts_wf
{
private:
  stapl::stream<ofstream> m_dbg;
public:
  partial_reduce_counts_wf(stapl::stream<ofstream> & out)
   : m_dbg(out)
  { }

  struct insert_update_wf
  {
    typedef void result_type;
    template<typename MapElement, typename NewElement>
    result_type operator()(MapElement& val, NewElement const& new_val) const
    {
      ulong local_count = new_val.second;
      ulong global_count = val.second;
      ulong result_count = global_count + local_count;
      val.second = result_count;
    }
  };


  typedef void result_type;
  template <typename View1, typename RepView2>
  result_type operator()(View1 proxy1, RepView2 & proxy2)
  {
    map_pair_ul_tp partial_counts = proxy1;
#ifdef FIXED_MAP_PAIR
    pmap_duo_ul_vw_tp global_counts_vw = proxy2;
#else
    pmap_encpair_ul_vw_tp global_counts_vw = proxy2;
#endif
    map_pair_ul_tp::iterator map_it;
    for ( map_it = partial_counts.begin();
         map_it != partial_counts.end(); ++map_it ) {
      pair_ul_tp temp = map_it->first;
      ulong first = temp.first;
      ulong second = temp.second;
      ulong partial_cnt = map_it->second;
#ifdef FIXED_MAP_PAIR
      duo_ul_tp key = duo<ulong>(first,second);
#else
      // encode
      encpair_tp key = (first << encode_shift) | second;
#endif

      global_counts_vw.insert( key, partial_cnt, insert_update_wf() );
    }
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_dbg);
  }
};

///////////////////////////////////////////////////////////////////////////
// select the pairs that have the minimum required support and save them
///////////////////////////////////////////////////////////////////////////

struct select_pair_counts_wf
{
private:
  double m_trans_cnt;
  double m_min_supp;
  stapl::stream<ofstream> m_dbg;
public:
  select_pair_counts_wf(double cnt, double supp, stapl::stream<ofstream> & out)
   : m_trans_cnt(cnt), m_min_supp(supp), m_dbg(out)
  { }
  typedef void result_type;
  template <typename View1, typename RepView2>
  result_type operator()(View1 counts, RepView2 & itemset_2_vw)
  {
#ifdef FIXED_MAP_PAIR
    duo<ulong> key = counts.first;
    ulong first = key.first();
    ulong second = key.second();
#else
    auto enckey = counts.first;
    // decode
    ulong first = (first_mask&enckey)>>encode_shift;
    ulong second = second_mask&enckey;
#endif
    double cnt_dbl = (double) counts.second;

    if ( ( cnt_dbl / m_trans_cnt ) >= m_min_supp ) {
      itemset_2_vw.push_back( make_pair(first,second) );
    }
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_trans_cnt);
    t.member(m_min_supp);
    t.member(m_dbg);
  }
};

///////////////////////////////////////////////////////////////////////////
// - identify 2-itemsets with counts
// - enumerate pairs with sufficient support
///////////////////////////////////////////////////////////////////////////

void initialize_flat( double min_supp, ulong trans_cnt, ulong item_cnt,
                    pvec_vec_item_vw_tp & horz_db_vw,
                    pvec_pair_item_vw_tp & itemset_2_vw,
                    stapl::stream<ofstream> & dbg_out ) {

  ulong num_locs = horz_db_vw.get_num_locations();
  ulong loc_id = horz_db_vw.get_location_id();

#ifdef FIXED_MAP_PAIR
  pvec_map_duo_ul_tp partial_counts(num_locs);
  pvec_map_duo_ul_vw_tp partial_counts_vw(partial_counts);
#else // FIXED_MAP_PAIR
  pvec_map_pair_ul_tp partial_counts(num_locs);
  pvec_map_pair_ul_vw_tp partial_counts_vw(partial_counts);
#endif // FIXED_MAP_PAIR

  stapl::map_func( partial_map_counts_wf(loc_id,dbg_out),
                   stapl::native_view(horz_db_vw),
                   stapl::make_repeat_view(partial_counts_vw) );

#if defined(DBG_DATA)
  stapl::do_once([&]() {
    dbg_out << "after partial_map_counts_wf " << partial_counts.size() << endl;
    pvec_map_pair_ul_vw_tp::iterator map_it;
    for ( map_it = partial_counts.begin();
          map_it != partial_counts.end(); ++map_it ) {
      dbg_out << (*map_it).size() << " ";
    }
    dbg_out << endl;
  });
#endif

#ifdef FIXED_MAP_PAIR
  pmap_duo_ul_tp global_counts;
  pmap_duo_ul_vw_tp arg_counts_vw(global_counts);
#else
  pmap_encpair_ul_tp global_counts;
  pmap_encpair_ul_vw_tp arg_counts_vw(global_counts);
#endif

  stapl::map_func( partial_reduce_counts_wf(dbg_out),
                   partial_counts_vw,     
                   stapl::make_repeat_view(arg_counts_vw) );

#ifdef FIXED_MAP_PAIR
  pmap_duo_ul_vw_tp global_counts_vw(global_counts);
#else
  pmap_encpair_ul_vw_tp global_counts_vw(global_counts);
#endif

#if defined(DBG_DATA)
    stapl::do_once([&]() {
      dbg_out << "partial_reduce_counts_wf: " << global_counts.size() << endl;
#ifdef FIXED_MAP_PAIR
      pmap_duo_ul_vw_tp::iterator map_it;
      for ( map_it = global_counts.begin();
         map_it != global_counts.end(); ++map_it ) {
        auto key = (*map_it).first;
        ulong count = (*map_it).second;
        ulong first = key.first;
        ulong second = key.second;
        dbg_out << first << " " << second << " " << count << endl;
      }
#else
      pmap_encpair_ul_vw_tp::iterator map_it;
      for ( map_it = global_counts.begin();
         map_it != global_counts.end(); ++map_it ) {
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

  double trans_cnt_dbl = (double) trans_cnt;
  stapl::map_func( select_pair_counts_wf(trans_cnt_dbl,min_supp,dbg_out),
                   global_counts_vw,     
                   stapl::make_repeat_view(itemset_2_vw) );
}
