#include "freqmine.hpp"
#include "eclat2d.hpp"

//#define DBG_TRACE 1
//#define DBG_DATA 1
//#define DBG_COUNT 1

///////////////////////////////////////////////////////////////////////////
// count the occurrences of items locally in one segment
// then update the global counts
///////////////////////////////////////////////////////////////////////////

struct map_counts_inner_wf
{
private:
  stapl::stream<ofstream> m_dbg;
public:
  map_counts_inner_wf(stapl::stream<ofstream> & dbg)
   : m_dbg(dbg)
  { }
  typedef map_pair_ul_tp result_type;
  template <typename Elem1, typename Elem2, typename View1>
  result_type operator()(Elem1 left, Elem2 ndx, View1 &trans_vw)
  {
    result_type trans_counts;
    for ( ulong j=ndx+1; j<trans_vw.size(); j++ ) {
      ulong right = trans_vw[j];
      pair_ul_tp key = make_pair(left,right);
      ulong count = trans_counts[key];
      trans_counts[key] = count + 1;
    }
    return trans_counts;
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_dbg);
  }
};

///////////////////////////////////////////////////////////////////////////
// merge map that contains keys and counts
///////////////////////////////////////////////////////////////////////////

struct reduce_counts_wf
{
private:
  stapl::stream<ofstream> m_dbg;
public:
  reduce_counts_wf(stapl::stream<ofstream> & dbg)
   : m_dbg(dbg)
  { }
  typedef map_pair_ul_tp result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 proxy1, View2 proxy2)
  {
    map_pair_ul_tp left_counts = proxy1;
    map_pair_ul_tp right_counts = proxy2;

    map_pair_ul_tp::iterator map_it;
    for ( map_it = left_counts.begin();
         map_it != left_counts.end(); ++map_it ) {
      pair_ul_tp key = map_it->first;
      ulong arg_cnt = map_it->second;

      ulong temp_cnt = right_counts[key];
      right_counts[key] = temp_cnt + arg_cnt;
    }

#if defined(DBG_COUNT)
  stapl::do_once([&]() {
    m_dbg << "< reduce_counts: " << right_counts.size() << endl;
  });
#endif
    return right_counts;
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_dbg);
  }
};

///////////////////////////////////////////////////////////////////////////
// count the occurrences of items locally in one segment
// then update the global counts
///////////////////////////////////////////////////////////////////////////

struct map_counts_outer_wf
{
private:
  stapl::stream<ofstream> m_dbg;
public:
  map_counts_outer_wf(stapl::stream<ofstream> & dbg)
   : m_dbg(dbg)
  { }

  typedef map_pair_ul_tp result_type;
  template <typename SegView>
  result_type operator()(SegView seg)
  {
    ulong count = 0;
    result_type seg_counts;
    typename SegView::iterator seg_it;
    for ( seg_it = seg.begin(); seg_it != seg.end(); ++seg_it ) {
      auto trans_vw = *seg_it;
      ulong size = trans_vw.size();

#if defined(DBG_COUNT)
  stapl::do_once([&]() {
    m_dbg << "> map_counts_outer: " << trans_vw.size() << endl;
  });
#endif

      result_type trans_counts =
        stapl::map_reduce( map_counts_inner_wf(m_dbg),
                           reduce_counts_wf(m_dbg), trans_vw,
                           stapl::counting_view<int>(size),
                           stapl::make_repeat_view(trans_vw) );
      count += trans_counts.size();

      merge_counts(trans_counts, seg_counts);

#if defined(DBG_COUNT)
      stapl::do_once([&]() {
        m_dbg << "trans_counts " << trans_counts.size() << " "
                                 << seg_counts.size() << endl;
      });
#endif
    }

#if defined(DBG_COUNT)
    stapl::do_once([&]() {
      m_dbg << "map_counts_outer_wf := " << seg_counts.size() << endl;
      map_pair_ul_tp::iterator map_it;
      for ( map_it = seg_counts.begin();
         map_it != seg_counts.end(); ++map_it ) {
        pair_ul_tp key = map_it->first;
        ulong local_cnt = map_it->second;
        m_dbg << key.first << " " << key.second << " " << local_cnt << endl;
      }
      m_dbg << endl;
    });
#endif

    return seg_counts;
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_dbg);
  }
};


///////////////////////////////////////////////////////////////////////////
// - identify 2-itemsets with counts
// - enumerate pairs with sufficient support
///////////////////////////////////////////////////////////////////////////

void initialize_nest( double min_supp, ulong trans_cnt, ulong item_cnt,
                    pvec_pvec_item_vw_tp & horz_db_vw,
                    pvec_pair_item_vw_tp & itemset_2,
                    stapl::stream<ofstream> & dbg_out ) {

  map_pair_ul_tp counts =
    stapl::map_reduce( map_counts_outer_wf(dbg_out), reduce_counts_wf(dbg_out),
                       stapl::native_view(horz_db_vw) );

#if defined(DBG_DATA)
    stapl::do_once([&]() {
      dbg_out << "map_counts_outer_wf: " << counts.size() << endl;
      map_pair_ul_tp::iterator map_it;
      for ( map_it = counts.begin();
         map_it != counts.end(); ++map_it ) {
        pair_ul_tp key = map_it->first;
        ulong local_cnt = map_it->second;
        dbg_out << key.first << " " << key.second << " " << local_cnt << endl;
      }
      dbg_out << endl;
    });
#endif

  // OPTIMIZE: change counts to pmap, replace loop with map_func
  stapl::do_once([&]() {
    double trans_cnt_dbl = (double) trans_cnt;
    map_pair_ul_tp::iterator map_it;
    for ( map_it = counts.begin();
         map_it != counts.end(); ++map_it )  {

      pair_ul_tp key = map_it->first;
      double cnt_dbl = (double) map_it->second;

      if ( ( cnt_dbl / trans_cnt_dbl ) >= min_supp ) {
        itemset_2.push_back( key ); // flush below
      }
    }
  });
  itemset_2.flush(); // push_back above

#if defined(DBG_DATA)
  stapl::do_once([&]() {
    dbg_out << "filtered pairs: " << itemset_2.size() << endl;
    pvec_pair_ul_vw_tp::iterator pairs_it;
    for ( pairs_it = itemset_2.begin();
          pairs_it != itemset_2.end(); ++pairs_it )  {
      dbg_out << (*pairs_it).first << " " << (*pairs_it).second << endl;
    }
    dbg_out << endl;
  });
#endif
}

