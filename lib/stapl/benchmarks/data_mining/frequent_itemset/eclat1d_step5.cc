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

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

struct aggregate_flat_wf
{
private:
  ulong m_loc_id;
  stapl::stream<ofstream> m_dbg;
public:
  aggregate_flat_wf( ulong loc, stapl::stream<ofstream> & out )
    : m_loc_id(loc), m_dbg(out)
  { }

  typedef void result_type;
  template<typename SegView, typename RepView>
  result_type operator() (SegView seg_vw,
                          RepView & bin_itemsets_vw )
  {
    ulong count = 0;
    ulong seg_ctr = 0;
    typename SegView::iterator seg_it;
    for ( seg_it = seg_vw.begin(); seg_it != seg_vw.end(); ++seg_it ) {
      vec_item_tp itemset = *seg_it;
      ulong size = itemset.size();
      count += (1 + size);
      ++seg_ctr;
    }

    vec_item_tp buffer(1+count);
    ulong buf_ctr = 0;
    buffer[buf_ctr++] = seg_ctr;
    for ( seg_it = seg_vw.begin(); seg_it != seg_vw.end(); ++seg_it ) {
      vec_item_tp itemset = *seg_it;
      ulong size = itemset.size();
      buffer[buf_ctr++] = size;
      for ( ulong i=0; i<size; i++ ) {
        buffer[buf_ctr++] = itemset[i];
      }
    }
    bin_itemsets_vw[m_loc_id] = buffer;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_dbg);
  }
};

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

void aggregate_flat( pvec_vec_item_vw_tp & out_itemsets_vw,
                     pvec_vec_item_vw_tp & bin_itemsets_vw,
                     stapl::stream<ofstream> & dbg_out ) {

  ulong loc_id = out_itemsets_vw.get_location_id();
  stapl::map_func( aggregate_flat_wf(loc_id, dbg_out),
                   stapl::native_view(out_itemsets_vw),
                   stapl::make_repeat_view(bin_itemsets_vw) );
}
