#include <cstdio>

#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <algorithm>
#include <stack>

#include <stapl/utility/tuple.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/domains/continuous.hpp>

#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>

#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>

#include <stapl/views/map_view.hpp>
#include <stapl/containers/map/map.hpp>
#include <stapl/containers/distribution/specifications.hpp>

#define USE_UNORD_MAP 1
#ifdef USE_UNORD_MAP
#include <stapl/containers/unordered_map/unordered_map.hpp>
#endif

#undef USE_UNORD_ARB_MAP
// this doesn't work yet
//#define USE_UNORD_ARB_MAP 1

#undef USE_SET
// defining this will cause transaction ID sets (TIDsets) to be stored
// in an STL unordered_set instead of a vector.
// this has a negative impact on performance
#ifdef USE_SET
#include <stapl/containers/unordered_set/unordered_set.hpp>
#include <stapl/views/set_view.hpp>
#endif

#include <stapl/views/native_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/repeated_view.hpp>

#include <stapl/skeletons/serial.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include <stapl/utility/do_once.hpp>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include <stapl/utility/type_printer.hpp>

using namespace std;

#include "rel_alpha_util.hpp"
#include "algo_supp.hpp"
#include "algo.hpp"
#include "duo.hpp"


///////////////////////////////////////////////////////////////////////////

typedef unsigned long ulong;
typedef unsigned short item_tp;
typedef ulong encpair_tp;

typedef vector<ulong>                               vec_ul_tp;
typedef pair<ulong,ulong>                           pair_ul_tp;
typedef vector<pair_ul_tp>                          vec_pair_ul_tp;
typedef vector<vector<ulong>>                       vec_vec_ul_tp;
typedef map<pair_ul_tp,ulong>                       map_pair_ul_tp;
#ifdef USE_SET
typedef vector<unordered_set<ulong>>                vec_set_ul_tp;
#else
typedef vector<vector<ulong>>                       vec_set_ul_tp;
#endif
#ifdef USE_SET
typedef unordered_set<ulong>                        set_ul_tp;
typedef unordered_map<ulong,unordered_set<ulong>>   hash_ul_set_ul_tp;
typedef map<ulong,unordered_set<ulong>>             map_ul_set_ul_tp;
#else
typedef vector<ulong>                               set_ul_tp;
typedef unordered_map<ulong,vector<ulong>>          hash_ul_set_ul_tp;
typedef map<ulong,vector<ulong>>                    map_ul_set_ul_tp;
#endif

typedef unordered_map<ulong,vector<ulong>>          hash_ul_vec_ul_tp;
typedef map<ulong,vector<ulong>>                    map_ul_vec_ul_tp;

typedef vector<item_tp>                             vec_item_tp;
typedef vector<vec_item_tp>                         vec_vec_item_tp;

typedef pair<item_tp,item_tp>                       pair_item_tp;
typedef vector<pair_item_tp>                        vec_pair_item_tp;

typedef pair<ulong,set_ul_tp>                       pair_ul_set_ul_tp;

typedef duo<ulong>                                  duo_ul_tp;
typedef map<duo<ulong>,ulong>                       map_duo_ul_tp;

// - - - - - - - - - -

typedef stapl::indexed_domain<int>                  ndx_dom_tp;
typedef stapl::indexed_domain<ulong>                ndx_dom_ul_tp;

typedef stapl::array<ulong>                         pary_ul_tp;
typedef stapl::array_view<pary_ul_tp>               pary_ul_vw_tp;

typedef stapl::vector<ulong>                        pvec_ul_tp;
typedef stapl::vector_view<pvec_ul_tp>              pvec_ul_vw_tp;

typedef stapl::vector<vec_ul_tp>                    pvec_vec_ul_tp;
typedef stapl::vector_view<pvec_vec_ul_tp>          pvec_vec_ul_vw_tp;

typedef stapl::vector<pvec_ul_tp>                   pvec_pvec_ul_tp;
typedef stapl::vector_view<pvec_pvec_ul_tp>         pvec_pvec_ul_vw_tp;

typedef stapl::vector<pair_ul_tp>                   pvec_pair_ul_tp;
typedef stapl::vector_view<pvec_pair_ul_tp>         pvec_pair_ul_vw_tp;

typedef stapl::vector<map_pair_ul_tp>               pvec_map_pair_ul_tp;
typedef stapl::vector_view<pvec_map_pair_ul_tp>     pvec_map_pair_ul_vw_tp;

typedef stapl::vector<map_duo_ul_tp>                pvec_map_duo_ul_tp;
typedef stapl::vector_view<pvec_map_duo_ul_tp>      pvec_map_duo_ul_vw_tp;

#ifdef USE_SET
typedef stapl::unordered_set<ulong>                 pset_ul_tp;

typedef stapl::set_view<pset_ul_tp>                 pset_ul_vw_tp;
#else
typedef stapl::vector<ulong>                        pset_ul_tp;
typedef stapl::vector_view<pset_ul_tp>              pset_ul_vw_tp;
#endif

#ifdef USE_UNORD_MAP
typedef stapl::unordered_map<ulong,pset_ul_tp>      phash_ul_pset_ul_tp;
typedef stapl::unordered_map<ulong,vec_ul_tp>       phash_ul_set_ul_tp;

typedef stapl::unordered_map<pair_ul_tp, ulong,
        stapl::hash<pair_ul_tp>, stapl::equal_to<pair_ul_tp>>
                                                    phash_pair_ul_tp;
typedef stapl::map_view<phash_pair_ul_tp>           phash_pair_ul_vw_tp;

typedef stapl::unordered_map<encpair_tp, ulong,
        stapl::hash<encpair_tp>, stapl::equal_to<encpair_tp>>
                                                    phash_encpair_ul_tp;
typedef stapl::map_view<phash_encpair_ul_tp>        phash_encpair_ul_vw_tp;

typedef stapl::unordered_map<duo_ul_tp, ulong,
        stapl::hash<duo_ul_tp>, stapl::equal_to<duo_ul_tp>>
                                                    phash_duo_ul_tp;
typedef stapl::map_view<phash_duo_ul_tp>            phash_duo_ul_vw_tp;

#endif

typedef stapl::map<ulong,pset_ul_tp>                pmap_ul_pset_ul_tp;
typedef stapl::map<ulong,vec_ul_tp>                 pmap_ul_set_ul_tp;

typedef stapl::map<pair_ul_tp,ulong>                pmap_pair_ul_tp;
typedef stapl::map_view<pmap_pair_ul_tp>            pmap_pair_ul_vw_tp;

typedef stapl::map<encpair_tp,ulong>                pmap_encpair_ul_tp;
typedef stapl::map_view<pmap_encpair_ul_tp>         pmap_encpair_ul_vw_tp;

typedef stapl::map<encpair_tp,ulong>                pmap_duo_ul_tp;
typedef stapl::map_view<pmap_duo_ul_tp>             pmap_duo_ul_vw_tp;

typedef stapl::map_view<pmap_ul_pset_ul_tp>         pmap_ul_pset_ul_vw_tp;
typedef stapl::map_view<pmap_ul_set_ul_tp>          pmap_ul_set_ul_vw_tp;

typedef stapl::distribution_spec<>                  dist_spec_tp;
typedef stapl::array_view<stapl::array<stapl::arbitrary_partition_info>>
          part_info_vw_tp;
typedef stapl::map< int,set_ul_tp,std::less<int>,
                    stapl::view_based_partition<dist_spec_tp, part_info_vw_tp>,
                    stapl::view_based_mapper<dist_spec_tp> >
                                                    pmap_ul_set_ul_arb_tp;
typedef stapl::map_view<pmap_ul_set_ul_arb_tp>      pmap_ul_set_ul_arb_vw_tp;

#ifdef USE_UNORD_MAP
typedef stapl::unordered_map< int, set_ul_tp,
                    stapl::hash<ulong>, stapl::equal_to<ulong>,
                    stapl::view_based_partition<dist_spec_tp, part_info_vw_tp>,
                    stapl::view_based_mapper<dist_spec_tp> >
                                                    phash_ul_set_ul_arb_tp;
typedef stapl::map_view<phash_ul_set_ul_arb_tp>     phash_ul_set_ul_arb_vw_tp;
#endif

typedef stapl::vector<item_tp>                      pvec_item_tp;
typedef stapl::vector_view<pvec_item_tp>            pvec_item_vw_tp;

typedef stapl::vector<vec_item_tp>                  pvec_vec_item_tp;
typedef stapl::vector_view<pvec_vec_item_tp>        pvec_vec_item_vw_tp;

typedef stapl::vector<pvec_item_tp>                 pvec_pvec_item_tp;
typedef stapl::vector_view<pvec_pvec_item_tp>       pvec_pvec_item_vw_tp;

typedef stapl::vector<pair_item_tp>                 pvec_pair_item_tp;
typedef stapl::vector_view<pvec_pair_item_tp>       pvec_pair_item_vw_tp;

typedef stapl::vector<hash_ul_set_ul_tp>            pvec_hash_ul_set_ul_tp;
typedef stapl::vector_view<pvec_hash_ul_set_ul_tp>  pvec_hash_ul_set_ul_vw_tp;

typedef stapl::vector<map_ul_set_ul_tp>             pvec_map_ul_set_ul_tp;
typedef stapl::vector_view<pvec_map_ul_set_ul_tp>   pvec_map_ul_set_ul_vw_tp;

typedef stapl::vector<pair_ul_set_ul_tp>            pvec_pair_ul_set_ul_tp;
typedef stapl::vector_view<pvec_pair_ul_set_ul_tp>  pvec_pair_ul_set_ul_vw_tp;

///////////////////////////////////////////////////////////////////////////

typedef struct _eqv_cl {
  ulong weight;
  ulong begin; // first index
  ulong end; // last index + 1
  ulong proc;
  void define_type(stapl::typer& t)
  {
    t.member(weight);
    t.member(begin);
    t.member(end);
    t.member(proc);
  }
} eqv_cl_tp;

typedef stapl::vector<eqv_cl_tp>                    pvec_eqv_cl_tp;
typedef stapl::vector_view<pvec_eqv_cl_tp>          pvec_eqv_cl_vw_tp;

///////////////////////////////////////////////////////////////////////////

typedef std::vector<
          std::tuple<
            std::pair<ulong,ulong>,
            ulong, stapl::location_type>> arb_dist_tp;

extern ulong first_mask, second_mask, encode_shift, max_key;
extern ulong first_mask_list[], second_mask_list[];

///////////////////////////////////////////////////////////////////////////


ulong eclat_flat( double, double, double,
               ulong, ulong, ulong, char,
               pary_ul_tp &, pary_ul_tp &,
               stapl::stream<ifstream> &,
               stapl::stream<ofstream> &,
               stapl::stream<ofstream> &);


ulong eclat_nest( double, double, double,
                 ulong, ulong, ulong, char,
                 pary_ul_tp &, pary_ul_tp &,
                 stapl::stream<ifstream> &,
                 stapl::stream<ofstream> &,
                 stapl::stream<ofstream> &);

ulong gen_arb_dist( pvec_vec_item_vw_tp &,
                   stapl::array<stapl::arbitrary_partition_info> &,
                   stapl::stream<ofstream> & );

///////////////////////////////////////////////////////////////////////////

typedef stapl::identity<ulong> id_ul_wf;
typedef stapl::plus<ulong> add_ul_wf;

///////////////////////////////////////////////////////////////////////////
// equivalence class processing
///////////////////////////////////////////////////////////////////////////

bool eqv_cl_weight_lt( eqv_cl_tp const *, eqv_cl_tp const *);
bool eqv_cl_begin_lt( eqv_cl_tp const *, eqv_cl_tp const *);

void build_equiv_class( vec_pair_item_tp &, vector<eqv_cl_tp *> & );

void build_equiv_class( pvec_pair_item_vw_tp &, vector<eqv_cl_tp *> &,
                        stapl::stream<ofstream> & );

void build_equiv_class( vec_vec_item_tp &, vec_pair_ul_tp & );

template <typename SegView1, typename SegView2 >
void build_equiv_class( SegView1, SegView2, vector<pair_ul_tp> &,
                        stapl::stream<ofstream> & );

template <typename SegView1, typename SegView2 >
void build_equiv_class( SegView1, SegView2, pvec_pair_ul_vw_tp &,
                        stapl::stream<ofstream> & );

void schedule_classes( vector<eqv_cl_tp *> &, ulong,
                       stapl::stream<ofstream> &);

void schedule_classes( pvec_eqv_cl_vw_tp &, ulong,
                       stapl::stream<ofstream> &);

void show_equiv_class( vec_pair_item_tp &, vector<eqv_cl_tp *> &,
                       stapl::stream<ofstream> &);

void show_equiv_class( pvec_pair_item_vw_tp &, vector<eqv_cl_tp *> &,
                       stapl::stream<ofstream> &);

extern ulong weights[];
#define MAX_WEIGHT 1000

///////////////////////////////////////////////////////////////////////////
// count processing
///////////////////////////////////////////////////////////////////////////

void merge_counts(map_pair_ul_tp &, map_pair_ul_tp &);

ulong count_control_breaks( pvec_vec_item_vw_tp &, stapl::stream<ofstream> & );

/*=========================================================================
 * INPUT / OUTPUT
 *=========================================================================*/

///////////////////////////////////////////////////////////////////////////
// resize elements as specified
///////////////////////////////////////////////////////////////////////////

struct resize_vec_wf
{
  typedef void result_type;
  template <typename Elem1, typename Elem2>
  result_type operator()(Elem1 el1, Elem2 el2)
  {
    el1.resize(el2);
  }
};

///////////////////////////////////////////////////////////////////////////
// read one transaction in text format
///////////////////////////////////////////////////////////////////////////

struct read_trans_txt_wf
{
private:
  stapl::stream<ifstream> m_in;
public:
  read_trans_txt_wf(stapl::stream<ifstream> const& in)
    : m_in(in)
  { }
  typedef void result_type;
  template <typename Cont>
  result_type operator()(Cont cont)
  {
    ulong buf_size = 8192;
    char line_buf[buf_size];

    m_in.getline(line_buf, buf_size);

    ulong val_buf[buf_size/2];
    ulong val_ctr = 0;
    char *token = NULL;
    token = strtok(line_buf, " ");
    while ( token != NULL ) {
      ulong value = atoi(token);
      val_buf[val_ctr++] =value;
      token = strtok( NULL, " " );
    }

    for ( ulong i=0; i<val_ctr; i++ ) {
      cont[i] = val_buf[i];
    }
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_in);
  }
};

///////////////////////////////////////////////////////////////////////////
// read one transaction in binary format
///////////////////////////////////////////////////////////////////////////

struct read_trans_bin_wf
{
private:
  stapl::stream<ifstream> m_in;
public:
  read_trans_bin_wf(stapl::stream<ifstream> const& in)
    : m_in(in)
  { }
  typedef void result_type;
  template <typename Cont>
  result_type operator()(Cont cont)
  {
    long int count = cont.size();
    long int data_size = sizeof(double) * sizeof(char);
    std::streamsize buf_size = data_size * count;
    //char_type * char_buf = new char_type(buf_size);
    //char char_buf[buf_size];
    char char_buf[100000];

#if defined(DEBUG)
    stapl::do_once([&]() {
      cerr << "read_trans_bin_wf: item count: " << count << endl;
    });
#endif

    m_in.read(char_buf, buf_size );

    union {
      char c[8];
      double d;
    } convert;

    for ( std::streamsize i=0; i<buf_size; i+=8) {
      convert.c[0] = char_buf[i+0];
      convert.c[1] = char_buf[i+1];
      convert.c[2] = char_buf[i+2];
      convert.c[3] = char_buf[i+3];
      convert.c[4] = char_buf[i+4];
      convert.c[5] = char_buf[i+5];
      convert.c[6] = char_buf[i+6];
      convert.c[7] = char_buf[i+7];
      cont[i] = convert.d;
    }
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_in);
  }
};

///////////////////////////////////////////////////////////////////////////
// save frequent itemsets to file
// INPUT:  pvec_vec_ul_vw_tp out_item_sets_vw(out_item_sets);
///////////////////////////////////////////////////////////////////////////

struct write_freq_std_wf
{
private:
  stapl::stream<ofstream> m_out;
public:
  write_freq_std_wf(stapl::stream<ofstream> const &out)
    : m_out(out)
  { }

  typedef void result_type;
  template <typename Cont>
  result_type operator()(Cont cont)
  {
    for ( auto iter = cont.begin(); iter != cont.end(); ++iter ) {
      if ( iter != cont.begin() ) {
        m_out << ":";
      }
      ulong elem = *iter;
      m_out << elem;
    }
    m_out << endl;
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_out);
  }
};

///////////////////////////////////////////////////////////////////////////
// save frequent itemsets to file
// INPUT:  pvec_vec_ul_vw_tp out_item_sets_vw(out_item_sets);
///////////////////////////////////////////////////////////////////////////

struct write_freq_bin_wf
{
private:
  stapl::stream<ofstream> m_out;
public:
  write_freq_bin_wf(stapl::stream<ofstream> const &out)
    : m_out(out)
  { }

  typedef void result_type;
  template <typename Vec>
  result_type operator()(Vec buffer)
  {
    ulong buf_ctr= 0;
    ulong rec_lmt = buffer[buf_ctr++];
    for ( ulong rec_ctr = 0; rec_ctr < rec_lmt; rec_ctr++ ) {
      ulong elem_lmt = buffer[buf_ctr++];
      for ( ulong elem_ctr = 0; elem_ctr < elem_lmt; elem_ctr++ ) {
        if ( elem_ctr > 0 ) {
          m_out << ":";
        }
        ulong elem = buffer[buf_ctr++];
        m_out << elem;
      }
      m_out << endl;
    }
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_out);
  }
};

///////////////////////////////////////////////////////////////////////////
// equivalence class processing
// pmap_ul_pset_ul_vw_tp & vert_db_vw
// pvec_pvec_ul_vw_tp & in_item_sets_vw
// pvec_pvec_ul_vw_tp & out_item_sets_vw
// vert_db_vw, in_item_sets_vw, seg_equiv_class
///////////////////////////////////////////////////////////////////////////

// this function depends on vert_db_vw being ordered

template <typename SegView, typename RepView >
void build_equiv_class( SegView vert_db_vw, RepView in_item_sets_vw,
                        vector<pair_ul_tp> & seg_equiv_class,
                        stapl::stream<ofstream> &dbg_out ) {
#if defined(DBG_ARB_DIST)
  dbg_out << "build_equiv_class #4" << endl;
#endif

  int prev_ndx = 0, curr_ndx = 0;

  auto db_it= vert_db_vw.begin();
  int prev_link = (*db_it).first;
  db_it++;

  int prefix = in_item_sets_vw[prev_link].size()-1;

  while ( db_it != vert_db_vw.end() ) {
    int curr_link = (*db_it).first;
    db_it++;

    bool ctl_break = false;
    for ( int i=0; i<prefix; i++ ) {
      if ( in_item_sets_vw[curr_link][i] != in_item_sets_vw[prev_link][i] ) {
        ctl_break = true;
      }
    }

    if ( ctl_break ) {
      pair_ul_tp eqv_cl;
      eqv_cl.first = prev_ndx;
      eqv_cl.second = curr_ndx;
      seg_equiv_class.push_back(eqv_cl);

      prev_ndx = curr_ndx + 1;
      prev_link = curr_link;
    }
    curr_ndx++;
  }

  pair_ul_tp eqv_cl;
  eqv_cl.first = prev_ndx;
  eqv_cl.second = curr_ndx;
  seg_equiv_class.push_back(eqv_cl);

#if defined(DBG_ARB_DIST)
  dbg_out << "build_equiv_class #4 output" << endl;
  for ( ulong i=0; i<seg_equiv_class.size(); i++ ) {
    pair_ul_tp eqv_cl = seg_equiv_class[i];
    dbg_out << "( " << eqv_cl.first << ","
                    << eqv_cl.second << " )" << endl;
  }
#endif
}

