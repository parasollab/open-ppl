//#define FIXED_MAP_PAIR 1

///////////////////////////////////////////////////////////////////////////

void initialize_flat(double,ulong,ulong,
                     pvec_vec_item_vw_tp &, 
                     pvec_pair_item_vw_tp &,
                     stapl::stream<ofstream> &);

void prepare_vertical(double,ulong,ulong,
                    pvec_vec_item_vw_tp &,
                    pvec_pair_item_vw_tp &, 
                    pvec_vec_item_vw_tp &,
#ifdef FIXED_MAP_PAIR
#ifdef USE_UNORD_MAP
                    phash_duo_ul_vw_tp &,
#else
                    pmap_duo_ul_vw_tp &,
#endif
#else
#ifdef USE_UNORD_MAP
                    phash_encpair_ul_vw_tp &,
#else
                    pmap_encpair_ul_vw_tp &,
#endif
#endif
                    vector<eqv_cl_tp *> &,
                    stapl::stream<ofstream> &);

void transform_flat( pvec_vec_item_vw_tp &,
                   pvec_pair_ul_set_ul_vw_tp &,
#ifdef USE_UNORD_ARB_MAP
                   phash_ul_set_ul_arb_vw_tp &,
#else
                   pmap_ul_set_ul_arb_vw_tp &,
#endif
                   vector<eqv_cl_tp *> &,

#ifdef FIXED_MAP_PAIR
#ifdef USE_UNORD_MAP
                   phash_duo_ul_vw_tp &,
#else
                   pmap_duo_ul_vw_tp &,
#endif
#else
#ifdef USE_UNORD_MAP
                   phash_encpair_ul_vw_tp &,
#else
                   pmap_encpair_ul_vw_tp &,
#endif
#endif
                   stapl::stream<ofstream> &);

void asynchronous_flat(double,ulong,ulong,
#ifdef USE_UNORD_ARB_MAP
                       phash_ul_set_ul_arb_vw_tp &,
#else
                       pmap_ul_set_ul_arb_vw_tp &,
#endif
                       pvec_vec_item_vw_tp &,
                       pvec_vec_item_vw_tp &,
                       stapl::stream<ofstream> &);

void aggregate_flat( pvec_vec_item_vw_tp & , 
                     pvec_vec_item_vw_tp & ,
                     stapl::stream<ofstream> &  ) ;

///////////////////////////////////////////////////////////////////////////
// display one transaction in text format
///////////////////////////////////////////////////////////////////////////

struct show_horz_len_wf
{
private:
  stapl::stream<ofstream> m_out;
public:
  show_horz_len_wf(stapl::stream<ofstream> const &out)
    : m_out(out)
  { }
  typedef void result_type;
  template <typename Cont, typename Ctr>
  result_type operator()(Cont cont, Ctr ctr)
  {
    m_out << cont.size() << ",";
    if ( 0== ((ctr+1) % 20 ) ) {
       m_out << endl;
    }
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_out);
  }
};

struct show_horz_val_wf
{
private:
  stapl::stream<ofstream> m_out;
public:
  show_horz_val_wf(stapl::stream<ofstream> const &out)
    : m_out(out)
  { }
  typedef void result_type;
  template <typename Cont>
  result_type operator()(Cont cont)
  {
    for ( auto iter = cont.begin(); iter != cont.end(); ++iter ) {
      ulong elem = *iter;
      m_out << elem << ",";
    }
    m_out  << endl;
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_out);
  }
};

///////////////////////////////////////////////////////////////////////////
// display vertical database
///////////////////////////////////////////////////////////////////////////

struct show_vert_std_wf
{
private:
  stapl::stream<ofstream> m_out;
  ulong m_count;
public:
  show_vert_std_wf(stapl::stream<ofstream> const &out, ulong count=0)
    : m_out(out), m_count(count)
  { }
 
  typedef void result_type;
  template <typename Pair, typename RepView, typename CountView>
  result_type operator()(Pair pair, RepView in_itemsets_vw, CountView ctr)
  {
    ulong ndx = pair.first;
    vec_ul_tp tidset = pair.second;

    stapl::do_once([&]() {
      m_out << " [" << ctr << "] ";
      vec_item_tp itemset = in_itemsets_vw[ndx];
      for ( vec_item_tp::iterator vec_it = itemset.begin();
           vec_it != itemset.end(); ++vec_it )  {
        if( vec_it != itemset.begin() ) {
          m_out << ":";
        }
        m_out << *vec_it;
      }

      ulong elm = 0;
      m_out << " @ { ";
      vec_ul_tp::iterator set_it = tidset.begin();
      for ( ; set_it != tidset.end(); ++set_it )  {
        if( m_count == 0 || elm < m_count ) {
          m_out << *set_it << ",";
          elm++;
          if( 0 == elm%10 ) {
            m_out << endl;
          }
        } else {
          m_out << " ... ";
          break;
        }
      }
      m_out << " } " << endl;
    });

  }

  void define_type(stapl::typer& t)
  {
    t.member(m_out);
  }
};
