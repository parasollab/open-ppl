///////////////////////////////////////////////////////////////////////////
// types only for nested parallelism
///////////////////////////////////////////////////////////////////////////

typedef stapl::vector<pvec_pvec_ul_tp>              pvec_pvec_pvec_ul_tp;
typedef stapl::vector_view<pvec_pvec_pvec_ul_tp>    pvec_pvec_pvec_ul_vw_tp;

typedef stapl::array<size_t>                        ary_sz_tp;
typedef stapl::array<ary_sz_tp>                     ary_2_sz_tp;

typedef stapl::array_view<ary_sz_tp>                ary_sz_vw_tp;
typedef stapl::array_view<ary_2_sz_tp>              ary_2_sz_vw_tp;

///////////////////////////////////////////////////////////////////////////
// nested parallel methods
///////////////////////////////////////////////////////////////////////////

void initialize_nest(double,ulong,ulong,
                     pvec_pvec_item_vw_tp &,
                     pvec_pair_item_vw_tp &,
                     stapl::stream<ofstream> &);

void prepare_vertical(double,ulong,ulong,
                    pvec_pvec_item_vw_tp &, 
                    pvec_pair_item_vw_tp &,
                    pvec_vec_item_vw_tp &,
                    vector<eqv_cl_tp *> &,
                    stapl::stream<ofstream> &);

void transform_nest( pvec_pvec_item_vw_tp &, 
                     pmap_ul_pset_ul_vw_tp &,
                     pvec_pair_item_vw_tp &,
                     vector<eqv_cl_tp *> &,
                     pvec_vec_item_vw_tp &,
                     stapl::stream<ofstream> &);

void asynchronous_nest(double,ulong,ulong,
                       pmap_ul_pset_ul_vw_tp &,
                       pvec_vec_item_vw_tp &, 
                       pvec_pvec_pvec_ul_vw_tp &,
                       pvec_ul_vw_tp &,
                       stapl::stream<ofstream> &);

void aggregate_nest(ulong, pvec_pvec_ul_vw_tp &,
                    stapl::stream<ofstream> &);

void show_itemset_2(stapl::stream<ofstream> &, pvec_pair_ul_vw_tp);

///////////////////////////////////////////////////////////////////////////
// resize elements as specified
///////////////////////////////////////////////////////////////////////////

struct inner_resize_wf
{
  typedef void result_type;
  template <typename Elem1, typename Elem2>
  result_type operator()(Elem1 el1, Elem2 el2)
  {
    auto cont = el1;
    cont.resize(el2);
  }
};

struct outer_resize_wf
{
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 v1, View2 v2) const
  {
    stapl::map_func( inner_resize_wf(), v1, stapl::make_repeat_view(v2));
  }
};

///////////////////////////////////////////////////////////////////////////
// fill elements as specified
///////////////////////////////////////////////////////////////////////////

struct inner_fill_wf
{
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 len, View2 val) const
  {
    len = val;
  }
};

struct outer_fill_wf
{
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 v1, View2 v2) const
  {
    stapl::map_func( inner_fill_wf(), v1, stapl::make_repeat_view(v2));
  }
};

///////////////////////////////////////////////////////////////////////////
// set processing for nested parallelism
///////////////////////////////////////////////////////////////////////////

struct set_copy_wf
{
  typedef void result_type;
  template <typename Elem, typename View1>
  void operator()(Elem u_elem, View1 & z_set) {
    z_set.push_back(u_elem);
  }
};

struct set_union_wf
{
  typedef void result_type;
  template <typename Elem, typename View>
  result_type operator()(Elem v_elem, View & z_set) {
    // is right not in result?
    if ( z_set.end() == z_set.find(v_elem) ) {
      z_set.push_back(v_elem);
    }
  }
};

struct set_intersect_wf
{
  typedef int result_type;
  template <typename View1, typename Elem, typename View3>
  result_type operator()(View1 u_set, Elem v_elem, View3 & z_set) {
    // is right in left?
    if ( z_set.end() != u_set.find(v_elem) ) {
      z_set.push_back(v_elem);
    }
  }
};

///////////////////////////////////////////////////////////////////////////
// display one transaction in text format
///////////////////////////////////////////////////////////////////////////

struct show_horz_stapl_wf
{
private:
  stapl::stream<ofstream> m_out;
public:
  show_horz_stapl_wf(stapl::stream<ofstream> const &out)
    : m_out(out)
  { }
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1)
  {
    m_out << "{" << vw1.size() << "}: ";
#if 0
    stapl::serial_io(put_val_wf(m_out), vw1);
#else
    for ( auto iter = vw1.begin(); iter != vw1.end(); ++iter ) {
      ulong elem = *iter;
      m_out << elem << " ";
    }
    m_out << endl;
#endif
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_out);
  }
};

///////////////////////////////////////////////////////////////////////////
// display vertical database
// INPUT:  pmap_ul_pset_ul_vw_tp vert_db_vw(vert_db);
///////////////////////////////////////////////////////////////////////////

struct show_vert_stapl_wf
{
private:
  stapl::stream<ofstream> m_out;
public:
  show_vert_stapl_wf(stapl::stream<ofstream> const &out)
    : m_out(out)
  { }
  typedef void result_type;
  template <typename Pair, typename RepView, typename CountView>
  result_type operator()(Pair pair, RepView in_itemsets_vw, CountView ctr)
  {
    typename Pair::first_reference key = pair.first;
    typename Pair::second_reference val = pair.second;

    m_out << " [" << ctr << "] ";

    auto key_vec = in_itemsets_vw[key];
    //stapl::serial_io(put_val_wf(m_out,':'), key_vec); // FIXME

    auto val_vec = val;
    //m_out << " (" << val_vec.size() << ") ";
    m_out << " @ { ";
    //stapl::serial_io(put_val_wf(m_out,','), val_vec); // FIXME
    m_out << " } " << endl;
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_out);
  }
};




