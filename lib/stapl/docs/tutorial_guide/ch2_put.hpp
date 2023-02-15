class put_val_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  put_val_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Ref>
  result_type operator()(Ref val) {
    m_zout << val << " ";
  }

  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

class put_map_val_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  put_map_val_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Ref>
  result_type operator()(Ref pair) {
    typename Ref::first_reference left = pair.first;
    typename Ref::second_reference right = pair.second;
    m_zout << "{" << left << "}= " << right << "\n";
  }

  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};
