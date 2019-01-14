// This stuff was extracted from Cfg.h.

#ifdef _PARALLEL
#include "views/proxy.h"
#endif


// Within class Cfg
#ifdef _PARALLEL

  public:

    //parallel connected component
    void active(bool _a) {m_active = _a;}
    bool active() const {return m_active;}
    void cc(size_t _c) {m_cc = _c;}
    size_t cc() const {return m_cc;}

    void define_type(stapl::typer& _t) {
      _t.member(m_v);
      _t.member(m_labelMap);
      _t.member(m_statMap);
      _t.member(m_robotIndex);
      _t.member(m_active);
      _t.member(m_cc);
    }

  private:

    bool m_active;
    size_t m_cc;

#endif


// Outside class Cfg
#ifdef _PARALLEL
namespace stapl {

  //////////////////////////////////////////////////////////////////////////////
  /// @TODO
  //////////////////////////////////////////////////////////////////////////////
  template <typename Accessor>
  class proxy<Cfg, Accessor> : public Accessor {

    friend class proxy_core_access;
    typedef Cfg target_t;

    public:

      //typedef target_t::parameter_type  parameter_type;
      explicit proxy(Accessor const& acc) : Accessor(acc) { }
      operator target_t() const {
        return Accessor::read();
      }

      proxy const& operator=(proxy const& rhs) {
        Accessor::write(rhs);
        return *this;
      }

      proxy const& operator=(target_t const& rhs) {
        Accessor::write(rhs);
        return *this;
      }

      int DOF() const {
        return Accessor::const_invoke(&target_t::DOF);
      }

      int PosDOF() const {
        return Accessor::const_invoke(&target_t::PosDOF);
      }

      void Write(ostream& _os) const {
        return Accessor::const_invoke(&target_t::Write, _os);
      }

      void Read(istream& _is) {
        return Accessor::const_invoke(&target_t::Read, _is);
      }

      const vector<double>& GetData() const {
        return Accessor::const_invoke(&target_t::GetData);
      }

      void SetData(vector<double>& _data) const {
        return Accessor::const_invoke(&target_t::SetData, _data);
      }

      bool GetLabel(string _label) const {
        return Accessor::const_invoke(&target_t::GetLabel, _label);
      }

      bool IsLabel(string _label) const {
        return Accessor::const_invoke(&target_t::IsLabel, _label);
      }

      bool SetLabel(string _label) const {
        return Accessor::const_invoke(&target_t::SetLabel, _label);
      }

      double GetStat(string _stat) const {
        return Accessor::const_invoke(&target_t::GetStat, _stat);
      }

      bool IsStat(string _stat) const {
        return Accessor::const_invoke(&target_t::IsStat, _stat);
      }

      void SetStat(string _stat, double _val) const {
        return Accessor::const_invoke(&target_t::SetStat, _stat,_val);
      }

      void IncStat(string _stat, double _val) const {
        return Accessor::const_invoke(&target_t::IncStat, _stat,_val);
      }

      static int GetNumOfJoints() {
        return Accessor::const_invoke(&target_t::GetNumOfJoints);
      }

      void active(bool _a) {
        return Accessor::invoke(&target_t::active, _a);
      }

      bool active() const {
        return Accessor::const_invoke(&target_t::active);
      }

      void cc(size_t _c) {
        return Accessor::invoke(&target_t::cc, _c);
      }

      size_t cc() const {
        return Accessor::const_invoke(&target_t::cc);
      }

  };
}
#endif

