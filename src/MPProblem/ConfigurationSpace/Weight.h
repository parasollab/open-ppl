#ifndef WEIGHT_H_
#define WEIGHT_H_

#include <iostream>
#include <numeric>
using namespace std;

#ifdef _PARALLEL
#include "views/proxy.h"
#endif

#include "Utilities/MPUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Weights
/// @brief Default weight class for roadmap edges. Defined as a value and a set
///        of intermediate configurations.
/// @tparam CfgType Cfg type
///
/// Weight is the concept for what is stored on the graph edges. Essentially,
/// edges are defined as polygonal chains \f$I={q_1, q_2, \ldots, q_n}\f$
/// through @cspace. They have two essential properties, a weight value
/// representing some idea of distance between the two end points of the edge
/// and a set of intermediate configurations defining the polygonal chain (not
/// including the start and goal configurations).
////////////////////////////////////////////////////////////////////////////////
template<class CfgType>
class DefaultWeight {
  public:

    // Constructors and Destructor
    DefaultWeight(string _lpLabel="", double _w=1, const vector<CfgType>& _intermediates = vector<CfgType>());
    virtual ~DefaultWeight();

    // Graph.h Interface
    static double InvalidWeight();
    static DefaultWeight MaxWeight(); // For Dijkstra's Alg

    virtual bool operator== (const DefaultWeight& _tmp) const;
    virtual const DefaultWeight& operator= (const DefaultWeight& _w);

    virtual DefaultWeight operator+(const DefaultWeight& _other) const ;
    virtual bool operator<(const DefaultWeight& _other) const ;

    // Read/Write values of datamember to given input/output stream.
    template<class C>
      friend ostream& operator<<(ostream& _os, const DefaultWeight<C>& _w);
    template<class C>
      friend istream& operator>>(istream& _is, DefaultWeight<C>& _w);

    // Access Methods
    string GetLPLabel() const { return m_lpLabel; }
    void SetLPLabel(string _lpLabel){ m_lpLabel = _lpLabel; }
    vector<CfgType>& GetIntermediates() { return m_intermediates; }
    const vector<CfgType>& GetIntermediates() const { return m_intermediates; }
    void SetIntermediates(vector<CfgType>& _intermediates){ m_intermediates = _intermediates;}

    double GetWeight() const { return m_weight; }
    double Weight() const { return GetWeight(); } //for GraphAlgo interface
    void SetWeight(double _w){ m_weight = _w; }

    bool IsChecked(int _mult) const { return m_checkedMult <= _mult; }
    void SetChecked(int _mult) { m_checkedMult = min(m_checkedMult, _mult); }
    int GetChecked() const { return m_checkedMult; }

    bool HasClearance() const {return m_hasClearance;}
    double GetClearance() const {return m_clearance;}
    void SetClearance(double _c){m_hasClearance=true; m_clearance = _c;}

    string GetStat(string _stat);
    bool IsStat(string _stat);
    void SetStat(string _stat, string _value);

    virtual void Read(istream& _is);
    virtual void Write(ostream& _os) const;

    // Data
  protected:
    string m_lpLabel;
    double m_weight;
    vector<CfgType> m_intermediates;
    static double MAX_WEIGHT;
    int m_checkedMult;
    bool m_hasClearance;
    double m_clearance;

    map<string,string> m_statMap;

  public:
    //changed local to member
#ifdef _PARALLEL
    void define_type(stapl::typer &t)
    {
      t.member(m_weight);
      t.member(m_lpLabel);
      t.member(m_intermediates);
    }
#endif
};

template<class CfgType>
double DefaultWeight<CfgType>::MAX_WEIGHT = numeric_limits<double>::max();

template<class CfgType>
DefaultWeight<CfgType>::
DefaultWeight(string _lpLabel, double _w,
    const vector<CfgType>& _intermediates) :
  m_lpLabel(_lpLabel), m_weight(_w), m_intermediates(_intermediates),
  m_checkedMult(numeric_limits<int>::max()), m_hasClearance(false) {
  }

template<class CfgType>
DefaultWeight<CfgType>::
~DefaultWeight() {
}

template<class CfgType>
double
DefaultWeight<CfgType>::
InvalidWeight() {
  return -1;
}

template<class CfgType>
DefaultWeight<CfgType>
DefaultWeight<CfgType>::
MaxWeight() {
  return DefaultWeight<CfgType>("INVALID", MAX_WEIGHT);
}

template<class CfgType>
bool
DefaultWeight<CfgType>::
operator==(const DefaultWeight<CfgType>& _tmp) const {
  return ( (m_lpLabel==_tmp.GetLPLabel()) && (m_weight==_tmp.GetWeight()) );
}

template<class CfgType>
const DefaultWeight<CfgType>&
DefaultWeight<CfgType>::
operator=(const DefaultWeight<CfgType>& _w) {
  m_lpLabel = _w.GetLPLabel();
  m_weight = _w.GetWeight();
  m_intermediates = _w.GetIntermediates();
  m_checkedMult = _w.GetChecked();
  m_hasClearance = _w.HasClearance();
  m_clearance = _w.GetClearance();
  m_statMap = _w.m_statMap;
  return *this;
}

template<class CfgType>
ostream&
operator<<(ostream& _os, const DefaultWeight<CfgType>& _w) {
  _w.Write(_os);
  return _os;
}

template<class CfgType>
istream&
operator>>(istream& _is, DefaultWeight<CfgType>& _w) {
  _w.Read(_is);
  return _is;
}

template<class CfgType>
DefaultWeight<CfgType>
DefaultWeight<CfgType>::
operator+(const DefaultWeight<CfgType>& _other) const {
  return DefaultWeight<CfgType>(m_lpLabel, m_weight+_other.m_weight);
}

template<class CfgType>
bool
DefaultWeight<CfgType>::
operator<(const DefaultWeight<CfgType>& _other) const {
  return m_weight < _other.m_weight;
}

template<class CfgType>
string
DefaultWeight<CfgType>::
GetStat(string _stat) {
  if(IsStat(_stat))
    return m_statMap[_stat];
  else
    throw RunTimeException(WHERE, "Cannot find Stat '" + _stat + "'.");
}

template<class CfgType>
bool
DefaultWeight<CfgType>::
IsStat(string _stat) {
  return m_statMap.count(_stat) > 0;
}

template<class CfgType>
void
DefaultWeight<CfgType>::
SetStat(string _stat, string _value) {
  m_statMap[_stat] = _value;
}

template<class CfgType>
void
DefaultWeight<CfgType>::
Read(istream& _is) {
  size_t numIntermediates;
  _is >> numIntermediates;
  m_intermediates.clear();
  CfgType tmp;
  for(size_t i = 0; i < numIntermediates; ++i) {
    _is >> tmp;
    m_intermediates.push_back(tmp);
  }
  _is >> m_weight;
}

template<class CfgType>
void
DefaultWeight<CfgType>::
Write(ostream& _os) const {
  _os << m_intermediates.size() << " ";
  for(auto&  cfg : m_intermediates)
    _os << cfg;
  _os << m_weight;
}

#endif
