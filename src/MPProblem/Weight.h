/* Weight class used for edge weights.  Other weight classes should be
 * derived off of this class.
 */

#ifndef WEIGHT_H_
#define WEIGHT_H_

#include <iostream>
#include <numeric>
using namespace std;

#ifdef _PARALLEL
#include "views/proxy.h"
#endif

#include "Utilities/MPUtils.h"

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
    template<class CfgType2>
    friend ostream& operator<< (ostream& _os, const DefaultWeight<CfgType2>& _w);
    template<class CfgType2>
    friend istream& operator>> (istream& _is, DefaultWeight<CfgType2>& _w);

    // Access Methods
    string GetLPLabel() const { return m_lpLabel; }
    void SetLPLabel(string _lpLabel){ m_lpLabel = _lpLabel; }
    const vector<CfgType>& GetIntermediates() const { return m_intermediates; }
    void SetIntermediates(vector<CfgType>& _intermediates){ m_intermediates = _intermediates;}

    double GetWeight() const { return m_weight; }
    double Weight() const { return GetWeight(); } //for GraphAlgo interface
    void SetWeight(double _w){ m_weight = _w; }

    bool IsChecked(int _mult) const { return m_checkedMult <= _mult; }
    void SetChecked(int _mult) { m_checkedMult = min(m_checkedMult, _mult); }
    int GetChecked() const { return m_checkedMult; }

    // Data
  protected:
    string m_lpLabel;
    double m_weight;
    vector<CfgType> m_intermediates;

    static double MAX_WEIGHT;
    int m_checkedMult;

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
DefaultWeight<CfgType>::DefaultWeight(string _lpLabel, double _w, const vector<CfgType>& _intermediates):
  m_lpLabel(_lpLabel), m_weight(_w), m_intermediates(_intermediates), m_checkedMult(numeric_limits<int>::max()){
  }

template<class CfgType>
DefaultWeight<CfgType>::~DefaultWeight(){}

template<class CfgType>
double
DefaultWeight<CfgType>::InvalidWeight(){
  return INVALID_DBL;
}

template<class CfgType>
DefaultWeight<CfgType> 
DefaultWeight<CfgType>::MaxWeight(){
  return DefaultWeight<CfgType>("INVALID", MAX_WEIGHT);
}

template<class CfgType>
bool 
DefaultWeight<CfgType>::operator==(const DefaultWeight<CfgType>& _tmp) const{
  return ( (m_lpLabel==_tmp.GetLPLabel()) && (m_weight==_tmp.GetWeight()) );
}

template<class CfgType>
const DefaultWeight<CfgType>& 
DefaultWeight<CfgType>::operator=(const DefaultWeight<CfgType>& _w){
  m_lpLabel = _w.GetLPLabel();
  m_weight = _w.GetWeight();
  m_intermediates = _w.GetIntermediates();
  m_checkedMult = _w.GetChecked();
  return *this;
}

template<class CfgType>
ostream& 
operator<<(ostream& _os, const DefaultWeight<CfgType>& _w){
  /*_os << _w.m_intermediates.size() << " ";
  for(vector<CfgType>::const_iterator cit = _w.m_intermediates.begin(); cit!= _w.m_intermediates.end(); cit++){
    _os << *cit << " ";
  }
  */
  //TODO::FIX::for now output 0 for number of intermediates, util vizmo gets updated. Then replace with the above code.
  _os << "0 ";
  _os << _w.m_weight;
  return _os;
}

template<class CfgType>
istream& 
operator>>(istream& _is, DefaultWeight<CfgType>& _w){
  int tmp;
  _is >> tmp >> _w.m_weight;
  return _is;
}

template<class CfgType>
DefaultWeight<CfgType> 
DefaultWeight<CfgType>::operator+(const DefaultWeight<CfgType>& _other) const {
  return DefaultWeight<CfgType>(m_lpLabel, m_weight+_other.m_weight);
}

template<class CfgType>
bool 
DefaultWeight<CfgType>::operator<(const DefaultWeight<CfgType>& _other) const {
  return m_weight < _other.m_weight;
}



#endif
