// $Id$
// Weight.h: interface for the Weight class.
//
//////////////////////////////////////////////////////////////////////
/*********************************************************************
 *@file Weight.h
 *@author Shawna Thomas
 *
 * Weight class used for edge weights.  Other weight classes should be
 * derived off of this class.
 *@date   12/28/02
 *********************************************************************/

#ifndef WEIGHT_H_
#define WEIGHT_H_

#include "CfgTypes.h"
#include <iostream>
using namespace std;

#ifdef _PARALLEL
#include "views/proxy.h"
#endif

/////////////////////////////////////////////////////////////
//
//	Weight
//
/////////////////////////////////////////////////////////////

static vector<CfgType> EMPTY = vector<CfgType>();

class DefaultWeight {
  public:

    // Constructors and Destructor
    DefaultWeight(string _lpLabel="", double _w=1, vector<CfgType>& _intermediates = EMPTY);
    virtual ~DefaultWeight();

    // Graph.h Interface
    static double InvalidWeight();
    static DefaultWeight MaxWeight(); // For Dijkstra's Alg

    virtual bool operator== (const DefaultWeight& _tmp) const;
    virtual const DefaultWeight& operator= (const DefaultWeight& _w);

    virtual DefaultWeight operator+(const DefaultWeight& _other) const ;
    virtual bool operator<(const DefaultWeight& _other) const ;

    // Read/Write values of datamember to given input/output stream.
    friend ostream& operator<< (ostream& _os, const DefaultWeight& _w);
    friend istream& operator>> (istream& _is, DefaultWeight& _w);

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


#endif
