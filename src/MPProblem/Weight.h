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

#if !defined(_OBPRM_WEIGHT_H_)
#define _OBPRM_WEIGHT_H_
using namespace std;
#include <iostream>
#include "views/proxy.h"
//#include "Defines.h"



/////////////////////////////////////////////////////////////
//
//	Weight
//
/////////////////////////////////////////////////////////////

class DefaultWeight {
 public:
  
  // Constructors and Destructor
  DefaultWeight();
  DefaultWeight(int lpID);
  DefaultWeight(int lpID, double w);
  virtual ~DefaultWeight();
  
  // Graph.h Interface
  static double InvalidWeight();
  static DefaultWeight MaxWeight(); // For Dijkstra's Alg
  
  virtual bool operator== (const DefaultWeight& tmp) const;
  virtual const DefaultWeight& operator= (const DefaultWeight& w);
   
  virtual DefaultWeight operator+(const DefaultWeight& _other) const ;
  virtual bool operator<(const DefaultWeight& _other) const ;
  
  // Read/Write values of datamember to given input/output stream.
  virtual inline void Output(ostream& out) const;
  friend ostream& operator<< (ostream& _os, const DefaultWeight& w);
  virtual inline void Input(istream& in);
  friend istream& operator>> (istream& _is, DefaultWeight& w);
  
  // Access Methods
  int GetLP() const { return lp; }
  void SetLP(int lpID){ lp = lpID; }

  double GetWeight() const { return weight; }
  double Weight() const { return GetWeight(); } //for GraphAlgo interface
  void SetWeight(double w){ weight = w; }
  
  // Data
 protected:
  int lp;
  double weight;

  static double MAX_WEIGHT;
  
   public:
//changed local to member
#ifdef _PARALLEL
    void define_type(stapl::typer &t)  
    {
      t.member(weight);
      t.member(lp);
    }
#endif
};


#endif // !defined(_OBPRM_WEIGHT_H_)
