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

#include "Defines.h"

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
  ~DefaultWeight();
  
  // Graph.h Interface
  static double InvalidWeight();
  static double& MaxWeight(); // For Dijkstra's Alg
  
  virtual bool operator== (const DefaultWeight& tmp) const;
  virtual const DefaultWeight& operator= (const DefaultWeight& w);
  
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
};

#endif // !defined(_OBPRM_WEIGHT_H_)
