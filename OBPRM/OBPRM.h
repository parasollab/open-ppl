// $Id$
/////////////////////////////////////////////////////////////////////
/**@file OBPRM.h
  *This is a set of definitions useful to the OBPRM application specifically.
  *Almost all the OBPRM specific files will need 
  *to include them.
  *
  *@author Lucia K. Dale
  *@date  8/25/98
  */
/////////////////////////////////////////////////////////////////////

#ifndef OBPRM_h
#define OBPRM_h

#ifdef HPUX
#include <sys/io.h>
#endif
#include "BasicDefns.h"

//-----------------------------------
// Constants
//---------------------------------
/** @name Constants used in OBPRM.*/
//@{
#define ORIENTATION_RES             0.05
#define POSITION_RES_FACTOR         0.05

#define NULL_WT_INFO -999              ///< to pad weight fields for graph conversions

#define INVALID_SID -999
#define INVALID_EID -999

#define INVALID_DM -999                 ///< invalid dm id
#define INVALID_CD -999                 ///< invalid cd id
#define INVALID_LP -999                 ///< invalid local planner id
#define INVALID_GN -999                 ///< invalid generate node id
#define INVALID_CN -999                 ///< invalid connect node id
//@}

/** @name NMA: for edge weights.
  * @warning Not good values or placement (in BasicDefns?).
  */
//@{
#define MAX_INT  999999999
#define INVALID_INT -999
#define MAX_DBL  999999999
#define INVALID_DBL -999
//@}

/**@name General data structures*/
//@{
typedef short SID;///< set id type
typedef short EID;///< element id type
//@}

//-----------------------------------
// c-space representations
//-----------------------------------
#include "Cfg.h"

//-----------------------------------
// choose represention for edge weights in roadmap graph, some samples
//-----------------------------------

/**Represention for edge weights in roadmap graph.
  *"normal" edges.
  */
class IntWeight { 
public:

  /**@name Constructor and Destructor*/
  //@{
  
  /**Default contructor.
    *This constructor sets nticks to 1 and other data members to invalid value.
    */
  IntWeight(){ lp = INVALID_LP; nticks=1; };
  /**Constrcutor with local planner id.
    *@param i The ID of local planner??.
    *@note Set nticks to 1.
    */
  IntWeight(int i){ lp=i; nticks=1; };
  
  /**Construct with local planner id and weight.
    *@param i The ID of local planner??.
    *@param j For nticks.
    */
  IntWeight(int i, int j){ lp=i; nticks=j; };
  
  ///Destructor
  ~IntWeight(){};
  //@}

  /**@name Operator overloading*/
  //@{

  ///Copy values from antoher given IntWeight instance.
  bool operator== (const IntWeight &tmp) const{return ((lp==tmp.lp)&&(nticks==tmp.nticks)) ;};
  
  ///Output values of datamember to given output stream.
  friend ostream& operator<< (ostream& _os, const IntWeight& w);  // in util.c
  
  ///Read values of datamember to given input stream.
  friend istream& operator>> (istream& _is, IntWeight& w);        // in util.c
  //@}

  /**@name Access methods*/
  //@{
  
  /**Create a new IntWeight instance with invalide weight value and return it.
    *@return A IntWeight instance with invalide weight value.
    */
  static IntWeight InvalidWeight() { return IntWeight(INVALID_LP); };
  
  static double MaxWeight() { return MAX_DBL; }; ///< For Dijkstra's Alg
  
  double& Weight() { return nticks; };
  
  int& LP() { return lp; };
  //@}

private:
  int lp;
  double nticks;
};

/**Represention for edge weights in roadmap graph.
  *"special" edges, for example, for protein folding.
  */
class DblWeight {
public:

  /**@name Constructor and Destructor*/
  //@{
  
  /**Default contructor.
    *This constructor sets its data members to invalid value.
    */
  DblWeight(){ lp=INVALID_LP; weight = INVALID_DBL; };
  /**Constrcutor with local planner id.
    *@param i The ID of local planner??.
    *@note Other datamembers are set to invalide values.
    */
  DblWeight(int i){ lp=i; weight=INVALID_DBL; };
  /**Construct with local planner id and weight.
    *@param i The ID of local planner??.
    *@param j Weight.
    */
  DblWeight(int i, double j){ lp=i; weight=j; };
  
  ///Destructor
  ~DblWeight(){};
  //@}
  
  /**@name Operator overloading*/
  //@{
  
  ///Copy values from antoher given DblWeight instance.
  bool operator== (const DblWeight &tmp) const{return ((lp==tmp.lp)&&(weight==tmp.weight)); };
  
  ///Output values of datamember to given output stream.
  friend ostream& operator<< (ostream& _os, const DblWeight& w);  // in util.c
  
  ///Read values of datamember to given input stream.
  friend istream& operator>> (istream& _is, DblWeight& w);        // in util.c  
  //@}
  
  /**@name Access methods*/
  //@{
  
  /**Create a new DblWeight instance with invalide weight value and return it.
    *@return A DblWeight instance with invalide weight value.
    */
  static DblWeight InvalidWeight() { return DblWeight(INVALID_LP); };
  
  static double MaxWeight() { return MAX_DBL; }; ///< For Dijkstra's Alg
  
  double& Weight() { return weight; };
  
  int& LP() { return lp; };
  //@}

private:
  int    lp;
  double weight;
};


//-----------------------------------
// now, actually choose weight type 
//-----------------------------------
#ifndef WEIGHT
typedef IntWeight WEIGHT;
#endif


#endif
