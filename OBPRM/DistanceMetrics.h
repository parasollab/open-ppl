// $Id$
/////////////////////////////////////////////////////////////////////
//
//  DistanceMetrics.h
//
//  General Description
//
//  Created
//      8/21/98  Daniel Vallejo 
//
//  Last Modified By:
//      08/24/98  Daniel Vallejo
//
/////////////////////////////////////////////////////////////////////

#ifndef DistanceMetrics_h
#define DistanceMetrics_h

#include "OBPRM.h"
#include "Sets.h"
#include "Transformation.h"
#include "Input.h"

#include <stdlib.h>
#include <stdio.h>

#include <iostream.h>
#include <fstream.h>
#include <strstream.h>
#include <iomanip.h>

#include <vector.h>


class LocalPlanners;


//---------------------------------------------------------------
//  Pre-defined Algobase Sets
//  CAUTION:  DO NOT CHANGE THE ENUMERATION ORDERS w/o CHANGING
//              SET DEFN's in "Roadmap.c"
//---------------------------------------------------------------
enum dm_predefined {    //-----------------
        S_EUCLID9,      // Scaled Euclidean s=0.9
        EUCLID,         // Euclidean
        DM_USER1};      // first user defined dm set, if any

//---------------------------------------------------------------
// Algo base information data structures
//---------------------------------------------------------------

class DM;
class DMSets;
class DistanceMetric;

class Roadmap;
class MultiBody;
class Input;

typedef double (*DMF) (MultiBody*,Cfg&,Cfg&,DM&);     	// pointer to dm function
                                    			// *NOTE* need to update
                                    			//  when params known


const int CS = 0;	// Type CS: Configuration space distance metric
const int WS = 1;	// Type WS: Workspace distance metric 

/////////////////////////////////////////////////////////////////////
//  class DM
//
//  General Description
//
/////////////////////////////////////////////////////////////////////
class DM {
  friend class DMSets;
public:
  //===================================================================
  // Constructors and Destructor
  //===================================================================
  DM();
  ~DM();

  //===================================================================
  // Operators
  //===================================================================
  DM&  operator=(const DM & _dm);
  bool operator==(const DM & _dm) const;

  //===================================================================
  // Other Methods
  //===================================================================
  char* GetName() const;
  DMF   GetDistanceMetric();
  int 	GetType() const; 
  double GetS() const;

protected:

  //===================================================================
  // Data
  //===================================================================
  char  name[80];
  DMF   distanceMetric;          // ptr to distance metric code
  EID	dmid;
  int  	type;
  double sValue;


private:
};

ostream& operator<< (ostream& _os, const DM& dm);


/////////////////////////////////////////////////////////////////////
//  class DMSets
//
//  General Description
//
/////////////////////////////////////////////////////////////////////
class DMSets : public BasicSets<DM> {
public:
  //===================================================================
  //  Constructors and Destructor
  //===================================================================
    DMSets();
    ~DMSets();

  //===================================================================
  //  Other Methods
  //===================================================================

   // Adding DMs, Making & Modifying DM sets

   int AddDM(const char* _dminfo);     // add dm(s) to universe
   int AddDMToSet(const SID _sid, const EID _dmid);
   int DeleteDMFromSet(const SID _sid, const EID _dmid);

   SID MakeDMSet(const char* dmlist);  // make an ordered set of dms,
   SID MakeDMSet(istream& _myistream); //  - add dm to universe if not there
   SID MakeDMSet(const EID _eid);
   SID MakeDMSet(const vector<EID> _eidvector);

   int DeleteDMSet(const SID _sid);

   // Getting Data & Statistics

   DM GetDM(const EID _dmid) const;
   vector<DM> GetDMs() const;
   vector<DM> GetDMSet(const SID _sid) const;
   vector<pair<SID,vector<DM> > > GetDMSets() const;


   // Display, Input, Output

   void DisplayDMs() const;
   void DisplayDM(const EID _dmid) const;
   void DisplayDMSets() const;
   void DisplayDMSet(const SID _sid) const;

   void WriteDMs(const char* _fname) const;
   void WriteDMs(ostream& _myostream) const;
   void ReadDMs(const char* _fname);
   void ReadDMs(istream& _myistream);


  //===================================================================
  //  Data
  //===================================================================
protected:
private:
};

/////////////////////////////////////////////////////////////////////
//  class DistanceMetric
//
//  General Description
//
/////////////////////////////////////////////////////////////////////
class DistanceMetric {
public:
  //===================================================================
  // Constructors and Destructor
  //===================================================================
  DistanceMetric();
  ~DistanceMetric();

  //===================================================================
  // Other Methods
  //===================================================================
  void DefaultInit();
  void UserInit(Input *,  GenerateMapNodes*, LocalPlanners*);

  double Distance(AttEnvironment *env, Cfg _c1, Cfg _c2, SID _dmsetid);

  static double EuclideanDistance(MultiBody* robot, Cfg& _c1, Cfg& _c2, DM& _dm);
  static double ScaledEuclideanDistance(MultiBody* robot, Cfg& _c1, Cfg& _c2, DM& _dm);

  //===================================================================
  // Data
  //===================================================================
  DMSets distanceMetrics;

protected:
private:
};

#endif

