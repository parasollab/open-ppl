// $Id$

/**
 * @file DistanceMetrics.h
 *
 * @author Daniel Vallejo
 * @date 8/21/1998
 */

////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DistanceMetrics_h
#define DistanceMetrics_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include standard headers

///Modified for VC
#if defined(_WIN32)
#include <strstrea.h>
#else
#include <strstream.h>
#endif

//////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers

#include "OBPRM.h"
#include "Sets.h"

/////////////////////////////////////////////////////////////////////////////////////////
class LocalPlanners;
class GenerateMapNodes;
class Cfg;
class DM;
class DMSets;
class MultiBody;
class Input;
class Environment;
/////////////////////////////////////////////////////////////////////////////////////////

//---------------------------------------------------------------
/**Pre-defined Algobase Sets
  *
  *@warning  DO NOT CHANGE THE ENUMERATION ORDERS w/o CHANGING
  *          SET DEFN's in "Roadmap.cpp"
  */
//---------------------------------------------------------------
enum dm_predefined 
{
        S_EUCLID9,      ///<Scaled Euclidean s=0.9
        EUCLID,         ///<Euclidean
        MINKOWSKI,      ///<Euclidean
        MANHATTAN,      ///<Manhattan
        COM,			///<Center of Mass
        DM_USER1		///<first user defined dm set, if any
};


//////////////////////////////////////////////////////////////////////////////////////////
//
//
// Algo base information data structures
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**@name Algo base information data structures*/
//@{

/**Pointer to dm function.
  *@note need to update when params known
  */
typedef double (*DMF) (MultiBody*,Cfg&,Cfg&,DM&);


const int CS = 0;	///< Type CS: Configuration space distance metric
const int WS = 1;	///< Type WS: Workspace distance metric 

//@}


class DM {

  friend class DMSets;

public:

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //	Constructors and Destructor
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Constructors and Destructor*/
   //@{

	  ///Default Constrcutor. Intialize evry thing to invalid value.
	  DM();

	  ///Destructor. Do nothing.
	  ~DM();

   //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //	Operator Overloading
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Operator Overloadings*/
   //@{

	  /**Copy.
	    *Copy name of DM instance, _dm, to name of this instance only.
		*@return reference of this instance.
	    */
	  DM&  operator=(const DM & _dm);

	  /**Compare between two DM instances.
	    *Compare name and sValue of DM instance, _cd, to name of this instance.
		*@return The return value depends on names of two DM instances.
		*If names of two DM instances are different, then false is returned.
		*if names of two DM instances are same, then
		*	-# if name is euclidean, ture will be returned
		*	-# if name is manhattan, ture will be returned
		*	-# if name is com, ture will be returned
		*	-# if name is scaledEuclidean and the difference of sValue of two
		*      DM is less than 0.000000001, true will be return.
		*	-# if name is minkowski and the differences of all ri of two
		*      DM are less than 0.000000001, true will be return.
		*otherwise false will be returned.
	    */
	  bool operator==(const DM & _dm) const;

   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Access Methods : Retrive and set related information of this class
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods*/
  //@{

	  /**Get the name of this DM instance
	    *@see DM::name
	    */
	  char* GetName() const;

	  ///Get a pointer to distance metric function
	  DMF   GetDistanceMetric();

	  /**Get the type of this DM instance.
	    *The value of type is WS or CS.
	    */
	  int 	GetType() const; 

	  /**Get SValue.
	    *@see DM::sValue
	    */
	  double GetS() const;

	  /**Get R1.
	    *@see DM::r1
	    */
	  double GetR1() const;

	  /**Get R2.
	    *@see DM::r2
	    */
	  double GetR2() const;

	  /**Get R3.
	    *@see DM::r3
	    */
	  double GetR3() const;
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

  /**The name of this DM instance. 
    *Could be "euclidean", "manhattan", "com", 
	*"scaledEuclidean", or "minkowski".
    */
  char  name[80];
  DMF   distanceMetric; ///<Pointer to a distance metric function
  EID	dmid;			///<The element id of this DM in universe
  int  	type;			///<WS or CS. Used to classify DM functions.

  /**@name Scaled Euludean*/
  //@{
    /**Scale for Euludean distance of position part. 
	  *Should between [0,1].
	  *@see DistanceMetric::ScaledEuclideanDistance
	  */
	double sValue;
  //@}

  /**@name Minkowski Variables
    *Power factors for Minkowski Distace.
	*@see DistanceMetric::MinkowskiDistance
    */
  //@{
	  double r1;	///<For position part.
	  double r2;	///<For Rotation part.
	  double r3;	///<for calcualting Root
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Private Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
};

///Output name and type of given DM instance, cd, to ouput stream.
ostream& operator<< (ostream& _os, const DM& dm);


class DMSets : public BasicSets<DM> {

public:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Constructors and Destructor*/
  //@{

	///Default Constructor. Do nothing.
    DMSets();
	///Destructor. Do nothing.
    ~DMSets();

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Adding DMs, Making & Modifying DM sets
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Adding DMs, Making & Modifying DM sets */
  //@{

	   /**Add distance metric info to universe.
	     *@note this method just adds DM instance to universe 
		 *, and no set will be created.
	     *@see MakeDMSet(const char* _dminfo)
	     */
	   int AddDM(const char* _dminfo);

	   /**Add element _dmid to (ordered) DM set _sid.
	     *@see BasicSets::AddElementToOSet
	     */
	   int AddDMToSet(const SID _sid, const EID _dmid);

	   /**Remove element _dmid from (ordered) DM set _sid.
	     *@see BasicSets::DeleteElementFromOSet
	     */
	   int DeleteDMFromSet(const SID _sid, const EID _dmid);

	   /**Read distance metric info from a given string.
	     *@return INVALID_SID if istrstream for this given
		 *string could not be created.
		 *@see MakeCDSet(istream& _myistream)
	     */
	   SID MakeDMSet(const char* dmlist);  /// make an ordered set of dms,

	   /**Read distance metric info from inputstream,
	     *, create DM instances for these info, and make an (ordered) DM set for these.
		 *Accroding to read-in DM "names", following rules are applied.
		 *
		 *	-# euclidean DM::distanceMetric = DistanceMetric::EuclideanDistance
		 *               DM::type = CS
		 *	-# scaledEuclidean DM::distanceMetric = DistanceMetric::ScaledEuclideanDistance
		 *     DM::type = CS
		 *     DM::sValue = user spcified value (if no user spcified value, default is 0.5)
		 *	-# minkowski DM::distanceMetric = DistanceMetric::MinkowskiDistance
		 *     DM::type = CS
		 *     DM::r1 = user spcified value (if no user spcified value, default is 3)
		 *     DM::r2 = user spcified value (if no user spcified value, default is 3)
		 *     DM::r3 = user spcified value (if no user spcified value, default is 1/3)
		 *	-# manhattan DM::distanceMetric = DistanceMetric::ManhattanDistance
		 *               DM::type = CS
		 *	-# com DM::distanceMetric = DistanceMetric::CenterOfMassDistance
		 *         DM::type = WS
		 *
		 *@return SID of new set if every thing is OK. Otherwise, process will be terminiated.
	     *@see BasicSets::MakeOSet 
	     */
	   SID MakeDMSet(istream& _myistream); /// add dm to universe if not there

	   /**Make a new (ordered) DM set with element _eid.
	     *@see BasicSets::MakeOSet(const EID _eid)
	     */
	   SID MakeDMSet(const EID _eid);

	   /**Make a new (ordered) DM cd set with a list of elements in _eidvector.
		 *@see BasicSets::MakeOSet 
	     */
	   SID MakeDMSet(const vector<EID> _eidvector);

	   /**Remove a (ordered) DM set from universe.
	     *@see BasicSets::DeleteOSet
	     */
	   int DeleteDMSet(const SID _sid);

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Access Method (Getting Data & Statistics)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods.
	*Getting Data & Statistics 
	*/
  //@{

	   /**Get a DM instance from universe.
	     *@note _dmid should be added to universe before.
	     *@see BasicSets::GetElement
	     */
	   DM GetDM(const EID _dmid) const;

	   /**Get all DM instances in universe.
	     *@return a list of DM instances.
		 *@see BasicSets::GetElements
	     */
	   vector<DM> GetDMs() const;
	   
	   /**Get a (ordered) DM set from universe.
	     *@note _sid should be created and added to universe before.
		 *@see BasicSets::GetOSet(const SID _sid)
	     */
	   vector<DM> GetDMSet(const SID _sid) const;
	   
	   /**Get all (ordered) DM set in universe.
	     *@return a list of (ordered) DM sets and their SIDs.
		 *Each (ordered) DM set contains a list of DM instances.
	     *@see BasicSets::GetOSets
	     */
	   vector<pair<SID,vector<DM> > > GetDMSets() const;

   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	I/O Method (Display, Input, Output)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name I/O Methods.
	 *Display, Input, Output.
	 */
   //@{

	   /**Output all DM instances info in universe
	     *to standard output.
	     *@see BasicSets::DisplayElements.
	     */
	   void DisplayDMs() const;

	   /**Output DM instance info to standard output.
	     *@param _dmid Specify which DM instance should be printed.
	     *@see BasicSets::DisplayElement.
	     */
	   void DisplayDM(const EID _dmid) const;
	   
       /**Output information of all (ordered) DM sets in universe.
	     *@see BasicSets::DisplayOSets
	     */
	   void DisplayDMSets() const;
	   
       /**Output information of (ordered) DM set with _sid.
	     *@see BasicSets::DisplayOSet
	     */
	   void DisplayDMSet(const SID _sid) const;

	   /**Ouput information about all DM instances to file.
	     *@param _fname filename for data file.
		 *@see WriteDMs(ostream& _myostream)
	     */
	   void WriteDMs(const char* _fname) const;

	   /**Ouput information about all DM instances to output stream.
	     *@note format: DM_NAME (a string) DM_PARMS (double, int, etc) 
		 *for each DM instance.
		 *if scaledEuclidean is encountered, then sValue will be printed.
		 *if minkowski is encountered, r1, r2, and r3 will be printed.
		 *@see GetDM 
	     */
	   void WriteDMs(ostream& _myostream) const;

	   /**Read information about DM instances from file.
	     *@param _fname filename for data file.
		 *@see ReadDMs(istream& _myistream)
	     */
	   void ReadDMs(const char* _fname);

	   /**Read information about DM instances from input stream.
	     *This method reads and adds DM instances to universe.
		 *@see AddDM for add DM instances to universe and WriteDMs 
		 *for data format
		 *@note if this method could not be able to understand
		 *input file format, then error message will be post to 
		 *standard output.
	     */
	   void ReadDMs(istream& _myistream);

   //@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Private Data members and Member methods
  //
  //
  /////////////////////////////////////////////////////////////////////////////////////////
private:

};

///  class DistanceMetric
///
///  General Description

class DistanceMetric {

public:

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Constructors and Destructor*/
  //@{

	  ///Default Constructor. Call DefaultInit().
	  DistanceMetric();
 	  ///Do nothing.
	  ~DistanceMetric();

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Helper functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Helper Methods*/
  //@{
      /**Initialize default values for distance metrics.
	    *Following DM Sets are created for each element.
	    *	-# scaledEuclidean 0.9
		*	-# euclidean
		*	-# minkowski 3 3 0.3333
		*	-# manhattan
		*	-# com
		*@see DMSets::MakeDMSet(istream&)
		*/
	  virtual void DefaultInit();

	  /**Initialize this instance accroding to user input.
	    *Sets will be created according to user input strings.
		*(one string for one set)
		*@see DMSets::MakeDMSet(istream&)
	    */
	  virtual void UserInit(Input *,  GenerateMapNodes*, LocalPlanners*);

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Distance functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Distance Methods.
    *High level Distance metrics
	*/
  //@{

	 /**Get Distance between two Configurations use metrics in specified
	   *DM Set.
	   *The distance is calculated by Distance Metric functions
	   *in DMs of specified DM Set.
	   *
	   *@param _dmsetid Specify which DM Set should be used in universe.
	   *@param _c1 Start Cfg
	   *@param _c2 End Cfg
	   *@return distance metrics between _c1 and _c2.
	   *@see DMSets::GetDistanceMetric and CORE Distance Methods for specific 
	   *matrics calculation.
	   *
	   *@note from souce code, it seems, if there are more than one DM in a
	   *DMSet, although all metric functions in this DMSet are called , 
	   *only the result of last metric function will be returned?! Returned
	   *values from previous calls are just ...forget?
	   *
	   */
	 virtual double Distance(Environment *env, Cfg _c1, Cfg _c2, SID _dmsetid);

  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	CORE Distance functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name CORE Distance Methods.
    *These fucntion provides specific distance metrics.
	*/
  //@{

  /**This method calculates 
    *sqrt((c11-c21)^2+(c12-c22)^2+(c13-c23)^2....+(c1n-c2n)^2).
	*
	*Here c1i and c2i are elements for each dimension and both of them
	*have n dimension. 
	*
    *@see Cfg::PositionMagnitude and OrientationMagnitude
    */
  static double EuclideanDistance(MultiBody* robot, Cfg& _c1, Cfg& _c2, DM& _dm);

  /**This method calculates 
    *sqrt(s*(Position Magnitude)^ + (1-s)*(Orientation Magnitude)^2)
	*Position Magnitude is eulidean distance of position part of dimensions.
	*Position Magnitude is eulidean distance of orientation part of dimensions.
	*Usually first 3 dimensions are positions and rest of them are orientations.
	*
	*EuclideanDistance is a special case of ScaledEuclideanDistance.
	*
	*@param _dm this parameter contains SValue, which is the scale, s, above.
    *@see Cfg::PositionMagnitude and OrientationMagnitude
    */
  static double ScaledEuclideanDistance(MultiBody* robot, Cfg& _c1, Cfg& _c2, DM& _dm);

  /**This method calculates 
    *pow( (c11-c21)^r1+(c12-c22)^r1+(c13-c23)^r1+(c14-c24)^r2....+(c1n-c2n)^r2 , r3)
	*
	*Here c1i and c2i are elements for each dimension and both of them
	*have n dimension. 
	*Usually first 3 dimensions are positions and rest of them are orientations.
	*position part using r1 as power factor, and orientation part using r2 as power factor.
	*usually, r1=r2 and r3=1/r1.
	*
	*@param _dm This parameter contains r1, r2 and r3 needed in above equation.
    */
  static double MinkowskiDistance(MultiBody* robot, Cfg& _c1, Cfg& _c2, DM& _dm);

  /**This method calculates 
    *( |c11-c21|+|c12-c22|+...+|c1n-c2n| ).
	*
	*Here |A| is absolute value of A.
    */
  static double ManhattanDistance(MultiBody* robot, Cfg& _c1, Cfg& _c2, DM& _dm);

  /**This method calculates
    *sqrt((c11-c21)^2+(c12-c22)^2+(c13-c23)^2).
    *This method only Euclidean Distance of position part and 
	*assumed that the first 3 dimension of Cfg are for position.
    */
  static double CenterOfMassDistance(MultiBody* robot, Cfg& _c1, Cfg& _c2, DM& _dm);

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  DMSets distanceMetrics; ///<Storing information for DM functions.

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Private Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
};

#endif

