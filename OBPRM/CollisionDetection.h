// $Id$
////////////////////////////////////////////////////////////////////////////////////////////
/**@file CollisionDetection.h

  @author Daniel Vallejo
  @date   8/11/98
*/   
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CollisionDetection_h
#define CollisionDetection_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include standard headers

//Modified for VC
#if defined(_WIN32)
#include <strstrea.h>
#else
#include <strstream.h>
#endif

///////////////////////////////////////////////////////////////////////////////
///CD libraries
#ifdef USE_CSTK
#include <cstkSmallAPI.h>
#include <cstk_global.h>
#endif
#ifdef USE_VCLIP
#include <vclip.h>
#endif
#ifdef USE_RAPID
#include <RAPID.H>
#endif
#ifdef USE_PQP
#include <PQP.h>
#endif

//////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "OBPRM.h"
#include "Sets.h"
#include "Transformation.h"
#include "Environment.h"
#include "DistanceMetrics.h"
#include "Cfg.h"

#include <vector.h>
//////////////////////////////////////////////////////////////////////////////
class Input;
class Environment;
class GenerateMapNodes;
class ConnectMapNodes;
class Roadmap;
class MultiBody;
class CD;
class CDSets;
class CollisionDetection;

///////////////////////////////////////////////////////////////////////////////

const double MaxDist =  1e10;



/**
*Algo base information data structures.
*
*This was made into a class so I knew everything was
*initialized properly. I got tired of trying to track
*down where all the CDInfo variables were created - BD July 2000
*@author BD
*@date 07/01/00
*/
class CDInfo {
	
public:
	
	//////////////////////////////////////////////////////////////////////////////////////////
	//
	//    Constructors and Destructor
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name Constructors and Destructor*/
	//@{
	
	///Default Constrcutor. Intialize evry thing to invalid value.
	CDInfo();
	///Destructor. Do nothing.
	~CDInfo();
	
	//@}
	
	//////////////////////////////////////////////////////////////////////////////////////////
	//
	//    Helper
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name Helper Methods*/
	//@{
	
    ///Re-Init vars as done by constructor
    void ResetVars();
	//@}
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//    Data
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	
    int colliding_obst_index;   ///< The index for fisrt discovered obstacle which collides with robot.
    bool ret_all_info;          ///< Is this instance contains all (following) infomation.
    int nearest_obst_index;     ///< The index for closest obstacle
    double min_dist;            ///< Distance between Robot and closet obstacle
    Vector3D robot_point;       ///< Cloest point on Robot to closet obstacle
    Vector3D object_point;      ///< Cloest point on closet obstacle to Robot
};

///Pointer to collision dectection function.
typedef bool (*CDF) (MultiBody*,MultiBody*,CD&,CDInfo&);     

const int Out = 0;      ///<Type Out: no collision sure; collision unsure.
const int In = 1;       ///<Type In: no collision unsure; collision sure.
const int Exact = 2;    ///<Type Exact: no collision sure; collision sure.


///////////////////////////////////////////////////////////////////
/**
* class CD
*/
class CD {
	
	friend class CDSets;
	
public:
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Constructors and Destructor
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name Constructors and Destructor*/
	//@{
	
	///Default Constrcutor. Intialize evry thing to invalid value.
	CD();
	///Destructor. Do nothing.
	~CD();
	
	//@}
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Operator Overloading
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name Operator Overloading*/
	//@{
	/**Copy.
	*Copy name of CD instance, _cd, to name of this instance only.
	*@return reference of this instance.
	*/
	CD&  operator=(const CD & _cd);
	
	/**Compare between two CD instances.
	*Compare name of CD instance, _cd, to the name of this instance.
	*@return true if the name of this instance is the same as name of _cd
	*or the name of this instance is "boundingSpheres", "insideSpheres",
	*"naive", "quinlan", "cstk", "vclip", or "RAPID". Otherwise false will be 
	*returned.
	*/
	bool operator==(const CD & _cd) const;
	//@}
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Access Methods : Retrive and set related information of this class
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name Access Methods*/
	//@{
	///Get the name of this CD instance
	char* GetName() const;
	
	///Get a pointer to collision dectection function
	CDF   GetCollisionDetection();
	
	/**Get the type of this CD instance.
	*Out, In, or Exact.
	*/
	int   GetType() const; 
	//@}
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Protected Data members and Member methods
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
protected:
	
/**The name of this CD instance. 
*Could be "boundingSpheres", "insideSpheres", "naive", "quinlan",
*"cstk", "vclip", or "RAPID".
    */
	char  name[80];            
	CDF   collision_detection; ///<A pointer to collision detection function
	EID   cdid;                ///<The element id of this CD in universe.
	int   type;                ///<In, Out, or Exact. Used to classify CD functions.
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Private Data members and Member methods
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
private:
};

///Output name and type of given CD instance, cd, to ouput stream.
ostream& operator<< (ostream& _os, const CD& cd);


/////////////////////////////////////////////////////////////////////
///  CDSets
class CDSets : public BasicSets<CD> {
public:
	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Constructors and Destructor
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name Constructors and Destructor*/
	//@{
	
    ///Default Constructor. Do nothing.
    CDSets();
    ///Destructor. Do nothing.
    ~CDSets();
	
	//@}
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Adding CDs, Making & Modifying CD set
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name Adding CDs, Making & Modifying CDs set */
	//@{
	
	/**Add collision detection info to universe.
	*@note this method just adds CD instance to universe 
	*, and no set will be created.
	*@see MakeCDSet(const char* cdlist)
	*/
	int AddCD(const char* _cdinfo);
	
	/**Add element _cdid to (ordered) cd set _sid.
	*@see BasicSets::AddElementToOSet
	*/
	int AddCDToSet(const SID _sid, const EID _cdid);
	
	/**Remove element _cdid from (ordered) cd set _sid.
	*@see BasicSets::DeleteElementFromOSet
	*/
	int DeleteCDFromSet(const SID _sid, const EID _cdid);
	
	/**Read collision detection info from given string.
	*@return INVALID_SID if istrstream for this given
	*string could not be created.
	*@see MakeCDSet(istream& _myistream)
	*/
	SID MakeCDSet(const char* cdlist);  // make an ordered set of cds,
	
										/**Read collision detection info from inputstream,
										*, create CD instances for these info, and make an (ordered) CD set for these.
										*Accroding to read-in cd names, following rules are applied.
										*
										*  -# boundingSpheres CD::collision_detection = CollisionDetection::IsInCollision_boundingSpheres
										*                     CD::type = Out
										*  -# insideSpheres CD::collision_detection = CollisionDetection::IsInCollision_insideSpheres
										*                   CD::type = In
										*  -# naive CD::collision_detection = CollisionDetection::IsInCollision_naive
										*           CD::type = Exact
										*  -# quinlan CD::collision_detection = CollisionDetection::IsInCollision_quinlan
										*             CD::type = Exact
										*  -# cstk CD::collision_detection = CollisionDetection::IsInCollision_cstk
										*          CD::type = Exact
										*  -# vclip CD::collision_detection = CollisionDetection::IsInCollision_vclip
										*           CD::type = Exact
										*  -# RAPID CD::collision_detection = CollisionDetection::IsInCollision_RAPID
										*           CD::type = Exact
										*
										*@return SID of new set if every thing is OK. Otherwise, process will be terminiated.
										*@see BasicSets::MakeOSet 
	*/
	SID MakeCDSet(istream& _myistream); //  - add cd to universe if not there
	
										/**Make a new (ordered) cd set with element _eid.
										*@see BasicSets::MakeOSet(const EID _eid)
	*/
	SID MakeCDSet(const EID _eid);
	
	/**Make a new (ordered) cd cd set with a list of elements in _eidvector.
	*@see BasicSets::MakeOSet 
	*/
	SID MakeCDSet(const vector<EID> _eidvector);
	
	/**Remove a (ordered) cd set from universe.
	*@see BasicSets::DeleteOSet
	*/
	int DeleteCDSet(const SID _sid);
	
	//@}
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Access MEthod (Getting Data & Statistics)
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name Access Methods.
    *Getting Data & Statistics 
    */
	//@{
	
	/**Get a CD instance from universe.
	*@note _cdid should be added to universe before.
	*@see BasicSets::GetElement
	*/
	CD GetCD(const EID _cdid) const;
	
	/**Get all CD instances in universe.
	*@return a list of CD instances.
	*@see BasicSets::GetElements
	*/
	vector<CD> GetCDs() const;
	
	/**Get a (ordered) CD set from universe.
	*@note _sid should be created and added to universe before.
	*@see BasicSets::GetOSet(const SID _sid)
	*/
	vector<CD> GetCDSet(const SID _sid) const;
	
	/**Get all (ordered) CD set in universe.
	*@return a list of (ordered) CD sets and their SIDs.
	*Each (ordered) CD set contains a list of CD instances.
	*@see BasicSets::GetOSets
	*/
	vector<pair<SID,vector<CD> > > GetCDSets() const;
	
	//@}
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    I/O Method (Display, Input, Output)
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name I/O Methods.
	*Display, Input, Output 
	*/
	//@{
	
	/**Output all CD instances info in universe
	*to standard output.
	*@see BasicSets::DisplayElements.
	*/
	void DisplayCDs() const;
	
	/**Output CD instance info to standard output.
	*@param _cdid Specify which CD instance should be printed.
	*@see BasicSets::DisplayElement.
	*/
	void DisplayCD(const EID _cdid) const;
	
	/**Output information of all (ordered) CD sets in universe.
	*@see BasicSets::DisplayOSets
	*/
	void DisplayCDSets() const;
	
	/**Output information of (ordered) CD set with _sid.
	*@see BasicSets::DisplayOSet
	*/
	void DisplayCDSet(const SID _sid) const;
	
	/**Ouput information about all CD instances to file.
	*@param _fname filename for data file.
	*@see WriteCDs(ostream& _myostream)
	*/
	void WriteCDs(const char* _fname) const;
	
	/**Ouput information about all CD instances to ouputstream.
	*@note format: CD_NAME (a string) \n for each CD instance.
	*@see GetCDs
	*/
	void WriteCDs(ostream& _myostream) const;
	
	/**Read information about CD instances from file.
	*@param _fname filename for data file.
	*@see ReadCDs(istream& _myistream)
	*/
	void ReadCDs(const char* _fname);
	
	/**Read information about CD instances from input stream.
	*This methods read and add CD instances to universe.
	*@see AddCD for add CD instances to universe and WriteCDs 
	*for data format
	*@note if this method could not be able to understand
	*input file format, then error message will be post to 
	*standard output.
	*/
	void ReadCDs(istream& _myistream);
	
	//@}
	
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Protected Data members and Member methods
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
protected:
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Private Data members and Member methods
	//
	//
	/////////////////////////////////////////////////////////////////////////////////////////
private:
};

//////////////////////////////////////////////////////////////////////////////////////////
/**  class CollisionDetection
*/
class CollisionDetection {
public:
	
	//////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Constructors and Destructor
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name Constructors and Destructor*/
	//@{
	
	///Default Constructor. Call DefaultInit().
	CollisionDetection();
	///Do nothing.
	~CollisionDetection();
	
	//@}
	
	//////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Helper functions
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name Helper Methods*/
	//@{
	
	///Initialize data members. Do nothing.
	void DefaultInit();
	/**Initialize this instance accroding to user input.
	*By default if USE_CSTK is defined, then a CD instance
	*with name, "cstk" will be created and added to a new set.
	*If USE_VCLIP is defined, then a CD instance
	*with name, "vclip" will be created and added to a new set.
	*If USE_RAPID is defined, then a CD instance
	*with name, "RAPID" will be created and added to a new set.
	*
	*One set will be created according to user input.
	*
	*@see CDSets::MakeCDSet(istream&)
	*@note all user speicifed CD names will be put into one set.
	*/
	void UserInit(Input * input,  GenerateMapNodes*, ConnectMapNodes*);
	
	/**Set penetration depth  
	* The parameter depth defines how many times the resolution 
	* Default value is -1, no penetration
	*/
	void SetPenetration(double times);
	
	/** Check if there is a collision but it is in permissible range,
	* i.e., the penetration is within the penetration range
	*/
	bool AcceptablePenetration(Cfg c,Environment *env,CollisionDetection *cd, 
		SID cdsetid, CDInfo& cdInfo);
	
	/** Initialize n direction vectors  with the penetration length*/
	void InitializePenetration(double times,int nCfgs,  Environment *env,
		DistanceMetric * dm, SID dmsetid,double ratio=0.5);
	
	double penetration; // Penetration distance
	
#ifdef USE_CSTK
	double cstkDistance(MultiBody* robot, MultiBody* obstacle);
#endif
	
	/**Get minimum distance from Robot to Obstacles in environment.
	*@note This method could be invoked iff USE_CSTK is defined.
	*if USE_CSTK is undefined, then process will be terminiated.
	*/
	double Clearance(Environment * env);
	
	/**
	   * Check if robot in given cfg is complete inside or outside obstacle.
	   * @warning The precondition is that robot is collision free
	   * in this given cfg. (i.e no intersections among boundaries of robot and obs)
	   * @return True, if robot is completely contained inside any obs.
	   * otherwise, false will be returned.
	   */
	bool isInsideObstacle(const Cfg & cfg, Environment* env, SID _cdsetid, CDInfo& _cdInfo);
	//@}
	
	//////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Collision detection functions
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name Collision detection Methods.
    *High level collision detection.
    */
	//@{
	
	/**Check collision for Robot with all Obstacles in Environment.
	*
	*@param lineRobot If this parameter is not NULL, then
	*this MultiBody will be used as robot. Otherwise, robot's
	*MultiBody will be retrived from Environment.
	*
	*@return true if Robot collides with Obstacle(s).
	*if returned value is true, post condition of _cdInfo depends on
	*_cdInfo.ret_all_info :
	*	-# if _cdInfo.ret_all_info is false, then _cdInfo.colliding_obst_index 
	*      is the index for the first obstacle colliding with robot.
	*	-# if _cdInfo.ret_all_info is true, then _cdInfo.colliding_obst_index
	*      is the index for the first obstacle colliding with robot, and 
	*      local_cd_info.nearest_obst_index is the index for cloeset Obstacle.
	*@note if self collision of Robot is found, _cdInfo will be set to odd value.
	*@see IsInCollision(Environment* , SID , CDInfo& , MultiBody* , MultiBody*)
	*/
	bool IsInCollision
        (Environment* env, SID _cdsetid, CDInfo& _cdInfo, MultiBody* lineRobot = NULL,bool enablePenetration=true);
	
		/**Check collision between MultiBody of robot and obstacle.
        *This method using collision detection information in _cdInfo to
        *check collision.
        *@return Following rules are used to determin collision:
        *
        *   -# if CDInfo::GetType = Out and CDInfo::GetCollisionDetection returns false
        *      then return false.
        *   -# if CDInfo::GetType = In and CDInfo::GetCollisionDetection returns true
        *      then return true.
        *   -# if CDInfo::GetType = Exact then return what CDInfo::GetCollisionDetection
        *      returned.
        *
        *@see Collision detection core functions, CDInfo::GetType, and 
        *CDInfo::GetCollisionDetection
	*/
	bool IsInCollision
        (Environment* env, SID _cdsetid, CDInfo& _cdInfo, MultiBody* rob, MultiBody* obstacle);
	
		/**Check collision by index of robot and obstacle.
        *This method retrives MultiBody instances from Environment insntace,
        *then call IsInCollision(Environment* , SID , CDInfo& , MultiBody* , MultiBody* )
        *to detect collision.
        *@see IsInCollision(Environment* , SID , CDInfo& , MultiBody* , MultiBody*)
	*/
	bool IsInCollision
        (Environment* env, SID _cdsetid, CDInfo& _cdInfo, int robot, int obstacle);
	
	//@}
	
	//////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Collision detection core functions
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	/**@name Collision detection CORE functions
    *These functions are designed for different
    *collision detection library, such as CSTK, VClIP, RAPID....
    */
	//@{
	
	/**Always return true.
    *@see CDSets::MakeCDSet(istream& _myistream), for collision function type
    */
	static bool IsInCollision_boundingSpheres
		(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);
	
		/**Always return true.
		*@see CDSets::MakeCDSet(istream& _myistream), for collision function type
    */
	static bool IsInCollision_insideSpheres
		(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);
	
		/**Always return true.
		*@see CDSets::MakeCDSet(istream& _myistream), for collision function type
    */
	static bool IsInCollision_naive
		(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);
	
		/**Always return true.
		*@see CDSets::MakeCDSet(istream& _myistream), for collision function type
    */
	static bool IsInCollision_quinlan
		(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);
	
	//////////////////////////////////////////////////////////////////////////////////////////
	//
	//    CSTK
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	
#ifdef USE_CSTK
	
	/**Using CSTK to check collision between two MultiBodys.
    *Collision is checked in Body level between two MultiBodys,
    *if any of Body from Robot collides with any of Body from obstacle,
    *true will be returned.
    *
    *@note This method doesn't support "Return all info", if 
    *_cdInfo.ret_all_info is true, it's just ignored.
    *
    *@note collision between two ajacent links will be ignore.
    *@return true if Collision found. Otherwise false will be returned.
    *@see Body::GetCstkBody
    */
	static bool IsInCollision_cstk
		(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);

	/**
	 * For given cfg, check if robot is completely inside obstacles.
	 * @see isInsideObstacle
	 */
	bool isInsideObs_cstk(const Cfg & cfg, Environment* env, SID _cdsetid, CDInfo& _cdInfo);
#endif
	
	/// for cstk, used by IsInCollision_cstk
	static void SetLineTransformation(const Transformation&, double linTrans[12]); 
	
	//////////////////////////////////////////////////////////////////////////////////////////
	//
	//    VCLIP
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	
#ifdef USE_VCLIP
	
	/**Using VCLIP to check collision between two MultiBodys.
    *Collision is checked in Body level between two MultiBodys,
    *if any of Body from Robot collides with any of Body from obstacle,
    *true will be returned.
    *
    *@note This method doesn't support "Return all info", if 
    *_cdInfo.ret_all_info is true, then error message will be post.
    *@note collision between two ajacent links will be ignore.
    *@return true if Collision found. Otherwise false will be returned.
    *
    *@see Body::GetVclipBody, and GetVclipPose.
    *@see IsInColl_AllInfo_vclip for get all info. 
    */
	static bool IsInCollision_vclip
		(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);
	
		/**Get VclipPose.
		*@todo I don't really know what this is....
    */
	static VclipPose GetVclipPose(const Transformation&, const Transformation&);
	
	/**Get all collsion information for given MultiBody.
    *Collision is checked in Body level between two MultiBodys,
    *if any of Body from Robot collides with any of Body from obstacle,
    *true will be returned.
    *
    *More information about collision between two object, such as the closet points between
    *two object, closest distance... all of these information are stored in _cdInfo.
    *
    *@note each obstacle could change the results in _cdInfo
    *Trace back to general IsInCollision call to see how it all
    *gets updated correctly.
    *@see IsInCollision(Environment*, SID, CDInfo& , MultiBody*)
    */
	static bool IsInColl_AllInfo_vclip
		(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);
	
	/**
	 * For given cfg, check if robot is completely inside obstacles.
	 * @see isInsideObstacle
	 */
	bool isInsideObs_vclip(const Cfg & cfg, Environment* env, SID _cdsetid, CDInfo& _cdInfo);

#endif
	
	//////////////////////////////////////////////////////////////////////////////////////////
	//
	//    RAPID
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	
#ifdef USE_RAPID
	
	/**Using RAPID to check collision between two MultiBodys.
    *Collision is checked in Body level between two MultiBodys,
    *if any of Body from Robot collides with any of Body from obstacle,
    *true will be returned.
    *
    *@note This method doesn't support "Return all info", if 
    *_cdInfo.ret_all_info is true, then error message will be post.
    *@note if RAPID_Collide, the RAPID method, return false, process will 
    *be terminated.
    *@note collision between two ajacent links will be ignore.
    *@return true if Collision found. Otherwise false will be returned.
    *@see Body::GetRapidBody
    */
	static bool IsInCollision_RAPID
		(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);

	/**
	 * For given cfg, check if robot is completely inside obstacles.
	 * @see isInsideObstacle
	 */
	bool isInsideObs_RAPID(const Cfg & cfg, Environment* env, SID _cdsetid, CDInfo& _cdInfo);
	
#endif
	
#ifdef USE_PQP
	static bool IsInCollision_PQP
		(MultiBody* robot, MultiBody* obstacle, CD& _cd, CDInfo& _cdInfo);

	/**
	 * For given cfg, check if robot is completely inside obstacles.
	 * @see isInsideObstacle
	 */
	bool isInsideObs_PQP(const Cfg & cfg, Environment* env, SID _cdsetid, CDInfo& _cdInfo);
	PQP_Model * BuildPQPSegment(PQP_REAL dX, PQP_REAL dY, PQP_REAL dZ) const;
	PQP_Model * m_pRay;

#endif
	//@}
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Public Data
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
	CDSets collisionCheckers; ///<Storing information about for CD functions.
	CDInfo cdInfo;            ///<No one use this??!!
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//
	//    Private Data members and Member methods
	//
	//
	//////////////////////////////////////////////////////////////////////////////////////////
private:
	vector <Cfg> directions;
	double acceptableRatio;
};

#endif
