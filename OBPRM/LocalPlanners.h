// $Id$
/////////////////////////////////////////////////////////////////////
/**@file  LocalPlanners.h
  *
  *    This set of classes supports a "Local Planning Algobase".
  *
  *    The classes in the set are:
  *      o LP            -- info related to an individual local planner 
  *      o LPSets        -- contains 'master' list of all local planners
  *                         and maintains sets of local planners
  *                         (derived from BasicSets<LP>)
  *      o LocalPlanners -- has LPSets as data element, and its methods
  *                         include the actual local planning algorithms.
  *                         A 'wrapper' method cycles thru sets of lps.
  *
  *     Each LP element is given a unique id, which is used to compose
  *     labels encoding which local planners succeeded in connection.A
  *     (e.g., for storing as edges in a roadmap).
  *
  *     NOTE: Assumes that the configuration type is known (e.g, typedef'd)
  *           as "Cfg" -- this must be provided.
  *
  * @author Nancy Amato
  * @date 8/7/98
  */

#ifndef LocalPlanners_h
#define LocalPlanners_h

////////////////////////////////////////////////////////////////////////////////////////////
#include "OBPRM.h"
#include "CollisionDetection.h" //for CDINFO instance, so we can not use forward declaration.
#include "Cfg.h"                //for vector<Cfg>, so we can not use forward declaration.

////////////////////////////////////////////////////////////////////////////////////////////

/**
  *Pre-defined Algobase Sets
  *@warning  DO NOT CHANGE THE ENUMERATION ORDERS w/o CHANGING
  *          SET DEFN's in "LocalPlanners.cpp"
  *@see LocalPlanners::DefaultInit. The order for set creations
  *is the same as the order here. (i.e. the enumeration values
  *here is set id!!). (ex: SL (0) is LP set id for "straightline"
  *in LocalPlanners::DefaultInit)
  */
enum lp_predefined {
        SL,             ///< straightline
        R5,             ///< rotate at s=0.5
        SL_R5,          ///< SL and R5
        AD69,           ///< a* distance w/ 6 tries and 9 neighbors
        SL_R5_AD69,     ///< SL & R5 & AD69
        LP_USER1        ///< first user defined lp set, if any
};


enum PLANNER {
    STRAIGHTLINE,
    ROTATE_AT_S,
    ASTAR_DISTANCE,
    ASTAR_CLEARANCE,
    APPROX_SPHERES,
    INVALID_PLANNER
};

//////////////////////////////////////////////////////////////////////////////////////////
//
//
// Algo base information data structures
//
//
//////////////////////////////////////////////////////////////////////////////////////////

struct CNInfo;
class Roadmap;
class DistanceMetric;

class LPInfo {

public:
  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //    Constructors and Destructor
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name Constructors and Destructor*/
   //@{
        ///Default constructor. Do nothing.
        LPInfo() {} 

       /**Constructor with initialization.
         *This method init its value
         *accroding to given parameters.
         *
         *Initialization Rules are :
         *  -# #positionRes = from rm's Environment
         *  -# #orientationRes = from rm's Environment
         *  -# #checkCollision = true
         *  -# #savePath = false
         *  -# #saveFailedPath = false
         *  -# #cd_cntr =0 
         *  -# #cdsetid = info.cdsetid
         *  -# #dmsetid = info.dmsetid
         */
        LPInfo(Roadmap *rm, const CNInfo& cnInfo);
   //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //    Data
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    /**@name */
    //@{
    /**Indicates what distance metric set will be used.
      *@see dm_predefined for predefined ids. DMSets for more information.
      */
    SID dmsetid;
    /**Indicates what collision detector will be used.
      *@see CDSets for more information.
      */
    SID cdsetid;
    CDInfo cdInfo;
    //@}

    double positionRes;     ///< Resolution of Position.
    double orientationRes;  ///< Resolution of Orientation

    bool checkCollision;    ///< Will local planner check collision during connection time.
    bool savePath;          ///< Will generated path be save in #path.
    bool saveFailedPath;    ///< Will generated path be save in #path even if connection failed.
    int cd_cntr;

    /**@name Results*/
    //@{
    vector<Cfg> path;           ///< Path found by local planner.
    pair<WEIGHT,WEIGHT> edge;   ///< Contains weights of edges defined in #path.
    pair<Cfg,Cfg> savedEdge;    ///< Failed Edge. savedEdge.second is the position that local planner failed.
    //@}
    
};
    
/////////////////////////////////////////////////////////////////////
/**@class LP
  *
  *  This class contains information relevant to a particular 
  *  local planner (e.g., name, ptr to function, etc).
  *  
  */    

class LP;
class LPSets;
class LocalPlanners;
class Roadmap;

class LP {
  friend class LPSets;
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
      LP();
      ///Destructor. Do nothing.
      ~LP();

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

      
      /**Compare between two LP instances.
        *Compare name and sValue of DM instance, _cd, to the name of this instance.
        *@return The return value depends on names of two DM instances.
        *If names of two LP instances are different, then false is returned.
        *if names of two LP instances are same, then
        *   -# if name is straightline, ture will be returned
        *   -# if name is rotate_at_s, ture will be returned
        *   -# if name is com, ture will be returned
        *   -# if name is scaledEuclidean, sValues of two LPs are compared.
        *      return (sValue == _lp.sValue)
        *   -# if name is a_star, and tries and neighbors of 
        *      two LPs are the same then true will be return.
        *otherwise ( n==_lp.n ) will be returned.
        */
      bool operator==(const LP & _lp) const;

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

      /**Get the name of this LP instance
        *@see #name
        */
      char*  GetName() const;

      /**Get the planner of this LP instance
        *@see #planner
        */
      PLANNER GetPlanner();

      /**Get S value of this LP instance.
        *@note the name of this LP instance should be rotate_at_s.
        *@return if name of this LP instance is rotate_at_s 
        *, then sValue will be returned. Otherwise, -1 will
        *be returned.
        *@see #sValue
        */
      double GetS() const;

      /**Get tries of this LP instance.
        *@note the name of this LP instance should be a_star.
        *@return if name of this LP instance is a_star 
        *, then tries will be returned. Otherwise, -1 will
        *be returned.
        *@see #tries
        */
      int    GetTries() const;

      /**Get neighbors of this LP instance.
        *@note the name of this LP instance should be a_star.
        *@return if name of this LP instance is a_star 
        *, then neighbors will be returned. Otherwise, -1 will
        *be returned.
        *@see #neighbors
        */
      int    GetNeighbors() const;

      /**Get id of this LP in universe.
        *@see #lpid
        */
      EID    GetID() const;

      /**Get forward edge of this LP in universe.
        *@see #forwardEdge
        */
      int    GetFEdgeMask() const;

      /**Get backward edge of this LP in universe.
        *@see #backEdge
        */
      int    GetBEdgeMask() const;

      /**Get n of this LP in universe.
        *@see #n
        */
      int    GetN() const;
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
protected:
  char   name[80];  ///the name of planner that this LP represents.
  PLANNER planner;  ///which planner this LP represents.
  EID    lpid;

  /**@name Rotate at S Local Planner Parameters */
  //@{
    double sValue;  ///< Should in [0,1]. This is the s for planner.
  //@}

  /**@name A* Local Planner Parameters*/
  //@{
    int    tries;       ///< How many time will be tried to connect to goal. (not used!?)
    int    neighbors;   ///< How many neighbors will be seached abound current Cfg. (not used?!)
  //@}

  /**@name A* Local Planner Parameters*/
  //@{
    int    n;   ///< Number of times Cfg::ApproxCSpaceClearance will try to find clearance.
  //@}

  /**@name "Magic" number for planer.
    *These numbers will be bitwise-or-ed with WEIGHT::lp in LPInfo::edge
    *to "remember" which planner(s) make connection.
    *Although roadmap is not directed, planner's "Magic" number for both 
    *forward and backward should be record for planner like rotate-at-s.
    *
    *@see LocalPlanners::IsConnectedFindAll and LocalPlanners::IsConnected
    */
  //@{
    int    forwardEdge; ///< forward
    int    backEdge;    ///< backward
  //@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private Data members and Member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
private:
};

///Output name and type of given LP instance, lp, to ouput stream.
ostream& operator<< (ostream& _os, const LP& lp); 

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//
//  class LPSets
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

/**  This class is derived from BasicSets<LP>.
  *
  *  This class manages/contains:
  *     - a 'master' list of all local planners, and
  *     - sets of local planners
  *
  *  Each local planner and set is given a unique id. The lp ids (EIDs)
  *  are used to compose labels encoding local planner sets (used, e.g.,
  *  to record which local planners succeed when connecting a pair of
  *  configurations).
  */

class LPSets : public BasicSets<LP> {

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
    LPSets();
    ///Destructor. Do nothing.
    ~LPSets();

  //@}


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Adding LPs, Making & Modifying LP sets
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Adding LPs, Making & Modifying LP sets */
  //@{

       /**Add local planner info to universe.
         *@note this method just adds LP instance to universe 
         *, and no set will be created.
         *@see MakeCDSet(const char* lplist)
         */
       int AddLP(const char* _lpinfo);


       /**Add element _lpid to (ordered) LP set _sid.
         *@see BasicSets::AddElementToOSet
         */
       int AddLPToSet(const SID _sid, const EID _lpid);

       /**Read local planner info from a given string.
         *@return INVALID_SID if istrstream for this given
         *string could not be created.
         *@see MakeLPSet(istream& _myistream)
         */
       SID MakeLPSet(const char* lplist);

       /**Read collision detection info from inputstream,
         *, create DM instances for these info, and make an (ordered) DM set for these.
         *Accroding to read-in DM "names", following rules are applied.
         *
         *  -# straightline LP::planner = STRAIGHTLINE
         *     forwardEdge=backEdge.
         *
         *  -# rotate_at_s  LP::planner = ROTATE_AT_S
         *     LP::sValue = user spcified value (if no user spcified value, default is 0.5)
         *     user spcified value should between 0 and 1, otherwise process will be terminated.
         *     if user spcified value are accpted, two LP instance, lp1 and lp2,
         *     will be created. lp1.sValue=user spcified value and lp2.sValue=1-lp1.sValue.
         *     Moreover lp1.forwardEdge = lp2.backEdge and lp2.forwardEdge=lp1.backEdge.
         *     if there are more than one user values, more than one "couple" will
         *     be created for each user value.
         *     EX: rotate_at_s 0.3 0.8 (4 LP (2 couples) will be created)
         *
         *  -# a_star_clearance LP::planner = ASTAR_CLEARANCE
         *  -# a_star_distance  LP::planner = ASTAR_DISTANCE
         *     In previous and this case, 
         *     LP::tries = user spcified value (if no user spcified value, default is 6)
         *     LP::neighbors = user spcified value (if no user spcified value, default is 3)
         *     user value for LP::tries should between 1-20, if not user value will be cast
         *     to closest value 1 or 20. user value for LP::neighbors should be 3, 9 or 15.
         *     if this is not true, user value for LP::neighbors will be cast to closest 
         *     value.
         *     forwardEdge=backEdge.
         *     EX: a_star_clearance  10 3 or a_star_distance 2 15
         *
         *  -# approx_spheres LP::planner = APPROX_SPHERES
         *     LP::n = user spcified value (if no user spcified value, default is 3)
         *     user value for LP:n should be larger than or equals 0
         *     forwardEdge=backEdge.
         *     EX: approx_spheres 6
         *
         *@return SID of new set if every thing is OK. Otherwise, process will be terminiated.
         *@see BasicSets::MakeOSet 
         */
       SID MakeLPSet(istream& _myistream);

       /**Make a new (ordered) LP set with element _eid.
         *@see BasicSets::MakeOSet(const EID _eid)
         */
       SID MakeLPSet(const EID _eid);

       /**Make a new (ordered) LP cd set with a list of elements in _eidvector.
         *@see BasicSets::MakeOSet 
         */
       SID MakeLPSet(const vector<EID> _eidvector);

       /**Remove a (ordered) LP set from universe.
         *@see BasicSets::DeleteOSet
         */
       int DeleteLPSet(const SID _sid);

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Access Method (Getting Data & Statistics)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Access Methods.
    *Getting Data & Statistics 
    */
  //@{

       /**Get a LP instance from universe.
         *@note _lpid should be added to universe before.
         *@see BasicSets::GetElement
         */
       LP GetLP(const EID _lpid) const;

       /**Get all LP instances in universe.
         *@return a list of LP instances.
         *@see BasicSets::GetElements
         */
       vector<LP> GetLPs() const;

       /**Get a (ordered) LP set from universe.
         *@note _sid should be created and added to universe before.
         *@see BasicSets::GetOSet(const SID _sid)
         */
       vector<LP> GetLPSet(const SID _sid) const;

       /**Get all (ordered) LP set in universe.
         *@return a list of (ordered) LP sets and their SIDs.
         *Each (ordered) LP set contains a list of LP instances.
         *@see BasicSets::GetOSets
         */
       vector<pair<SID,vector<LP> > > GetLPSets() const;

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    I/O Method (Display, Input, Output)
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
   /**@name I/O Methods.
     *Display, Input, Output.
     */
   //@{

       /**Output all LP instances info in universe
         *to standard output.
         *@see BasicSets::DisplayElements.
         */
       void DisplayLPs() const;

       /**Output LP instance info to standard output.
         *@param _lpid Specify which LP instance should be printed.
         *@see BasicSets::DisplayElement.
         */
       void DisplayLP(const EID _lpid) const;

       /**Output information of all (ordered) LP sets in universe.
         *@see BasicSets::DisplayOSets
         */
       void DisplayLPSets() const;

       /**Output information of (ordered) LP set with _sid.
         *@see BasicSets::DisplayOSet
         */
       void DisplayLPSet(const SID _sid) const;

       /**Ouput information about all LP instances to file.
         *@param _fname filename for data file.
         *@see WriteLPs(ostream& _myostream)
         */
       void WriteLPs(const char* _fname) const;

       /**Ouput information about all LP instances to output stream.
         *@note format: LP_NAME (a string) LP_PARMS (double, int, etc) 
         *for each LP instance.
         *  -# if rotate_at_s is encountered, then LP::sValue will be 
         *     printed.
         *  -# if a_star_distance or a_star_clearance is encountered, 
         *     LP::tries, and LP::neighbors will be printed.
         *
         *@see GetLPs 
         */
       void WriteLPs(ostream& _myostream) const;

       /**Read information about LP instances from file.
         *@param _fname filename for data file.
         *@see ReadLPs(istream& _myistream)
         */
       void ReadLPs(const char* _fname);

       /**Read information about LP instances from input stream.
         *This method reads and adds LP instances to universe.
         *@see AddLP for add LP instances to universe and WriteLPs 
         *for data format
         *@note if this method could not be able to understand
         *input file format, then error message will be post to 
         *standard output.
         */
       void ReadLPs(istream& _myistream);

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
//
//
//
//
//
//  class LocalPlanners
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
/** This is the main local planner class. 
  * It has an LPSets object
  * as a data element (list of lps and lp sets), and its methods
  * include the actual local planning algorithms:
  * - straightline
  * - rotate_at_s, 0 <= s <= 1
  * - a_star_distance and a_star_clearance, 
  *   1 <= tries <= 20
  *   neighbors = 3, 9, or 15
  *
  * 'Wrapper' methods cycle thru sets of lps, stopping upon 1st
  * success or trying all.
  */

class LocalPlanners {

   friend SID MakeLPSet(istream&); 

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
      LocalPlanners();
      ///Destructor. Currently do nothing.
      ~LocalPlanners();

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

      /**Initialize default values for local planners.
        *Following LP Sets are created for each line.
        *   -# straightline 
        *      (This set contains straightline only).
        *   -# rotate_at_s 
        *      (This set contains rotate_at_s only).
        *   -# straightline rotate_at_s 
        *      (This set contains straightline and rotate_at_s)
        *   -# a_star_distance 6 9 
        *      (This set contains a_star_distance only)
        *   -# straightline rotate_at_s a_star_distance 6 9
        *      (This set contains straightline, rotate_at_s, 
        *       and a_star_distance)
        *
        *@see LPSets::MakeLPSet(istream&)
        */
      virtual void DefaultInit();

      /**Initialize this instance accroding to user input.
        *Sets will be created according to user input strings.
        *(one string for one set)
        *Moreover, lineSegmentLength and usingClearance
        *are init from Input::lineSegment 
        *and Input::usingClearance.
        *
        *@see LPSets::MakeLPSet(istream&), lineSegmentLength, and 
        *usingClearance
        */
      virtual void UserInit(Input *input,  ConnectMapNodes*);
 
      /**Check if there are other planners in set which is
        *different from given planner.
        *
        *@param plannerName A string (name) of local planner.
        *@param lpsetid Set ID for specific LDSet.
        *@return if there are planner(s) other than
        *this given planner found in LDSet, lpsetid, 
        *true will be returned. Otherwise, false is returned.
        *(this means "plannerName" is the only planner in this set)
        */
      virtual bool UsesPlannerOtherThan(char plannerName[], SID lpsetid=0);
  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Local Planner Connection functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Local Planner Connection Methods.
    *High level conntion methods.
    */
  //@{

      /**Check if given 2 Cfgs could be connected.
        *This method won't check if or not this edge is
        *in roadmap. This method trys to connect given Cfgs
        *and add "new" edge to graph.
        *Moreover, this method stops when first connection is found, and
        *the planner that makes connection will be record in info (LPInfo::edge).
        *
        *@return true if connection is made. Otherwise false will be returned.
        */
      virtual bool IsConnected(Environment *env,
                               CollisionDetection *,
                               DistanceMetric *,
                               Cfg _c1, 
                               Cfg _c2, 
                               SID _lpsetid, 
                               LPInfo *info);

      /**Check if given 2 Cfgs could be connected.
        *This method checks if the edge is already in graph.
        *if so, return true immediately.
        *
        *Moreover, this method stops when first connection is found, and
        *the planner that makes connection will be record in info (LPInfo::edge).
        *
        *@return true if connection is made. Otherwise false will be returned.
        */
      virtual bool IsConnected(Roadmap * rm,
                               CollisionDetection *,
                               DistanceMetric *,
                               Cfg _c1,
                               Cfg _c2,
                               SID _lpsetid,
                               LPInfo *info);

      /**Find all possible LPs in the set that can make the connection.
        *The method will try all local planners enven if connection between
        *two given Cfgs is found.
        *Planners that make connection will be record in info (LPInfo::edge).
        *
        *@return true if connection is made. Otherwise false will be returned.
        */
      virtual bool IsConnectedFindAll(Environment *env,
                                      CollisionDetection *,
                                      DistanceMetric *,
                                      Cfg _c1, 
                                      Cfg _c2, 
                                      SID _lpsetid, 
                                      LPInfo *info);
  //@}

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    CORE Local Planner Connection functions
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name CORE Local Planner Connection functions.
    *These fucntion provides specific local planner.
    *Generalized form of LP functions, replace 'pointer 
    *to function' way doing this, but fulfill the same purpose. 
    *In addition, it eases deriving classes
    */
  //@{

      /**Dispatch request to specific connection method by PLANNER instance.
        *@param lpName Could be STRAIGHTLINE, ROTATE_AT_S, ASTAR_DISTANCE
        *APPROX_SPHERES.
        *@note Follow rules says who this method dispatches to other methods:
        *   -# STRAIGHTLINE to IsConnected_straightline
        *   -# ROTATE_AT_S to IsConnected_rotate_at_s
        *   -# ASTAR_DISTANCE to IsConnected_astar
        *   -# ASTAR_CLEARANCE to IsConnected_astar
        *   -# APPROX_SPHERES to IsConnected_approx_spheres
        *   if this method does not know how to dispatch, exit will be called.
        */
      virtual bool IsConnected(PLANNER lpName, Environment *env,CollisionDetection *,DistanceMetric *,
                                    Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);

      /**Check if two Cfgs could be connected by straight line.
        *This method implements straight line connection local planner
        *by checking collision of each Cfg along the line.
        *If the is any Cfg causes Robot collides with any obstacle,
        *false will be returned.
        *
        *@note if usingClearance is true, then the call will
        *be redirect to IsConnected_SLclearance
        *
        *@param env Used for isCollision
        *@param dm Used for IsConnected_SLclearance
        *@param _c1 start Cfg
        *@param _c2 goal Cfg
        *@param _lp Used for IsConnected_SLclearance 
        *@param info Used to record path information, such
        *as length of path...
        *
        *@return true if all Cfg are collision free.
        *@see IsConnected_straightline and Cfg::FindIncrement.
        */
      virtual bool IsConnected_straightline_simple(Environment *env,CollisionDetection *cd,
                    DistanceMetric * dm, Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);


      /**Check if two Cfgs could be connected by straight line with certain cleanrance.
        *This method uses binary search to check clearances of Cfgs between _c1 and 
        *_c2. If any Cfg with clearance less than 0.001 was found, false will be returned.
        *
        *@return true if no Cfg whose cleanrance is less than 0.001. Otherwise, false will be returned.
        */
      virtual bool IsConnected_SLclearance(Environment *env,CollisionDetection *,DistanceMetric *,
                                    Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);

      /**Check if the line, _c1->_c2, collides with any obstacle or not.
        *This is done by create a MultiBody, which is a triangle approximating
        *line segment _c1->_c2, and check collision of this triangle and other
        *obstacles in envronment.
        *@return True if there is collision. Following cases return false:
        *   -# the distance between _c1 and _c2 are shorter than lineSegmentLength
        *      (?maybe because this is too short so more expensive methods are used?)
        *   -# There is not collision between this triangle and other obstacles.
        *
        *@see CollisionDetection::IsInCollision
        */
      virtual bool lineSegmentInCollision(Environment *env,CollisionDetection *,DistanceMetric *,
                                    Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);

      /**Check if two Cfgs could be connected by straight line.
        *   -# First, if lineSegmentLength is not zero, 
        *      this method calls lineSegmentInCollision to make sure 
        *      _c1 and _c2 could see each other. If the answer is No,
        *      false will be returned. Otherwise, goes step 2.
        *   -# Second, IsConnected_straightline_simple is called
        *      and return what it returns.
        *
        *@return See description above.
        *@see lineSegmentInCollision and IsConnected_straightline_simple
        */
      virtual bool IsConnected_straightline(Environment *env,CollisionDetection *,DistanceMetric *,
                    Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);

      /**Check if two Cfgs could be connected by 3 straight lines in C-Space.
        *These 3 straight lines are translation line in position subspace,
        *rotation line in orientation subspace, and another translation line 
        *in position subspace.
        *These subspaces are subspaces of CSpace.
        *@see Cfg::GetMovingSequenceNodes and IsConnected_straightline_simple.
        */
      virtual bool IsConnected_rotate_at_s(Environment *env,CollisionDetection *,DistanceMetric *,
                    Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);

      /**Check if two Cfgs could be connected by A* strategy.
        *This method handles two local planner options, ASTAR_DISTANCE and ASTAR_CLEARANCE.
        *The only difference between these two is the way to choose next neighbor from current
        *Cfg. If ASTAR_DISTANCE is used, then the next neighbor has shorter distance to
        *goal than other do. If ASTAR_CLEARANCE is used, then the next neighbor has
        *largest clearance than others do.
        *
        *Following is Algorithm for A* used in this method.
        *   -# Set currentCfg = startCfg
        *   -# while currentCfg != goalCfg
        *       -# Increase currentCfg to goalCfg direction, called testCfg
        *       -# if testCfg is collision free
        *           -# currentCfg = testCfg
        *       -# else get neighbors of  currentCfg
        *           -# return fail if there is no collision free neighbors
        *           -# find best neighbor, bestN (depends on ASTAR_DISTANCE or ASTAR_CLEARANCE)
        *           -# currentCfg = bestN
        *       -# end if
        *       -# if tried to many times, return fail.
        *   -# end while
        *   -# return success
        *    
        *@see Cfg::FindNeighbors ,Cfg::Clearance, and DistanceMetric::Distance.
        */
      virtual bool IsConnected_astar(Environment *env,CollisionDetection *,DistanceMetric *,
                    Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);

      /**Roughly check if two Cfgs could be connected using clearance.
        *Algorithm is given here:
        *   -# set clearance1 as clearance for _c1
        *   -# set clearance2 as clearance for _c2
        *   -# set dist as distance from _c1 to c2
        *   -# if clearance1+clearance2 > dist
        *       -# connected
        *   -# else
        *       -# not connected
        *
        *@see Cfg::ApproxCSpaceClearance and Cfg::Clearance
        */
      virtual bool IsConnected_approx_spheres(Environment *env,CollisionDetection *,DistanceMetric *,
                                    Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);

  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  LPSets planners;              ///< Record sets of local planners, both defaul and user specified.
  static cd_predefined cdtype;  ///< Used for building line segment. (lineSegmentInCollision)
  static int lineSegmentLength; ///< default is 0.

  /**If true, local planner will check clearances of all Cfgs along path. 
    *(default is false)
    */
  static bool usingClearance;

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
  //////////////////////////////////////////////////////////////////////////////////////////
private:
};


#endif
