// $Id$
/**@file Cfg.h
*
*General Description
* Configuration Data Class, it has all the interface needed
* by other Motion Planning classes. Since it is abstract, it
* will have to 'ask' a helper class called CfgManager to
* provide implementation to some specific functions.
*
*@author  Guang Song
*@date 08/31/99
*/

////////////////////////////////////////////////////////////////////////////////////////////

#ifndef Cfg_h
#define Cfg_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include standard headers
#include <math.h>
#include <stdio.h>

#ifdef HPUX
#include <sys/io.h>
#endif


////////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "Vectors.h"
//#include "OBPRM.h"
#include "BasicDefns.h"

////////////////////////////////////////////////////////////////////////////////////////////
//Frowrad decalaration
class Body;
class Environment;
class CollisionDetection;
class CfgManager;
class DistanceMetric;
class CDInfo;
class Cfg;


////////////////////////////////////////////////////////////////////////////////////////////

//
/// Information about the cfg, for example the obstacle info.
//
class  InfoCfg {
public:
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //    Constructors and Destructor
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
    //@{
    
    InfoCfg():obst(NULL_INFO),tag(NULL_INFO),clearance(NULL_INFO){};
    InfoCfg(int _obst):obst(_obst),tag(NULL_INFO),clearance(NULL_INFO){};
    InfoCfg(int _obst,int _tag):obst(_obst),tag(_tag),clearance(NULL_INFO){};
    InfoCfg(int _obst,int _tag,double _cl):obst(_obst),tag(_tag),clearance(_cl){};
    
    //@}
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //    Operator Overloading
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Operator Overloading*/
    //@{
    bool operator== (const InfoCfg&tmp) const{return obst==tmp.obst && tag==tmp.tag && clearance==tmp.clearance;};
    bool operator!= (const InfoCfg&tmp) const{return obst!=tmp.obst || tag!=tmp.tag || clearance!=tmp.clearance;};
    //@}
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //    Access Method
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Access Method*/
    //@{
    void SetObst(int _obst){obst = _obst;};
    int GetObst() const {return obst;};
    void SetClearance(double _cl){clearance = _cl;};
    double GetClearance() const {return clearance;};
    //@}
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //    I/O
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O*/
    //@{
    void Write(ostream &os) const;
    void Read(istream &is);
    //@}
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //    Data
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    
    //Initialization of this member was moved to Cfg.cpp
    static const int NULL_INFO;  
    
    /**From which Obstacle this, this Cfg is generated.
    *@see GenerateMapNodes::GaussPRM, GenerateMapNodes::BasicOBPRM
    *GenerateMapNodes::OBPRM
    */
    int obst;
    int tag;
    
    /**Cleanance of this Cfg.
    *Closet distance from this Cfg to any Obstacle in Environment.s
    */
    double clearance;
};

//---------------------------------------------
/// Input/Output operators for InfoCfg
//---------------------------------------------
istream& operator>> (istream&s, InfoCfg &_c);
ostream& operator<< (ostream&s, const InfoCfg &_c);


/////////////////////////////////////////////////////////////////////////////////////////////
//
/// Information about the clearance of a cfg.
//
/////////////////////////////////////////////////////////////////////////////////////////////
class ClearanceInfo {

private:
   
    /////////////////////////////////////////////////////////////////////////////
    //
    // Data
    //
    /////////////////////////////////////////////////////////////////////////////
    /**Clearance of this Cfg.
    * Closet distance from this Cfg to any Obstacle in C-Space
    */
    double clearance;
    
    /**Direction of clearance, or "witness pair".
    *This Cfg is closest to the c-obst.
    */
    Cfg * direction;
    
    /**Flag for ApproxCSpaceClearance2(...).
    *If 0, calculate clearance as usual. (default)
    *If 1, calculate the clearance in one direction (direction).
    */
    int checkOneDirection;
    
public:
    
    /////////////////////////////////////////////////////////////////////////////
    //
    // Constructors and Destructor
    //
    /////////////////////////////////////////////////////////////////////////////  
    /**@name Constructors and Destructor*/
    //@{  
    ClearanceInfo();
    ClearanceInfo(double _clearance, Cfg * _direction);
    ClearanceInfo(double _clearance, Cfg * _direction, int _checkOneDirection);
    ClearanceInfo(Cfg * _direction, int _checkOneDirection);
    ~ClearanceInfo();
    //@}
    
    /////////////////////////////////////////////////////////////////////////////
    //
    // Access Methods
    //
    /////////////////////////////////////////////////////////////////////////////
    /**@name Access Method*/
    //@{
    double getClearance() {return clearance;};
    void setClearance(double _clearance) {clearance = _clearance;};
    
    Cfg * getDirection() {return direction;};
    void setDirection(Cfg * _direction) {direction = _direction;};
    
    int getCheckOneDirection() {return checkOneDirection;};
    void setCheckOneDirection(int _checkOneDirection) {checkOneDirection = _checkOneDirection;};
    //@}
    
};


/**
  *This class provides storage, tools, and operators for representing configuration.
  *
  *Client could constrcut Cfg instance in 3, 6 or more dimensions CSpace by choosing
  *relavant constructor. Comparasion, baisc operations (+ - * / ), or more advanced operations
  *(like weighted sum, AlmostEqual.. ) are also provided.
  *This class also provides input/output functions to read/write instances of this class
  *to many kinds of file (formats).
  *Morevoer, tools for create primitives of PRM, like random generation of Cfg for 
  *a given Workspace, and connections between Cfgs are also provided.
  *
  *However, most of methods for helping generation and connections call methods provide
  *by CfgManager or its offsprings, which are desinged for specific robots.
  *
  *@see CfgManager, Cfg_free, Cfg_free_serial, Cfg_fixed_tree, Cfg_fixed_PRR.
  */

class Cfg {
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
    
        ///Create configuration with (0.0,0.0,.....)
        Cfg();
        ///Create configuration with (x,y,z) and normalize it
        Cfg(double x,double y,double z);
        ///Create configuration with (x,y,z,roll,pitch,yaw) and normalize it
        Cfg(double x,double y,double z, double roll,double pitch,double yaw);
        ///Create configuration v2 and normalize it
        Cfg(const Vector6<double> &v2);
        ///Copy the configuration in v2 to this configuration
        Cfg(const vector<double> &v2);
        ///Copy Constructor
        //Cfg::Cfg( const Cfg & otherCfg);
        ///Destructor. Do nothing.
        ~Cfg();
    
    //@}
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Operator Ovverloading
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Operators*/
    //@{
    
        ///Return true if this and other Cfg instance have same configuration
        bool operator== (const Cfg&) const;
        ///Return true if this and other Cfg instance have different configuration
        bool operator!= (const Cfg&) const;
        /**create a new Cfg instance whose configuration is summation of this and the other Cfg.
        *The summation is done in every dimension in CSpace.
        */
        Cfg operator+ (const Cfg&) const;
        /**create a new Cfg instance whose configuration is configuration of this Cfg minus
        *the configuration of the other Cfg.
        *The substration is done in every dimension in CSpace.
        */
        Cfg operator- (const Cfg&) const;
        ///create a new Cfg instance whose configuration is complement configuration of this Cfg
        Cfg operator- () const;
        ///Scalar multiplication (just like scalar times a vector)
        Cfg operator* (double);
        Cfg operator/ (double);
    
        /**create a new Cfg instance whose configuration is weighted summation of the
        *first and the second Cfg.
        *The summation is done in every dimension in CSpace.
        *@param weight should between [0,1]. this weight is for thesecond Cfg. 
        * The weight for the first Cfg is (1-weight)
        */
        static Cfg WeightedSum(const Cfg&, const Cfg&, double weight = 0.5);
    
        ///This function calls CfgManager::AlmostEqual(this, _c)
        bool AlmostEqual(const Cfg& _c);
        ///This function calls CfgManager::isWithinResolution
        bool isWithinResolution(const Cfg &c, double positionRes, double orientationRes);
    
    //@}
    
    //===================================================================
    //  Other Methods
    //===================================================================
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    I/O
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O*/
    //@{
        ///Call InfoCfg::Write and Write in this class.
        friend ostream& operator<< (ostream&, const Cfg &);
        ///Call InfoCfg::Read and Read in this class.
        friend istream& operator>> (istream&, Cfg &);
        ///Write configuration to output stream
        void Write(ostream &os) const;
        ///Read configuration from input stream
        void Read(istream  &is);
        ///Call CfgManager::printLinkConfigurations.
        void printLinkConfigurations(Environment *env, vector<Vector6D> &cfigs) const;
        ///Call CfgManager::print_preamble_to_file.
        static void print_preamble_to_file(Environment *env, FILE *_fp, int numofCfg);
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
        ///Get internal storage of configuration
        const vector<double>& GetData() const;
        ///Call CfgManager::GetRobotCenterPosition(this)
        Vector3D GetRobotCenterPosition();
        /// Return the number of degrees of freedom for the configuration class
        static int DOFs();
        ///Return string, "Cfg_free_rigid"
        static const char* GetName();
    
        /** Return the range of a single parameter of the configuration (i.e., range of x)
        * @param param the parameter to get the range for.
        *
        * @note In the future, this function should get the range for x,y,z by the bounding box
        * Currently it assumes the range for a position parameter to be -10000 to 10000
        * and the range for an orientation parameter to be 0 to 1, which should
        * also be changed to reflect any self collision in a linked robot
        */
        pair<double,double> SingleParamRange(int param);
    
        /** Set a single parameter in the configuration (i.e., x,y,z,roll...)
        * @param param the parameter number to set.
        * @param value the value to set the parameter as
        */
        int SetSingleParam(int param, double value);
    
        /** Increment a single parameter in the configuration (i.e., x,y,z,roll...)
        * @param param the parameter number to set.
        * @param value the value to increment the parameter by.
        */
        int IncSingleParam(int param, double value);
    
        /** Retreive a single parameter in the configuration (i.e., x,y,z,roll...)
        * @param param the parameter number to retreive
        */
        double GetSingleParam(int param);
    
        /// methods for Distance Metric.
        vector<double>  GetPosition();
        vector<double>  GetOrientation();
    
        ///Call CfgManager::OrientationMagnitude(this)
        double  OrientationMagnitude();
        ///Call CfgManager::PositionMagnitude(this)
        double  PositionMagnitude();
    //@}
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Connection Related Methods : These methods are related to local planner for connecting
    //    Cfgs
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Connection Related Methods*/
        //@{
        /** pt1 & pt2 are two endpts of a line segment
        * find the closest point to the current cfg on that line segment
        * it could be one of the two endpoints of course
        */
        Cfg ClosestPtOnLineSegment(const Cfg&, const Cfg&) const;
    
        ///Call CfgManager::InvalidData
        static Cfg InvalidData();

        /**Return a Cfg whose cofiguration is d-resolution toward cfg2 from cfg1
        *in each dimension.
        */
        static Cfg c1_towards_c2(Cfg cfg1, Cfg cfg2, double d);
    
        ///Call CfgManager::GetRandomRay(incr)
        static Cfg GetRandomRay(double incr);
    
        /// for rotate-at-s Local Planner. Call CfgManager::GetMovingSequenceNodes
        vector<Cfg> GetMovingSequenceNodes(const Cfg& other, double s) const;
    
        /// methods for nodes connection. Call CfgManager::FindNeighbors
        vector<Cfg> FindNeighbors(
            Environment *env, const Cfg& increment,
            CollisionDetection *,
            int noNeighbors, 
            SID  _cdsetid, CDInfo& _cdInfo);
    
        /// methods for nodes connection. Call CfgManager::FindNeighbors
        vector<Cfg> FindNeighbors(
            Environment *env, const Cfg& goal, const Cfg& increment, 
            CollisionDetection *,
            int noNeighbors, 
            SID  _cdsetid, CDInfo& _cdInfo);
    
        ///Increase every value in this instance in each dimention by the value in _increment
        void Increment(const Cfg& _increment);
        ///Call CfgManager::IncrementTowardsGoal
        void IncrementTowardsGoal(const Cfg &goal, const Cfg &increment);
        ///Call CfgManager::FindIncrement
        Cfg  FindIncrement(const Cfg& _goal, int * n_ticks, double positionRes, double orientationRes);
        ///Call CfgManager::FindIncrement
        Cfg  FindIncrement(const Cfg& _goal, int  n_ticks);
    //@}
	static Cfg GetResolutionCfg(Environment *env);

    
    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Generation Related Methods : These methods are related to create Cfgs randomly
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Generation Related Methods*/
        //@{
        /** 
         * generates a random configuration without consideration of bounding box restrictions
         * Call CfgManager::GetRandomCfg
         */
        static Cfg GetRandomCfg(double R, double rStep);
    
        /** 
         * generates random configuration where workspace robot's CENTER OF MASS
         * is guaranteed to lie within the environment specified bounding box
         * Call CfgManager::GetRandomCfg_CenterOfMass
         */
        static Cfg GetRandomCfg_CenterOfMass(double *boundingBox);
    
        /** 
         * generates random configuration where workspace robot's EVERY VERTEX
         * is guaranteed to lie within the environment specified bounding box
         * @param maxTries Try maxTries time to rondomly generate Cfg and check if
         * every vertex is in environment specified bounding box. If
         * no this cfg could be found. The program will be abort.
         */
        static Cfg GetRandomCfg(Environment *env, int maxTries);
    
        /// ditto, but with a default number of tries (10).
        static Cfg GetRandomCfg(Environment *env);
	//
	//
	/// Generates a random configuration with approximate length
        static Cfg GetRandomCfg(Environment *env,DistanceMetric *_dm,
            SID _dmsetid,double length);
    
        /// generates random configuration and pushes it to the medial axis of the
        /// free c-space
        static Cfg GetMedialAxisCfg(Environment *_env, CollisionDetection *_cd,
            SID _cdsetid, CDInfo &_cdInfo, DistanceMetric *_dm,
            SID _dmsetid, int n);
    
        /// pushes a free node towards the medial axis
        static Cfg MAPRMfree(Cfg cfg, Environment *_env, CollisionDetection *cd,
            SID cdsetid, CDInfo& cdInfo, DistanceMetric *dm,
            SID dmsetid, int n);
    
        /// pushes a colliding node towards the free space
        static Cfg MAPRMcollision(Cfg cfg, Environment *_env, CollisionDetection *cd,
            SID cdsetid, CDInfo& cdInfo, int n);
    
        /// generates random configuration that is in Free CSpace. Call CfgManager::GetFreeRandomCfg
        static Cfg GetFreeRandomCfg(Environment *env,CollisionDetection* cd,SID _cdsetid, CDInfo& _cdInfo);
	/// generates N random configurations
	static void GetNFreeRandomCfgs(vector<Cfg> &nodes, Environment *env,
	     CollisionDetection* cd,SID _cdsetid, CDInfo& _cdInfo, int num);

    //@}
    
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Helper Methods : Bounding Box, Clearance, Collision detection, Potential
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Helper Methods */
    //@{
        /** 
         * tests whether or not robot in this configuration has every vertex inside
         * the environment specified bounding box
         */
        bool InBoundingBox(Environment *env);
    
    
        /**
         *Set Robot's Cfg as this Cfg instance and 
         *check Robot's clearance.
         *@see CollisionDetection::Clearance
         */
        double Clearance(Environment *env,CollisionDetection* cd);
    
        ///Approximate C-Space Clearance.
        double ApproxCSpaceClearance(Environment *env, 
                                     CollisionDetection *cd, 
                                     SID cdsetid, 
                                     CDInfo& cdInfo, 
                                     DistanceMetric * dm, 
                                     SID dmsetid, 
                                     int n, 
                                     bool bComputePenetration=false);
    
        /// returns the clearance and the direction via ClearanceInfo
        void ApproxCSpaceClearance2(Environment *env, 
                                    CollisionDetection *cd,
                                    SID cdsetid,
                                    CDInfo& cdInfo,
                                    DistanceMetric * dm, 
                                    SID dmsetid, 
                                    int n, 
                                    ClearanceInfo & clearInfo, 
                                    bool bComputePenetration);

	///Approximate C-Space Contact Points
	/// given an origin Cfg and a vector of directions
	/// returns the obstacle contact point for each direction from origin
	/// (contact points are in-collision)
	vector<Cfg> ApproxCSpaceContactPoints(vector<Cfg> directions,
						   Environment *_env,
						   CollisionDetection *cd,
						   SID cdsetid,
						   CDInfo &cdInfo);    

        ///Call CfgManager::ConfigEnvironment
        bool ConfigEnvironment(Environment *env);
    
        ///Call CfgManager::isCollision
        bool isCollision(Environment *env,CollisionDetection* cd, SID _cdsetid, CDInfo& _cdInfo,bool enablePenetration=true);
    
        ///Call CfgManager::isCollision
        bool isCollision(Environment *env, CollisionDetection *cd,int robot, int obs, SID _cdsetid, CDInfo& _cdInfo,bool enablePenetration=true);
    
        ///Call CfgManager::GenerateOverlapCfg
        static bool GenerateOverlapCfg(Environment *env, int robot,
            Vector3D robot_start, Vector3D robot_goal, Cfg *resultCfg);  // OBPRM and BasicOBPRM
    
        ///Call CfgManager::GenerateOverlapCfg
        static vector<Cfg> GenSurfaceCfgs4ObstNORMAL(Environment * env,
            CollisionDetection *,int obstacle, int nCfgs,
            SID _cdsetid, CDInfo& _cdInfo);
    
        /// return a configuration(conformation)'s potential. Call CfgManager::Potential
        double Potential(Environment *env) const;
    //@}
    
    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Protected Data members and Member methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

  protected:
      ///Scalar division (just like a vector divided by a scalar)
      
      /** Normalize the orientation to the some range.
      * call CfgManager's Normalize_orientation.
      */
      void Normalize_orientation(int index = -1);
      
      ///////////////////////////////////////////////////////////////////////////////////////////
      //
      //
      //    Private Data members and Member methods
      //
      //
      //////////////////////////////////////////////////////////////////////////////////////////
  private:
      
      vector<double> v;   
      
  public:
      static CfgManager * CfgHelper;  ///<Cfg_Free
      InfoCfg info;
      friend class CfgManager;
}; // class Cfg


#endif
