#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include "boost/shared_ptr.hpp"

#include "MPProblem/Boundary.h"
#include "MPProblem/Robot.h"
#include "MPProblem/Geometry/MultiBody.h"
#include "Utilities/MPUtils.h"
#include "Graph.h"
#include "GraphAlgo.h"

using boost::shared_ptr;

class MultiBody;

class Environment {
  public:
    //m_booundary must be set later
    Environment();

    //m_boundary will also be set to _bb
    Environment(shared_ptr<Boundary> _bb);

    /** 
     * Copy Constructor.
     * copies multibodies from usable_multibodies of _env and
     * updates usable_multibodies accordingly.
     */
    Environment(const Environment &_env);

    /** 
     * Copy Constructor.
     * uses _boundary instead of boundaries in _env
     */
    Environment(const Environment &_env, const Boundary &_boundary);

    ///\brief Constructor taking in an XML object
    Environment(XMLNodeReader& _node);

    /**
     * Destructor.
     * Free memory for every added MultiBody instance if this instance
     * was not copied from another one.	
     */
    virtual ~Environment();

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Access Methods : Retrive and set related information of this class
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Access Methods*/
    //@{

    string& GetEnvFileName(){return m_filename;}
    void SetEnvFileName(string _filename){m_filename = _filename;}

    //////////////////////////////////////////////////////////////////////////////////////////
    //
    //  MultiBody
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    ///Return the number of MultiBodies in the environment
    virtual size_t GetUsableMultiBodyCount() const;

    //Returns the number of active MultiBodies in the environment
    virtual size_t GetActiveBodyCount() const;

    ///Return the number of External Bodies in the environment;
    virtual size_t GetExternalBodyCount() const;

    /**Return a pointer to MultiBody according to this given index.
     *If this index is out of the boundary of list, NULL will be returned.
     *@param _index the index for target, a MultiBody pointer
     */
    virtual shared_ptr<MultiBody> GetMultiBody(size_t _index) const;

    ///Return the number of navigable surfaces in the environment;
    virtual size_t GetNavigableSurfacesCount() const;
    virtual size_t GetRandomNavigableSurfaceIndex();
    /**Return a pointer to MultiBody in m_navigableSurfaces given index.
     *If this index is out of the boundary of list, NULL will be returned.
     *@param _index the index for target, a MultiBody pointer
     */
    virtual shared_ptr<MultiBody> GetNavigableSurface(size_t _index) const;

    /*
    GetRandomObstacle: Will get a random obstacle from m_usableMultibodies that is not an active body
    */

    shared_ptr<MultiBody> GetRandomObstacle() const;

    void PrintOptions(ostream& out_os);
    //////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Resolution.
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    /**Return the resolution for translation.
     *This tells client how fine this workspace is descretized about translation.
     *@see Get, this function reads this value from Input instance.
     *@note FindBoundingBox also calculates defalut Position Resolution. If there is no
     *user input to overwrite this default value, default value will be returned. 
     */
    inline double GetPositionRes() const { return positionRes; }
    inline void SetPositionRes(const double pRes) {positionRes=pRes;}

#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
    inline double GetRdRes() const {return rd_res;} 
#endif
    /**Return the resolution for rotation.
     *This tells client how fine this workspace is descretized about rotation.
     *@see Get, this function reads this value from Input instance.
     */
    inline double GetOrientationRes() const { return orientationRes; }
    inline void SetOrientationRes(const double rRes) {orientationRes=rRes;}

    void ComputeResolution(double _posRes, double _oriRes, 
        double _posResFactor, double _oriResFactor, size_t _numJoints = 0);


    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Sort External obstacles operations
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    //============================================
    //SortBodies so that the external bodies appear first in the array
    //============================================
    void SortMultiBodies();

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Bounding Box operations
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Bounding Box Methods*/
    //@{

    virtual void ResetBoundingBox(double _d, size_t _robotIndex);
    /**Return a Bounding Box that encloses all MultiBodys added to this instance.
     *@see FindBoundingBox, Input::bbox_scale, and Input::bbox
     */
    virtual shared_ptr<Boundary> GetBoundary() const;
    virtual void SetBoundary(shared_ptr<Boundary> _b);

    /**Return maximum axis range of bounding box.
     *This value is calculated in FindBoundingBox.
     */
    virtual double Getminmax_BodyAxisRange();
    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    IO functions
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O Methods*/
    //@{

    /**Read data from Environment file and check version.
     */
    void Read(string _filename);
    void BuildCDstructure(cd_predefined cdtype);

    //BuildRobotStructure, builds a robot graph which determines DOFs for a given robot
    //In an environment with multiple active bodies, for now this function will assume they all have the same DOFs
    //until PMPL is changed later to support multiple roadmaps for heterogeneous systems. That is, this function assumes
    //that if there is a multiagent sim going on, the agents are homogenous
    void BuildRobotStructure(); 
    vector<Robot>& GetRobots(){return robotVec;} 
    void SetRobots(const vector<Robot>& _rV) { robotVec=_rV; }
    /**Write the Input data for an environment into a given output stream.
     *2 things are output, Number of MultiBodys, and information about MultiBodys.
     *@see MultiBody::Write
     */
    virtual void Write(ostream & _os);

    //@}

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Helper functions
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    //@todo make private
    virtual void SelectUsableMultibodies();

    bool operator==(const Environment& _rhs) const;
    bool operator!=(const Environment& _rhs) const { return !(*this == _rhs); }

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Protected Data member and member methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

  protected:
    vector<shared_ptr<MultiBody> > m_activeBodies;
    vector<shared_ptr<MultiBody> > m_otherMultiBodies;
    vector<shared_ptr<MultiBody> > m_usableMultiBodies;
    size_t m_usableExternalbodyCount;

    vector<shared_ptr<MultiBody> > m_navigableSurfaces;

    //vector<int> m_robotIndices; //robot indices for multi-robot systems
    shared_ptr<Boundary> m_boundaries;

#if (defined(PMPReachDistCC) || defined(PMPReachDistCCFixed))
    double rd_res;
#endif
    double positionRes;
    double orientationRes;
    double positionResFactor;
    double orientationResFactor;
    double minmax_BodyAxisRange;    
    typedef stapl::sequential::graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES, size_t> RobotGraph;
    RobotGraph m_robotGraph;
    vector<Robot> robotVec;
    string m_filename;
};

///////////////////////////////////////////////////////////////////////////////////////////
//
//  implemetation of Environment
//
//////////////////////////////////////////////////////////////////////////////////////////

//===================================================================
///  Inline functions
//===================================================================

//-------------------------------------------------------------------
///  GetExternalBodyCount
///  Output: the number of MultiBodies in the environment
//-------------------------------------------------------------------

inline size_t 
Environment::
GetExternalBodyCount() const {
  return m_usableExternalbodyCount;
}


//-------------------------------------------------------------------
///  GetUsableMultiBodyCount
///  Output: the number of usable MultiBodies in the environment
//-------------------------------------------------------------------
inline size_t 
Environment::
GetUsableMultiBodyCount() const {
  return m_usableMultiBodies.size();
}

//-------------------------------------------------------------------
///  GetActiveBodyCount
///  Output: the number of active MultiBodies in the environment
//-------------------------------------------------------------------
inline size_t 
Environment::
GetActiveBodyCount() const {
  return m_activeBodies.size();
}

//-------------------------------------------------------------------
///  GetMultiBody
///  Output: A pointer to a MultiBody in the environment
//-------------------------------------------------------------------
inline shared_ptr<MultiBody>
Environment::
GetMultiBody(size_t _index) const {
  if(_index < m_usableMultiBodies.size())
    return m_usableMultiBodies[_index];
  else
    return shared_ptr<MultiBody>();
}

//-------------------------------------------------------------------
///  GetNavigableSurfaces
///  Output: A pointer to a MultiBody in the environment
//-------------------------------------------------------------------
inline size_t
Environment::GetNavigableSurfacesCount() const {
  return m_navigableSurfaces.size();
}

//-------------------------------------------------------------------
///  GetMultiBody
///  Output: A pointer to a MultiBody in the environment
//-------------------------------------------------------------------
inline shared_ptr<MultiBody>
Environment::GetNavigableSurface(size_t _index) const {
  if(_index < m_navigableSurfaces.size())
    return m_navigableSurfaces[_index];
  else
    return shared_ptr<MultiBody>();
}

#endif
