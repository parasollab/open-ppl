#ifndef Environment_h
#define Environment_h

////////////////////////////////////////////////////////////////
#include "Boundary.h"
#include "BoundingBox.h"
#include "BoundingSphere.h"

#include "Robot.h"
#include "MultiBody.h"
#include <string>

#include "MPUtils.h"
#include "Graph.h"
#include "GraphAlgo.h"

#include <sstream>
#include <iostream>
#include <fstream>
#include "boost/shared_ptr.hpp"


using boost::shared_ptr;

class MultiBody;
class MPProblem;
////////////////////////////////////////////////////////////////

/**@name Format version for path files
 *
 *      The number breaks down as YearMonthDay (YYYYMMDD) so numerical
 *      comparisons can be made.
 * Warning: Be consistent.  It should be YYYYMMDD
 */
//@{
#define PATHVER_LEGACY                     20001025
#define PATHVER_20001022                   20001022
#define PATHVER_20001125                   20001125
//@}


class Environment : public MPBaseObject{
  public:

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //    Constructors and Destructor
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    //===================================================================
    /**@name  Constructors and Destructor*/
    //===================================================================
    //@{

    /** Constructor. 
     * Set pathversion as newest path file version ID (hard coded),
     * initialize other data members. m_boundaries must be set later
     */
    Environment();

    //This version of the constructor is the same as the default, except
    //m_boundaries will also be set to _bb
    Environment(shared_ptr<Boundary> _bb);

    /** 
     * Copy Constructor.
     * copies multibodies from usable_multibodies of from_env and
     * updates usable_multibodies accordingly.
     */
    Environment(const Environment &from_env);

    /** 
     * Copy Constructor. COPIES FROM MPPRoblem's Environment
     * copies multibodies from usable_multibodies of from_env and
     * updates usable_multibodies accordingly.
     */
    Environment(MPProblem* in_pProblem);

    /** 
     * Copy Constructor.
     * uses i_boundaries instead of boundaries in from_env
     */
    Environment(const Environment &from_env, const Boundary &i_boundaries);

    ///\brief Constructor taking in an XML object
    Environment(XMLNodeReader& in_Node, MPProblem* in_pProblem);

    Environment(const Environment &from_env, string filename);

    /**
     * Destructor.
     * Free memory for every added MultiBody instance if this instance
     * was not copied from another one.	
     */
    virtual ~Environment();
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

    string& GetEnvFileName() { return m_filename; }
    void SetEnvFileName(string _newFileName){m_filename = _newFileName;}

    //////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Path Version
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    ///Return format version for path files
    virtual int GetPathVersion();

    //////////////////////////////////////////////////////////////////////////////////////////
    //
    //  MultiBody
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    ///Return the number of MultiBody's in the environment
    virtual int GetMultiBodyCount() const;

    ///Return the number of External Body's in the environment;
    virtual int GetExternalBodyCount() const;

    /**Return a pointer to MultiBody according to this given index.
     *If this index is out of the boundary of list, NULL will be returned.
     *@param _index the index for target, a MultiBody pointer
     */
    virtual shared_ptr<MultiBody> GetMultiBody(size_t _index) const;

    ///Return the number of navigable surfaces in the environment;
    virtual int GetNavigableSurfacesCount() const;
    virtual int GetRandomNavigableSurfaceIndex();
    /**Return a pointer to MultiBody in m_navigableSurfaces given index.
     *If this index is out of the boundary of list, NULL will be returned.
     *@param _index the index for target, a MultiBody pointer
     */
    virtual shared_ptr<MultiBody> GetNavigableSurface(size_t _index) const;

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

    //////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Robot Index.
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    /**Return the Index for Robot in environment. This index is the index for MultiBody list in
     *this Environment instance.
     */
    virtual int GetRobotIndex() const { return robotIndex; }

    //@}

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

    /**Calculate a Bounding Box that encloses all MultiBodys added to this instance.
     *Calculate a Bounding Box that encloses all MultiBodys added to this instance except
     *Robot. Moreover, minmax_BodyAxisRange and resolution for postion are also calculated.
     *@see MultiBody::GetBoundingBox and MultiBody::GetMaxAxisRange
     */
    virtual void FindBoundingBox();
    virtual void ResetBoundingBox(double _d);
    virtual void ConstrainBoundingBox(double _d, int _start, int _goal);

    /**Return a Bounding Box that encloses all MultiBodys added to this instance.
     *@see FindBoundingBox, Input::bbox_scale, and Input::bbox
     */
    virtual shared_ptr<Boundary> GetBoundary() const;
    virtual void SetBoundary(shared_ptr<Boundary> _b);

    /**Return manximu axis range of bounding box.
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
    void buildCDstructure(cd_predefined cdtype);
    void BuildRobotStructure(); 
    vector<Robot>& GetRobots(){return robotVec;} 
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

    bool operator==(const Environment& e) const;
    bool operator!=(const Environment& e) const { return !(*this == e); }

    ///////////////////////////////////////////////////////////////////////////////////////////
    //
    //
    //    Protected Data member and member methods
    //
    //
    //////////////////////////////////////////////////////////////////////////////////////////

  protected:
    //---------------------------------------------------------------
    /// @name  Data
    //---------------------------------------------------------------
    int pathVersion;          /// Format version for path files

    vector<shared_ptr<MultiBody> > multibody;
    vector<shared_ptr<MultiBody> > usable_multibody;
    int usable_externalbody_count;

    vector<shared_ptr<MultiBody> > m_navigableSurfaces;

    int robotIndex; //index of the robot in the usable_multibody vector
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

/// Format version for path files
inline int 
Environment::
GetPathVersion() {
  return pathVersion;
}

//-------------------------------------------------------------------
///  GetMultiBodyCount
///  Output: the number of MultiBody's in the environment
//-------------------------------------------------------------------

inline int 
Environment::
GetExternalBodyCount() const {
  return usable_externalbody_count;
}


//-------------------------------------------------------------------
///  GetMultiBodyCount
///  Output: the number of MultiBody's in the environment
//-------------------------------------------------------------------
inline int 
Environment::
GetMultiBodyCount() const {
  return usable_multibody.size();
}

//-------------------------------------------------------------------
///  GetMultiBody
///  Output: A pointer to a MultiBody in the environment
//-------------------------------------------------------------------
inline shared_ptr<MultiBody>
Environment::
  GetMultiBody(size_t _index) const {
    if(_index < usable_multibody.size())
      return usable_multibody[_index];
    else
      return shared_ptr<MultiBody>();
  }

//-------------------------------------------------------------------
///  GetNavigableSurfaces
///  Output: A pointer to a MultiBody in the environment
//-------------------------------------------------------------------
inline int 
Environment::GetNavigableSurfacesCount() const {
  return m_navigableSurfaces.size();
}

//-------------------------------------------------------------------
///  GetMultiBody
///  Output: A pointer to a MultiBody in the environment
//-------------------------------------------------------------------
inline shared_ptr<MultiBody>
  Environment::GetNavigableSurface(size_t _index) const {
    if(_index < m_navigableSurfaces.size() && _index >= 0)
      return m_navigableSurfaces[_index];
    else
      return shared_ptr<MultiBody>();
  }

#endif
