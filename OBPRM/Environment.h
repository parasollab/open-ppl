#ifndef Environment_h
#define Environment_h

////////////////////////////////////////////////////////////////
#include "Defines.h"

#include "Boundary.h"


#include "MultiBody.h"
#include "Input.h"
#include "OBPRMDef.h" 
#include "string.h"
#include <sstream>


class MultiBody;
class Input;
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


/*
 * @todo: DeleteObstaclesOutsideBoundingBox()->UpdateUsableMultibody() (and it should be a private function in the constructor
 */

class Environment {
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
     * initialize other data member to 0, NULL, and false. sets
     * default boundaries with passed number of dofs and pos_dofs.
     */
    Environment(int dofs, int pos_dofs);

    /** 
     * Copy Constructor.
     * copies multibodies from usable_multibodies of from_env and
     * updates usable_multibodies accordingly.
     */
    Environment(Environment &from_env);

    /** 
     * Copy Constructor.
     * uses i_boundaries instead of boundaries in from_env
     */
    Environment(Environment &from_env, BoundingBox &i_boundaries);


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
    virtual int GetMultiBodyCount();

    ///Return the number of External Body's in the environment;
    virtual int GetExternalBodyCount();

    /**Return a pointer to MultiBody according to this given index.
      *If this index is out of the boundary of list, NULL will be returned.
      *@param _index the index for target, a MultiBody pointer
      */
    virtual MultiBody * GetMultiBody(int _index);


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
    inline double GetPositionRes() {return positionRes;}
    inline void SetPositionRes(const double pRes) {positionRes=pRes;}

    /**Return the resolution for rotation.
      *This tells client how fine this workspace is descretized about rotation.
      *@see Get, this function reads this value from Input instance.
      */
    inline double GetOrientationRes() {return orientationRes;};
    inline void SetOrientationRes(const double rRes) {orientationRes=rRes;}

    //////////////////////////////////////////////////////////////////////////////////////////
    //
    //  Robot Index.
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    /**Return the Index for Robot in environment. This index is the index for MultiBody list in
      *this Environment instance.
      */
    virtual int GetRobotIndex(){return robotIndex;};

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

    /**Return a Bounding Box that encloses all MultiBodys added to this instance.
      *@see FindBoundingBox, Input::bbox_scale, and Input::bbox
      */
    virtual BoundingBox * GetBoundingBox();
    
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
  /*@name Helper functions*/
  //@{
    /**Get information from Input instance.
      *According to information from _input, MultiBody(s) is (are) created and added to
      *this Environment instance. By the way, Bounding Box is calculated.
      *Resolution information is also fetched from input instance.
      *@param _input provides information to initialize Environment.
      *@see MultiBody::Get, FindBoundingBox
      */
    virtual void Get(Input * _input);

  //@}

    virtual void UpdateUsableMultibody();

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

    vector<MultiBody *> multibody;
    int externalbodyCount;     // Bodies enclosing internal obstacles

    vector<MultiBody *> usable_multibody;
    int usable_externalbody_count;

    int robotIndex;
    BoundingBox boundaries;

    double positionRes;
    double orientationRes;
    double minmax_BodyAxisRange;    

    bool copied_instance; //true if instance copied. Used for memory
			  //management
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
GetExternalBodyCount() {
  return usable_externalbody_count;
}

    
//-------------------------------------------------------------------
///  GetMultiBodyCount
///  Output: the number of MultiBody's in the environment
//-------------------------------------------------------------------
inline int 
Environment::
GetMultiBodyCount() {
  return usable_multibody.size();
}

//-------------------------------------------------------------------
///  GetMultiBody
///  Output: A pointer to a MultiBody in the environment
//-------------------------------------------------------------------
inline MultiBody * 
Environment::
GetMultiBody(int _index) {
  if (_index < usable_multibody.size())
    return usable_multibody[_index];
  else
    return 0;
}


#endif
