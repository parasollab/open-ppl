// $Id$
/**@file Environment.h
 * Created   2/25/98 Aaron Michalk
 * Modified  4/13/98 Aaron Michalk
 * Modified  4/16/98 Wookho Son
 * Added     5/18/98 Wookho Son
 * Modified  7/14/98 Wookho Son
 * Added/Modified  7/31/98 Wookho Son
 *
 * Header files should be included whenever if need to use 
 * the membership functions here. Otherwise, just the inclusion of 
 * the class definition is ok.
 *
 * @date 2/25/1998
 * @author Aaron Michalk
 */

////////////////////////////////////////////////////////////////////////////////////////////
#ifndef Environment_h
#define Environment_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include standard headers
#include <fstream.h>
#include <list.h>

//////////////////////////////////////////////////////////////////////////////////
class MultiBody;
class Input;
//////////////////////////////////////////////////////////////////////////////////

/**@name Format version for path files
 *
 *      The number breaks down as YearMonthDay so numerical
 *      comparisons can be more easily made.
 * Warning: Be consistent.  It should be YYYYMMDD
 *      Inconsistent conversions can be misleading.  
 *      For example, comparing 200083  to 20000604.
 */
//@{
#define PATHVER_LEGACY                     20001025
#define PATHVER_20001022                   20001022
#define PATHVER_20001125                   20001125
//@}


class Environment {
public:
    //-----------------------------------------------------------
    /// @name  Enumerations
    //-----------------------------------------------------------
    enum DirectionType {
         GMS_NORMAL = 0,
         GMS_TANGENTIAL = 1,
         GMS_ORTHOGONAL = 2
    };

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

  //===================================================================
  /**@name  Constructors and Destructor*/
  //===================================================================
  //@{

    /**Constrcutor. Set pathversion as newest path file version ID (hard coded).
      *and initialize other data member to 0, NULL, and false.
      */
    Environment();

    /**Constrcutor. Set pathversion as newest path file version ID (hard coded).
      *set Rotbot id as index and initialize other data member to 0, NULL, and false.
      *@param index the index for robot.
      */
    Environment(int index);

    /**Destrcutor.
      *Free memory for every added MultiBody instance.
      */
    ~Environment();
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
    int GetPathVersion();

    //////////////////////////////////////////////////////////////////////////////////////////
    //
    //  MultiBody
    //
    //////////////////////////////////////////////////////////////////////////////////////////

    ///Return the number of MultiBody's in the environment
    int GetMultiBodyCount();

    ///Put this _multibody into list and increase the number of MultiBody.
    void AddMultiBody(MultiBody *_multibody);

    /**Return a pointer to MultiBody according to this given index.
      *If this index is out of the boundary of list, NULL will be returned.
      *@param _index the index for target, a MultiBody pointer
      */
    MultiBody * GetMultiBody(int _index);

    ///Do nothing Not implemeneted?
    void Couple(MultiBody *_multibody[], int _number);

    ///Do nothing. Not implemeneted?
    void Decouple(MultiBody *_multibody[], int _number);

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
    int GetRobotIndex(){return robotIndex;};

    /**Set the Index for Robot in environment. This index is the index for MultiBody list in
      *this Environment instance.
      */  
    void SetRobotIndex(int index){robotIndex = index;};

    //@}

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
    void FindBoundingBox();

    /**Return a Bounding Box that encloses all MultiBodys added to this instance.
      *The format of this bounding box is a 6 elements array.
      *They are minx, maxx, miny, maxy, minz, and maxz.
      *@see FindBoundingBox, Input::bbox_scale, and Input::bbox
      */
    double * GetBoundingBox();

    /**Update a Calculate Bounding Box according to user input.
      *This methods did two things. First, if user provide a Bounding Box, this Bounding Box
      *will overwrite the one calculated in FindBoundingBox. Second, if scale factor for bound
      *box is not 1, then scale this bounding box.
      */
    void UpdateBoundingBox(Input * _input);

    /**Set Bounding by given parameters.
      *@param x Minimam x for bounding box
      *@param X Maximam x for bounding box
      *@param y Minimam y for bounding box
      *@param Y Maximam y for bounding box
      *@param z Minimam z for bounding box
      *@param Z Maximam z for bounding box
      */
    void PutBoundingBox(double x,double X,double y,double Y,double z,double Z);


    /**Return manximu axis range of bounding box.
      *This value is calculated in FindBoundingBox.
      */
    double Getminmax_BodyAxisRange();
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
    void Write(ostream & _os);

    /**Output the bounding box to ouputstream.
      *Display it in text format.....
      */
    void DisplayBoundingBox(ostream & _os);

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
      *@see MultiBody::Get, FindBoundingBox, and UpdateBoundingBox
      */
    void Get(Input * _input);
  //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Protected Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

protected:

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Private Data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

private:
    //---------------------------------------------------------------
    /// @name  Data
    //---------------------------------------------------------------
    int pathVersion;           /// Format version for path files
    int multibodyCount;
    MultiBody **multibody;
    int robotIndex;
    double boundingBox[6];
    double positionRes;
    double orientationRes;
    double minmax_BodyAxisRange;

};

///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//  implemetation of Environment
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

//===================================================================
///  Inline functions
//===================================================================

/// Format version for path files
inline int Environment::GetPathVersion() {
    return pathVersion;
}

//-------------------------------------------------------------------
///  GetMultiBodyCount
///  Output: the number of MultiBody's in the environment
//-------------------------------------------------------------------
inline int Environment::GetMultiBodyCount() {
    return multibodyCount;
}

//-------------------------------------------------------------------
///  GetMultiBody
///  Output: A pointer to a MultiBody in the environment
//-------------------------------------------------------------------
inline MultiBody * Environment::GetMultiBody(int _index) {
    if (_index < multibodyCount)
        return multibody[_index];
    else
        return 0;
}

#endif
