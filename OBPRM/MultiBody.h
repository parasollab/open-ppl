// $Id$

/**
  * @file MultiBody.h
  * @date 2/25/98
  * @author Aaron Michalk
  */
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef MultiBody_h
#define MultiBody_h

////////////////////////////////////////////////////////////////////////////////////////////
//Include standard headers
#include <fstream.h>
#include <list.h>

////////////////////////////////////////////////////////////////////////////////////////////
//include OBPRM headers
#include "Transformation.h"
#include "FixedBody.h"
#include "FreeBody.h"

#ifndef VID
	#include "Graph.h"
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////
class Environment;
class Input;

//////////////////////////////////////////////////////////////////////////////////////////////////////
typedef pair<VID,int> RANGE_TYPE;

//////////////////////////////////////////////////////////////////////////////////////////////////////
/**A MultiBody represent a Obstacle or a Robot in workspace.
  *MultiBody contain one or more Bodys, either Fixed or Free Bodys.
  *Many access methods are implemented to allow client access internal information
  *about MultiBody instance, like number of Bodys, Fixed and Free, Bounding box,
  *center of mass, surface area size, and bounding sphere radius.
  *
  *@see Body, FreeBody, and FixedBody
  */
class MultiBody {
public:
    //-----------------------------------------------------------
    //  Static Methods
    //-----------------------------------------------------------

	/**@todo Check this!! ComputePUMAInverseKinematics ??
	 */
    static void ComputePUMAInverseKinematics(Transformation & _t, double _a2, double _d3, double _a3, double _d4, double theta[8][6]);


  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Constructors and Destructor*/
	//@{

	///Constructor. Set _owner as the owner of this MultiBody instance.
    MultiBody(Environment * _owner);

	///Destrucot. Free memory allocated to all Bodys added to this multiBody.
    ~MultiBody();

	//@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Access Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    /**@name Access Methods. Use these method to acess or change internal state*/
	//@{

	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//	Get/Set Free Body info
	//
	//////////////////////////////////////////////////////////////////////////////////////////

	///Return a free body accroding to the given index. the index should be in [0,GetFreeBodyCount())
    FreeBody * GetFreeBody(int _index);
	///Number of free body in this mutilbody.
    int GetFreeBodyCount();
	///Search index for given FreeBody, _b, if _b is not in this multibody, -1 is returned.
    int GetFreeBodyIndex(FreeBody * _b);
	///Add a Free Body
    void AddBody(FreeBody * _body);

	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//	Get/Set Fixed Body info
	//
	//////////////////////////////////////////////////////////////////////////////////////////

	///Return a fixed body accroding to the given index. the index should be in [0,GetFixedBodyCount())
    FixedBody * GetFixedBody(int _index);
	///Number of fixed body in this mutilbody.
    int GetFixedBodyCount();
	///Search index for given FixedBody, _b, if _b is not in this multibody, -1 is returned.
    int GetFixedBodyIndex(FixedBody * _b);
	///Add a Fixed Body
    void AddBody(FixedBody * _body);

	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//	Get/Set Body info
	//
	//////////////////////////////////////////////////////////////////////////////////////////

	/**Return a free body accroding to the given index.
	  *_index should in [0, GetFreeBodyCount()+GetFixedBodyCount())
	  *if _index is smaller than GetFixedBodyCount(), then fixed body will be returned.
	  *if _index is larger than GetFixedBodyCount(), then free body with (_index-GetFixedBodyCount())
	  *index will be returned.
	  */
    Body * GetBody(int _index); // new
	///return GetFreeBodyCount()+GetFixedBodyCount()
    int GetBodyCount(); // new
	/**Get the very first body in a MultiBody.
	 *It is a fixed base body, if the MultiBody is a manipualtor
	 *Otherwise, it is the body itself.
	 */
    Body * GetFirstBody();

	///Get number of of links in "this" MultiBody by checking forward connection.
    int GetNumberOfLinks();

	/**Determine if the MultiBody at hand is a manipulator.
	  *If there is no free body attached to it,
	  *it is considered to be a manipulator
	  */
    int IsManipulator();

	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//	Get/Set Range info (bounding box, bounding sphere...etc.
	//
	//////////////////////////////////////////////////////////////////////////////////////////


	/**to get center of mass, you don't need to additionally call the above: ComputeCenterOfMass
	  *because GetCenterOfMass will check if it is necessary to call this method.
	  *@see ComputeCenterOfMass
	  */
    Vector3D GetCenterOfMass();

	/**Return Max Axis Range, which is computed during finding bounding box.
	  *Max Axis Range is max distance in X, Y, or Z direction in bounding box.
	  *@see FindBoundingBox
	  */
    double GetMaxAxisRange();

	/*Return bounding box of this multibody
	 *@see FindBoundingBox
	 */
    double * GetBoundingBox();

    /**Compute and return the maximum size of this multibody.
	  *The maximum size is computed by (Radius of first link+ 2*Radius of second link+ ... )
	  *@see GMSPolyhedron::maxRadius
	  */
    double GetBoundingSphereRadius();

	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//	Get/Set Area info.
	//
	//////////////////////////////////////////////////////////////////////////////////////////

	///@todo What is numBodies for?
	int GetNumBodies();

	/**Get total area of fixed bodys in this instance. (computed in Get method)
	  *@see Get, GetFixAreas
	  */
    double GetFixArea();

	/**Get a list of areas of fixed bodys in this instance. 
	  *(computed in Get method)
	  *@see Get, GetFixArea
	  */
    vector<double> GetFixAreas();

	/**Get total area of free bodys in this instance. (computed in Get method)
	  *@see Get, GetFreeAreas
	  */
    double GetFreeArea();

	/**Get a list of areas of free bodys in this instance. 
	  *(computed in Get method)
	  *@see Get, GetFreeArea
	  */
    vector<double> GetFreeAreas();

	/**Get total area of free bodys and fixed bodys in this instance. 
	  *(computed in Get method)
	  *@see Get, GetFixArea, and GetFreeArea
	  */
    double GetArea();


	///////////////////////////////////////////////////////////////////////////////////////////
	//
	//	Get/Set Area info.
	//
	//////////////////////////////////////////////////////////////////////////////////////////

	/**Get user input information from Input instance.
	  *This method get number of (free and fixed) body info and number of connection from input.
	  *Then it calls Get(s) in FreeBody or FixedBody to get information from Input instance.
	  *Number of bodys (fixed and free), areas, bounding box, and center of mass are all computed
	  *in here.
	  *@param _multibodyIndex index for the owner of this fixed body in input
	  *@see FreeBody::Get, FixedBody::Get, Connection, FreeBody::Link, Connection::Get,
	  *FixedBody::Link ,FindBoundingBox, and ComputeCenterOfMass
	  */
    virtual void Get(Input * _input, int _index);

	//@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	I/O
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name I/O Methods. Use these method to read in/write out internal state*/
    //@{

	/**Write information about this MultiBody instance to outputstream.
	  *First the tag "MultiBody was output, and then calls Fixed and (or) Free Bodys'
	  *write and Connnection's write.
	  *@see FixedBody::Write, FreeBody::Write, and Connection::Write
	  */
    void Write(ostream & _os);

	//@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Helpers
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
    /**@name Help Methods*/
	//@{

	/**Alway return 0.0
	  *@todo why 0.0?
	  */
    double ComputeDistance(Body * _body1, Body * _body2);


	/**Configure the joint by the given amount of displacement.
	  *@param _dof Number of Freebody that is going to be reconfigured (moved)
	  *@param _s An array of displacement value. The length of _s is _dof.
	  */
    void ConfigureJoint(double * _s, int _dof);

    /**if GetCenterOfMass() is called for the first time, then
      *ComputeCenterOfMass() is called automatically, and the
      *computed value is stored in this class for the next time.
	  *@see Body::ComputeCenterOfMass
	  */
    void ComputeCenterOfMass();

	/**Calculate bounding box by FreeBodys and FixedBodys in this instance.
	  *maxAxisRange is its byproduct...
	  */
    void FindBoundingBox();

	//@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Protected data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

protected:
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	get information from input
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  //@{

	///Get Body Info from input
	void GetBodyInfoFromInput(Input * _input, int _index);
	///Get Link Info from input
	void GetLinkInfoFromInput(Input * _input, int _index);

  //@}

	
  // Area Stuff
  int numBodies;              ///< Total number of Bodies
  double fixArea;             ///< Area of FixedBodies
  double freeArea;            ///< Area of FreeBodies
  double area;                ///< Total Area of Bodies
  vector<double> fixAreas;    ///< Vector of Areas of FixedBodies
  vector<double> freeAreas;   ///< Vector of Areas of FreeBodies

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Private data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

private:
    //-----------------------------------------------------------
    ///  Data
    //-----------------------------------------------------------
    Environment * environment;	///Owner

    int FixedBodyCount;
    FixedBody ** fixedBody;
    int FreeBodyCount;
    FreeBody ** freeBody;
    //Equation motionEquation;

    Vector3D CenterOfMass;
    bool CenterOfMassAvailable;

    double boundingBox[6];
    double maxAxisRange;
};


///////////////////////////////////////////////////////////////////////////////////////////
//
//
//
//
//	Implementation of MultiBody
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////

//===================================================================
///  Inline Functions
//===================================================================

//-------------------------------------------------------------------
//  GetCenterOfMass
//-------------------------------------------------------------------
inline Vector3D MultiBody::GetCenterOfMass(){
    if (!CenterOfMassAvailable) {
        ComputeCenterOfMass();
    }
    return CenterOfMass;
}

//-------------------------------------------------------------------
//  GetFixedBodyCount
//-------------------------------------------------------------------
inline int MultiBody::GetFixedBodyCount() {
    return FixedBodyCount;
}

//-------------------------------------------------------------------
//  GetFreeBodyCount
//-------------------------------------------------------------------
inline int MultiBody::GetFreeBodyCount() {
    return FreeBodyCount;
}

//-------------------------------------------------------------------
//  GetBodyCount
//-------------------------------------------------------------------
inline int MultiBody::GetBodyCount() {
    return FreeBodyCount+FixedBodyCount;
}
//-------------------------------------------------------------------
//  GetBody
//-------------------------------------------------------------------
inline Body * MultiBody::GetBody(int _index) {
    if(_index < 0 || _index >= FreeBodyCount+FixedBodyCount) {
        cout << "Error in MultiBody::GetBody !!" << endl;
        exit(-1);
    } else
    if (_index < FixedBodyCount) {
        return fixedBody[_index];
    } else {
        return freeBody[_index-FixedBodyCount];
    }
}

//-------------------------------------------------------------------
//  GetFixedBody
//-------------------------------------------------------------------
inline FixedBody * MultiBody::GetFixedBody(int _index) {
    if (_index < FixedBodyCount)
        return fixedBody[_index];
    else
        return 0;
}

//-------------------------------------------------------------------
//  GetFreeBody
//-------------------------------------------------------------------
inline FreeBody * MultiBody::GetFreeBody(int _index) {
    if (_index < FreeBodyCount)
        return freeBody[_index];
    else
        return 0;
}

//-------------------------------------------------------------------
//  GetFreeBodyIndex
//-------------------------------------------------------------------
inline int MultiBody::GetFreeBodyIndex(FreeBody * _b) {
    for (int i=0; i < FreeBodyCount; i++)
        if (_b == GetFreeBody(i))
	    return i;
    // error
    return -1;
}

//-------------------------------------------------------------------
//  GetFixedBodyIndex
//-------------------------------------------------------------------
inline int MultiBody::GetFixedBodyIndex(FixedBody * _b) {
    for (int i=0; i < FixedBodyCount; i++)
        if (_b == GetFixedBody(i))
	    return i;
    // error
    return -1;
}

//-------------------------------------------------------------------
//  IsManipulator
///  Function: Determine if the MultiBody at hand is a manipulator.
///            If there is no free body attached to it,
///            it is considered to be a manipulator
///
///  Output:   True/False
//-------------------------------------------------------------------
inline int MultiBody::IsManipulator() {
    return (FreeBodyCount > 0) ? 1 : 0;
}


#endif
