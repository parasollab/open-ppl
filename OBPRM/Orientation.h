// $Id$
/**@file Orientation.h
  *
  *@author Aaron Michalk
  *@date 2/25/98
  */
  
#ifndef Orientation_h
#define Orientation_h

/////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "Vectors.h"

/////////////////////////////////////////////////////////////////////////////////////////

const double IdentityMatrix[3][3] = 
{{1, 0, 0},
 {0, 1, 0},
 {0, 0, 1}};

/////////////////////////////////////////////////////////////////////////////////////////

/**This class stores inforation about object orientation in 3D space.
  *Many kinds of representation are implemented here, such as
  *	- Matrix
  *	- Euler Angle
  *	- Quaternion
  *	- Rodriques
  *Operations for there representations are provided.
  *Coversion between these representation types are also available.
  */
class Orientation {
public:
    //-----------------------------------------------------------
    //  Enumerations
    //-----------------------------------------------------------
    
    /**The type of Orientation instance. 
      *Including Matix, Euler, Quaternion, and Rodriques types.
      */
    enum OrientationType {
	Matrix   = 0,
	EulerXYZ = 1,  FixedZYX = 1,
	EulerXZY = 2,  FixedYZX = 2,
	EulerYXZ = 3,  FixedZXY = 3,
	EulerYZX = 4,  FixedXZY = 4,
	EulerZXY = 5,  FixedYXZ = 5,
	EulerZYX = 6,  FixedXYZ = 6,
	EulerXYX = 7,  FixedXYX = 7,
	EulerXZX = 8,  FixedXZX = 8,
	EulerYXY = 9,  FixedYXY = 9,
	EulerYZY = 10, FixedYZY = 10,
	EulerZXZ = 11, FixedZXZ = 11,
	EulerZYZ = 12, FixedZYZ = 12,
	Quaternion = 20,
	Rodriques  = 21
    };
    //-----------------------------------------------------------
    //  Static Data
    //-----------------------------------------------------------
    ///Identity matrix
    static const Orientation Identity;
    
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------
    /**@name  Constructors and Destructor.*/
    //-----------------------------------------------------------
    //@{
    /**Default constructor.
      *If no argument is assumed, it is considered as having Euler angles.
      *The type is set to EulerXYZ.
      *alpha ,beta ,gamma is set to 0.
      */
    Orientation();
    
    /**Construct itself according to specified orientation type.
      *@param _type What type this Orientation instance should be.
      *@todo Quaternion and  Rodriques is not implemented.
      *@notice if _type is Matrix, then elements in matrix with coordinate
      *i=j are set to 1 and other elements are set to 0.
      *@see OrientationType for more type information.
      */
    Orientation(OrientationType _type);
    
    /**Construct a matrix type Orientation instance.
      *@param _matrix[3][3] A transformation matrix which will be copied in to this->matrix.
      *@notice this->type is set to Matrix.
      *@see OrientationType for more type information.
      */
    Orientation(const double _matrix[3][3]);
    
    /**Construct a matrix with given Euler angles.
      *@param _type What type this Orientation instance should be.
      *@param _alphs .
      *@param _beta .
      *@param _gamma .
      *@see OrientationType for more type information.
      */ 
    Orientation(OrientationType _type, double _alpha, double _beta, double _gamma);
    
    
    /**A copy constructor.
      *This construtor calls "operator=" function.
      *@param _o Copy _o to this instance.
      *@see operator=
      */
    Orientation(const Orientation & _o);
    
    /**Construct a Quaterion type Orientation instance.
      *@param _rotationAngle Rotate amount (CCW) according to a give axis.
      *@param _rotationAxis A vector which indicates the rotation axis.
      *@notice this->type is set to  Quaternion.
      */  
    Orientation(double _rotationAngle, const Vector3D &_rotationAxis); // quaternion
    
    ///Destructor
    ~Orientation();
    //@}
    
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Operator Overloadings
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    /**@name Operator overloading.*/
    //@{
        
    /** Matrix times vector.
      * This method convert any type of Orientation instance to Matrix type.
      * Then multuply this matrix with a given vector. (Rotate this vector (point) )
      * @param _v A 3D vector.
      * @return _v*(this->matrix[3][3]), the result is a 3D vector.
      */
    Vector3D operator*(const Vector3D & _v);
    
    /** Matrix times another Orientation to change orientation value.
      * This method convert any type of Orientation instance to Matrix type.
      * Then multuply this matrix with a another given Orientation instance.
      * @param _orientation An Orientation with any OrientationType.
      * @return (this->matrix[3][3])*(_orientation.matrix), the result is a new Orientation.
      */
    Orientation operator*(const Orientation & _orientation);
    
    /** Euler angles in this instance plus Euler angles in a given Orientation instance.
      * This method convert any type of Orientation instance to EulerXYZ type and then
      * add given alpha, beta, and gamma to alpha, beta, and gamma in this instance.
      * @param _orientation An Orientation with any OrientationType
      * @return An new orientation whose type is Matrix.
      */
    Orientation operator+(const Orientation & _orientation);
    
    /** Euler angles in this instance minus Euler angles in a given Orientation instance.
      * This method convert any type of Orientation instance to EulerXYZ type and then
      * minus given alpha, beta, and gamma to alpha, beta, and gamma in this instance.
      * @param _orientation An Orientation with any OrientationType
      * @return An new orientation whose type is Matrix.
      */
    Orientation operator-(const Orientation & _orientation);
    
    /** Copy values of a given Orientation instance to this instance according to OrientaionType.
      * According to type in given Orientation instance, values of data memebers are copied to 
      * data member in this instance.
      * @param _orientation An Orientation with any OrientationType
      * @return This instance is returned.
      * @todo Rodriques and Quaternion are not implemented.
      */
    Orientation & operator=(const Orientation & _o);
    
    /** Access matix element by given coordinate.
      * @param _row Whis row the element is in.
      * @param _col Whis coloum the element is in.
      * @return the value in matrix[_row-1][_col-1]
      */
    double & operator()(int _row, int _col);
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Help Methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------
    /**@name Help Methods.*/
    //-----------------------------------------------------------
    //@{
    /** Swap Aij to Aji in orientation matix.
      * This method coneverts any type of Orientation instance to Matrix type.
      * Then it swaps values in element Aij and Aji in the matix of this instance.
      * @note Transpose of Orientation matrix is inverse of Orientation matrix
      */
    void Invert();
    
    /** Create a new Orientation instance which is transpose of matix in this instance.
      * This method coneverts any type of Orientation instance to Matrix type.
      * Then it creates a new Orienation whose value in element Aij equals 
      * ualue in element Aji in the matix of this instance.
      * @return A new Orienetaion instance with Matrix type. Its values are tranpose
      * of matrix in this instance.
      * @note Transpose of Orientation matrix is inverse of Orientation matrix
      */  
    Orientation Inverse();
    
    /** Convert the Orientation instance from current type to specified type.
      * @param _newType The type this instance will be converted to.
      * @see OrientationType for more type information.
      * @todo There are a lot of to do marks in implemetation (.cpp) of this function.
      */
    void ConvertType(OrientationType _newType);
    
    /** Read data from input stream.
      * Read Euler angles and convert to Matrix type of Orientation instance.
      * @param _is The source of input data, alpha, beta,and gamma.
      */
    void Read(istream & _is);
    
    /** Write alpha, beta,and gamma to output stream.
      * @param _os Onput data, alpha, beta,and gamma to _os.
      */
    void Write(ostream & _os);
    //@}

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //	Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

    //-----------------------------------------------------------
    //  Data
    //-----------------------------------------------------------
    
    ///Type of the Orientation instance.
    OrientationType type;

    /**@name  Matrix type Data members*/
    //@{
    double matrix[3][3];
    //@}

    /**@name Euler angle type Data members*/
    //@{
    double alpha;
    double beta;
    double gamma;
    //@}

    /** @name Quaternion type Data members.
      * We assume that the vector [rotationAngle, rotationAxis] is normalized
      */
    //@{
    double rotationAngle;
    Vector3D rotationAxis;

    double epsilon1;
    double epsilon2;
    double epsilon3;
    double epsilon4;
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
  //	Private data member and member methods
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////

private:
};

//===============================================================
// Inline functions
//===============================================================
inline double & Orientation::operator()(int _row, int _col) {
    return matrix[_row-1][_col-1];
}

#endif

