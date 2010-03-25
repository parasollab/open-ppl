// $Id$
/////////////////////////////////////////////////////////////////////
//  MultiBody.c
//
//  Created   3/ 1/98 Aaron Michalk
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
// header files for C-space Toolkit
/////////////////////////////////////////////////////////////////////////////////
//#include <sys/time.h>
#ifdef HPUX
#include <sys/io.h>
#endif
#ifdef USE_CSTK
#include <three_d.h>
#include <wrappers.h>
#include <distance.h>
#include <witnesses.h>
#endif

//////////////////////////////////////////////////////////////////////////////////
#include "MultiBody.h"
#include "Transformation.h"
#include "util.h"
#include <numeric>

#define MAXCONTACT  10

//
// Global variable to be used for contact checking by C-Space Toolkit
//

//===================================================================
//  ComputePUMAInverseKinematics
//===================================================================
void MultiBody::ComputePUMAInverseKinematics(Transformation & _t, double _a2, double _d3, double _a3, double _d4, double theta[8][6]) 
{
    //---------------------------------------------------------------
    //  Compute theta1
    //---------------------------------------------------------------
    double root = sqrt(_t.position.getX()*_t.position.getX() + _t.position.getY()*_t.position.getY() - _d3*_d3);
    theta[0][0] = atan2(_t.position.getY(), _t.position.getX()) - atan2(_d3, root);
    theta[4][0] = atan2(_t.position.getY(), _t.position.getX()) - atan2(_d3, root);
    int i;
    for (i=1; i < 4; i++) {
        theta[i][0] = theta[0][0];
        theta[i+4][0] = theta[0][0];
    }
    //---------------------------------------------------------------
    //  Compute theta3
    //---------------------------------------------------------------
    double K = (_t.position.getX()*_t.position.getX() + _t.position.getY()*_t.position.getY() + _t.position.getZ()*_t.position.getZ() - _a2*_a2 - _a3*_a3 - _d3*_d3 - _d4*_d4)/(2*_a2);
    theta[0][2] = atan2(_a3, _d4) - atan2(K,  sqrt(_a3*_a3 + _d4*_d4 - K*K));
    theta[1][2] = theta[0][2];
    theta[2][2] = atan2(_a3, _d4) - atan2(K, -sqrt(_a3*_a3 + _d4*_d4 - K*K));
    theta[3][2] = theta[1][2];
    theta[4][2] = theta[0][2];
    theta[5][2] = theta[0][2];
    theta[6][2] = theta[1][2];
    theta[7][2] = theta[1][2];
    //---------------------------------------------------------------
    //  Compute theta2
    //---------------------------------------------------------------
    double s1, c1, s3, c3;
    for (i=0; i < 8; i += 2) {
        s1 = sin(theta[i][0]);
        c1 = cos(theta[i][0]);
        s3 = sin(theta[i][2]);
        c3 = cos(theta[i][2]);
        theta[i][1] = atan2((-_a3 - _a2*c3)*_t.position.getZ() - (c1*_t.position.getX() + s1*_t.position.getY())*(_d4 - _a2*s3),
                              (_a2*s3 - _d4)*_t.position.getZ() - (_a3 + _a2*c3)*(c1*_t.position.getX() + s1*_t.position.getY()))
                        - theta[i][2];
        theta[i+1][1] = theta[i][1];
    }
    //---------------------------------------------------------------
    //  Compute theta4
    //---------------------------------------------------------------
    double r13 = _t.orientation.matrix[1][3];
    double r23 = _t.orientation.matrix[2][3];
    double r33 = _t.orientation.matrix[3][3];
    double s23, c23;
    for (i=0; i < 8; i += 2) {
        s1 = sin(theta[i][0]);
        c1 = cos(theta[i][0]);
        s23 = sin(theta[i][1] + theta[i][2]);
        c23 = cos(theta[i][1] + theta[i][2]);
        theta[i][3] = atan2(-r13*s1 + r23*c1,
                            -(r13*c1 + r23*s1)*c23 + r33*s23);
        theta[i+1][3] = theta[i][3] + 180.0;
    }
    //---------------------------------------------------------------
    //  Compute theta5
    //---------------------------------------------------------------
    double s4, c4;
    for (i=0; i < 8; i += 2) {
        s1 = sin(theta[i][0]);
        c1 = cos(theta[i][0]);
        s23 = sin(theta[i][1] + theta[i][2]);
        c23 = cos(theta[i][1] + theta[i][2]);
        s4 = sin(theta[i][3]);
        c4 = cos(theta[i][3]);
        theta[i][4] = atan2(-r13*(c1*c23*c4 + s1*s4) - r23*(s1*c23*c4 - c1*s4) + r33*(s23*c4),
                            -(r13*c1 + r23*s1)*s23 - r33*c23);
        theta[i+1][4] = -theta[i][4];
    }
    //---------------------------------------------------------------
    //  Compute theta6
    //---------------------------------------------------------------
    double s5, c5;
    double r11 = _t.orientation.matrix[1][1];
    double r21 = _t.orientation.matrix[2][1];
    double r31 = _t.orientation.matrix[3][1];
    for (i=0; i < 8; i += 2) {
        s1 = sin(theta[i][0]);
        c1 = cos(theta[i][0]);
        s23 = sin(theta[i][1] + theta[i][2]);
        c23 = cos(theta[i][1] + theta[i][2]);
        s4 = sin(theta[i][3]);
        c4 = cos(theta[i][3]);
        s5 = sin(theta[i][4]);
        c5 = cos(theta[i][4]);
        theta[i][5] = atan2(-r11*(c1*c23*s4 - s1*c4) - r21*(s1*c23*s4 + c1*c4) + r31*s23*s4,
                            r11*((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5) + r21*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) - r31*(s23*c4*c5 + c23*s5));
        theta[i+1][5] = theta[i][5] + 180.0;
    }
}


//===================================================================
//  Constructors and Destructor
//===================================================================
MultiBody::MultiBody(Environment * _owner) 
  : bInternal(false), environment(_owner), CenterOfMassAvailable(false) 
{}

MultiBody::~MultiBody() 
{}


//-------------------------------------------------------------------
//  GetFreeBody
//-------------------------------------------------------------------
shared_ptr<FreeBody> MultiBody::GetFreeBody(int _index) const 
{
  if(_index < (int)freeBody.size())
    return freeBody[_index];
  else
    return shared_ptr<FreeBody>();
}

//-------------------------------------------------------------------
//  GetFreeBodyCount
//-------------------------------------------------------------------
int MultiBody::GetFreeBodyCount() const
{
  return freeBody.size();
}

//-------------------------------------------------------------------
//  GetFreeBodyIndex
//-------------------------------------------------------------------
int MultiBody::GetFreeBodyIndex(const FreeBody& _b) const {
  for(size_t i=0; i<freeBody.size(); ++i)
    if(_b == *freeBody[i].get())
      return i;
  return -1;
}
int MultiBody::GetFreeBodyIndex(const shared_ptr<FreeBody>& _b) const {
  for(size_t i=0; i<freeBody.size(); ++i)
    if(_b == freeBody[i])
      return i;
  return -1;
}

//===================================================================
//  AddBody
//===================================================================
void MultiBody::AddBody(const FreeBody& _body) 
{
  AddBody(shared_ptr<FreeBody>(new FreeBody(_body)));
}
void MultiBody::AddBody(const shared_ptr<FreeBody>& _body) 
{
  freeBody.push_back(_body);
}


//-------------------------------------------------------------------
//  GetFixedBody
//-------------------------------------------------------------------
shared_ptr<FixedBody> MultiBody::GetFixedBody(int _index) const
{
  if(_index < (int)fixedBody.size())
    return fixedBody[_index];
  else
    return shared_ptr<FixedBody>();
}

//-------------------------------------------------------------------
//  GetFixedBodyCount
//-------------------------------------------------------------------
int MultiBody::GetFixedBodyCount() const
{
  return fixedBody.size();
}

//-------------------------------------------------------------------
//  GetFixedBodyIndex
//-------------------------------------------------------------------
int MultiBody::GetFixedBodyIndex(const FixedBody& _b) const 
{
  for(size_t i=0; i<fixedBody.size(); ++i)
    if(_b == *fixedBody[i].get())
      return i;
  return -1;
}
int MultiBody::GetFixedBodyIndex(const shared_ptr<FixedBody>& _b) const 
{
  for(size_t i=0; i<fixedBody.size(); ++i)
    if(_b == fixedBody[i])
      return i;
  return -1;
}

//===================================================================
//  AddBody
//===================================================================
void MultiBody::AddBody(const FixedBody& _body) 
{
  AddBody(shared_ptr<FixedBody>(new FixedBody(_body)));
}
void MultiBody::AddBody(const shared_ptr<FixedBody>& _body) 
{
  fixedBody.push_back(_body);
}

//-------------------------------------------------------------------
//  GetBody
//-------------------------------------------------------------------
shared_ptr<Body> MultiBody::GetBody(int _index) const
{
  if(_index < 0 || _index >= (int)(freeBody.size() + fixedBody.size())) {
    cout << "Error in MultiBody::GetBody !!" << endl;
    exit(-1);
  } 
  else
    if(_index < (int)fixedBody.size())
      return fixedBody[_index];
    else
      return freeBody[_index - fixedBody.size()];
}

//-------------------------------------------------------------------
//  GetBodyCount
//-------------------------------------------------------------------
int MultiBody::GetBodyCount() const
{
  return freeBody.size() + fixedBody.size();
}

//===================================================================
//  GetFirstBody
//
//  Function: Get the very first body in a MultiBody.
//            It is a fixed base body, if the MultiBody is a manipualtor
//            Otherwise, it is the body itself.
//
//  Output: The pointer to the body
//
//===================================================================
shared_ptr<Body> MultiBody::GetFirstBody() const
{
  //  I assume that the first body in the list is the anchor body.
  //  That is, all other bodies in the MultiBody are linked sequentially
  //  in a "forward" direction (with possible branches) from this anchor.
  if(!fixedBody.empty())
    return fixedBody.front();
  else 
    if(!freeBody.empty()) 
      return freeBody.front();
    else 
      return shared_ptr<Body>();
}


//===================================================================
//  GetNumberOfLinks
//
//  Output: The number of links in "this" MultiBody
//===================================================================
int MultiBody::GetNumberOfLinks() const
{
  shared_ptr<Body> bb = GetFirstBody();
  if(bb == shared_ptr<Body>())
    return 0;

  shared_ptr<Body> b = bb;
  int i = 1;
  while (b->ForwardConnectionCount() > 0) {
    b = b->GetForwardConnection(0).GetNextBody();
    i++;
  }
  return i-1;
}

//-------------------------------------------------------------------
//  IsManipulator
///  Function: Determine if the MultiBody at hand is a manipulator.
///            If there is no free body attached to it,
///            it is considered to be a manipulator
///
///  Output:   True/False
//-------------------------------------------------------------------
bool MultiBody::IsManipulator() const
{
  return !freeBody.empty();
}


//-------------------------------------------------------------------
//  GetCenterOfMass
//-------------------------------------------------------------------
Vector3D MultiBody::GetCenterOfMass()
{
  if(!CenterOfMassAvailable) 
    ComputeCenterOfMass();
  return CenterOfMass;
}

//===================================================================
//  GetMaxAxisRange
//===================================================================
double MultiBody::GetMaxAxisRange() const
{
  return maxAxisRange;
}

//===================================================================
//  GetBoundingBox
//===================================================================
const double * MultiBody::GetBoundingBox() const
{
  return boundingBox;
}

//===================================================================
//  GetBoundingSphereRadius
//    the maximum size of this multibody
//===================================================================
double MultiBody::GetBoundingSphereRadius() const
{
  double result = GetBody(0)->GetPolyhedron().maxRadius;
  for(int i=1; i<GetBodyCount(); ++i)
    result += GetBody(i)->GetPolyhedron().maxRadius * 2.0;
  return result;
}

//===================================================================
//  GetInsideSphere Radius
//    the minimum size of the multibody
//===================================================================
double MultiBody::GetInsideSphereRadius() const 
{
  double result = GetBody(0)->GetPolyhedron().minRadius;
  for(int i=1; i<GetBodyCount(); ++i)
    if(GetBody(i)->GetPolyhedron().minRadius > result)
      result = GetBody(i)->GetPolyhedron().minRadius;
  return result;
}


//===================================================================
//	Area Methods
//===================================================================


//===================================================================
//  GetFixArea
//===================================================================
double MultiBody::GetFixArea() const
{
  return fixArea;
}

//===================================================================
//  GetFixAreas
//===================================================================
vector<double> MultiBody::GetFixAreas() const
{
  return fixAreas;
}

//===================================================================
//  GetFreeArea
//===================================================================
double MultiBody::GetFreeArea() const
{
  return freeArea;
}

//===================================================================
//  GetFreeAreas
//===================================================================
vector<double> MultiBody::GetFreeAreas() const
{
  return freeAreas;
}

//===================================================================
//  GetArea
//===================================================================
double MultiBody::GetArea() const
{
  return area;
}

//===================================================================
// CalculateArea
//===================================================================
void MultiBody::CalculateArea()
{
  fixArea = freeArea = 0;
  
  for(vector<shared_ptr<FixedBody> >::iterator I = fixedBody.begin(); I != fixedBody.end(); ++I) 
  {
    fixAreas.push_back((*I)->GetPolyhedron().area);
    fixArea += (*I)->GetPolyhedron().area;
  }

  for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin(); I != freeBody.end(); ++I) 
  {
    freeAreas.push_back((*I)->GetPolyhedron().area);
    freeArea += (*I)->GetPolyhedron().area;
  }

  area = fixArea + freeArea;
}


//===================================================================
//  Read
//===================================================================
void MultiBody::Read(istream& _is, int action, const char* descDir) 
{
  static const int FILENAME_LENGTH = 80;

  //get body info
  char string[32];
  readfield(_is, &string); //read "MultiBody"
  readfield(_is, &string); //read "Active/Passive"

  char cPeek = _is.peek();
  while((cPeek == ' ') || (cPeek == '\n')) {
    _is.get();
    cPeek = _is.peek();
  }

  bInternal = false;    
  if(cPeek =='I') {
    readfield(_is, &string);
    if (!strncmp(string, "Internal", 8)) 
      bInternal = true;
  }

  int bodyCount;
  readfield(_is, &bodyCount);
  vector<bool> isFree;

  double fixSum = 0;
  double freeSum = 0;
  for(int i=0; i<bodyCount; ++i) {
    vector<char*> comments;
    readfield(_is, &string, comments); //Tag FixedBody or FreeBody
    int BodyIndex;
    _is >> BodyIndex;

    char tmpFilename[FILENAME_LENGTH*2], bodyFileName[FILENAME_LENGTH];
    _is >> tmpFilename;
    strcpy(bodyFileName, descDir);
    strcat(bodyFileName, tmpFilename);
    VerifyFileExists(bodyFileName, action);

    if(!strncmp(string, "FixedBody", 10)) {
      isFree.push_back(false);
      
      Vector3D bodyPosition(_is);
      Vector3D origbodyOrientation(_is);
      Orientation bodyOrientation(Orientation::FixedXYZ,
				  origbodyOrientation[2]*TWOPI/360.0,
				  origbodyOrientation[1]*TWOPI/360.0,
				  origbodyOrientation[0]*TWOPI/360.0);
      Transformation transformation(bodyOrientation, bodyPosition);
      
      FixedBody fix(this);
      fix.Read(bodyFileName);
      fix.PutWorldTransformation(transformation);
      fixAreas.push_back(fix.GetPolyhedron().area);
      fixSum += fix.GetPolyhedron().area;
      AddBody(fix);      
    } else { // FreeBody
      isFree.push_back(true);

      if(i==0) {
	Vector3D bodyPosition(_is);
	Vector3D origbodyOrientation(_is);
	Orientation bodyOrientation(Orientation::FixedXYZ,
				    origbodyOrientation[2]*TWOPI/360.0,
				    origbodyOrientation[1]*TWOPI/360.0,
				    origbodyOrientation[0]*TWOPI/360.0);
	Transformation transformation(bodyOrientation, bodyPosition);
      }

      FreeBody free(this);
      free.Read(bodyFileName);
      freeAreas.push_back(free.GetPolyhedron().area);
      freeSum += free.GetPolyhedron().area;
      AddBody(free);
    } // endelse FreeBody    
  } //endfor i

  fixArea = fixSum;
  freeArea = freeSum;
  area = fixArea + freeArea;

  //get connection info
  readfield(_is, &string);     // Tag, "Connection"
  int connectionCount;
  _is >> connectionCount;   // # of connections

  for(int i=0; i<connectionCount; i++) {
    int previousBodyIndex, nextBodyIndex;
    _is >> previousBodyIndex;              // first body
    _is >> nextBodyIndex;                  // second body

    readfield(_is, &string);             // Tag, "Actuated/NonActuated"
      
    Vector3D transformPosition(_is);
    Vector3D angles(_is);
    Orientation transformOrientation = Orientation(Orientation::FixedXYZ,
						   angles[2]*TWOPI/360.0, 
						   angles[1]*TWOPI/360.0, 
						   angles[0]*TWOPI/360.0);
    
    DHparameters dhparameters;
    _is >> dhparameters.alpha;          // DH parameter, alpha
    _is >> dhparameters.a;              // DH parameter, a
    _is >> dhparameters.d;              // DH parameter, d
    _is >> dhparameters.theta;          // DH parameter, theta
    
    readfield(_is, &string);   // Tag, "Revolute" or "Prismatic"
    int connectionType;
    if (!strncmp(string, "Revolute", 9))
      connectionType = 0;              // Revolute type
    else
      connectionType = 1;              // Prismatic type
    
    Vector3D positionToDHFrame(_is);
    angles = Vector3D(_is);
    Orientation orientationToDHFrame = Orientation(Orientation::FixedXYZ,
						   angles[2]*TWOPI/360.0, 
						   angles[1]*TWOPI/360.0, 
						   angles[0]*TWOPI/360.0);
    
    shared_ptr<Body> prevBody;
    if(isFree[previousBodyIndex]) {
      int numFreeBeforeIndex = accumulate(isFree.begin(), isFree.begin()+previousBodyIndex, 0);
      prevBody = GetFreeBody(numFreeBeforeIndex);
    } else {
      int numFreeBeforeIndex = accumulate(isFree.begin(), isFree.begin()+previousBodyIndex, 0);
      prevBody = GetFixedBody(previousBodyIndex - numFreeBeforeIndex);
    }
    
    shared_ptr<Body> nextBody;
    if(isFree[nextBodyIndex]) {
      int numFreeBeforeIndex = accumulate(isFree.begin(), isFree.begin()+nextBodyIndex, 0);
      nextBody = GetFreeBody(numFreeBeforeIndex);
    } else {
      int numFreeBeforeIndex = accumulate(isFree.begin(), isFree.begin()+nextBodyIndex, 0);
      nextBody = GetFixedBody(nextBodyIndex - numFreeBeforeIndex);
    }
    
    Connection c(prevBody, nextBody);
    c.Read(prevBody, nextBody,
	    transformPosition, transformOrientation,
	    positionToDHFrame, orientationToDHFrame,
	    dhparameters, Connection::ConnectionType(connectionType));

    prevBody->Link(c);
  } //endfor i

  FindBoundingBox();
  ComputeCenterOfMass();
}


void MultiBody::buildCDstructure(cd_predefined cdtype, int nprocs)
{
  for(vector<shared_ptr<FixedBody> >::iterator I = fixedBody.begin(); I != fixedBody.end(); ++I)
    (*I)->buildCDstructure(cdtype, nprocs);
  for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin(); I != freeBody.end(); ++I)
    (*I)->buildCDstructure(cdtype, nprocs);
}


//===================================================================
//  Write
//===================================================================
void MultiBody::Write(ostream & _os) 
{
  //---------------------------------------------------------------
  // Write tag
  //---------------------------------------------------------------
  _os << "MultiBody" << endl;
  
  //---------------------------------------------------------------
  // Write bodies
  //---------------------------------------------------------------
  _os << fixedBody.size() << endl;
  for(vector<shared_ptr<FixedBody> >::iterator I = fixedBody.begin(); I != fixedBody.end(); ++I)
      (*I)->Write(_os);
  _os << freeBody.size() << endl;
  for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin(); I != freeBody.end(); ++I)
      (*I)->Write(_os);
  
  //---------------------------------------------------------------
  // Write links
  //---------------------------------------------------------------
  for(vector<shared_ptr<FixedBody> >::iterator I = fixedBody.begin(); I != fixedBody.end(); ++I) {
      _os << (*I)->ForwardConnectionCount() << endl;
      for(int j=0; j < (*I)->ForwardConnectionCount(); j++)
	    (*I)->GetForwardConnection(j).Write(_os);
  }
  for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin(); I != freeBody.end(); ++I) {
      _os << (*I)->ForwardConnectionCount() << endl;
      for(int j=0; j < (*I)->ForwardConnectionCount(); j++)
	    (*I)->GetForwardConnection(j).Write(_os);
  }
}


//===================================================================
//  ConfigureJoint
//
//  Function: Configure the joint by the given amount of displacement
//
//===================================================================
void MultiBody::ConfigureJoint(double * _s, int _dof) 
{
  for(size_t i = 0; i<(size_t)_dof; ++i)
    freeBody[i]->GetForwardConnection(0).GetDHparameters().theta = _s[i];
}


//===================================================================
//  ComputeCenterOfMass
//
//  The degree of approximation in calculating center of mass is
//  the same as in Body.cpp. To be more accurate, we need to
//  modify this function to consider the mass of each body.
// 
//===================================================================
void MultiBody::ComputeCenterOfMass()
{
  if(freeBody.empty() && fixedBody.empty()) 
  {
    cout << "\nERROR: No MultiBodies to take MultiBody::CenterOfMass from...\n";
  } 
  else
  {
     Vector3D sum(0,0,0);
     for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin(); I != freeBody.end(); ++I)
       sum = sum + (*I)->GetCenterOfMass();
     for(vector<shared_ptr<FixedBody> >::iterator I = fixedBody.begin(); I != fixedBody.end(); ++I)
       sum = sum + (*I)->GetCenterOfMass();
     CenterOfMass = sum / (freeBody.size() + fixedBody.size());
     CenterOfMassAvailable = true;
  }
}


//===================================================================
//  FindBoundingBox
//===================================================================
void MultiBody::FindBoundingBox()
{	
  double minx, miny, minz, maxx, maxy, maxz;
	
  ///////////////////////////////////////////////////////////
  //Check Free Bodys' Boudning Box
  if(!freeBody.empty())
  {
    freeBody.front()->FindBoundingBox();
    double* tmp = freeBody.front()->GetBoundingBox();
    minx = tmp[0]; maxx = tmp[1];
    miny = tmp[2]; maxy = tmp[3];
    minz = tmp[4]; maxz = tmp[5];
	
    for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin()+1; I != freeBody.end(); ++I)
    {
      (*I)->FindBoundingBox();
      tmp = (*I)->GetBoundingBox();
      minx = min(minx, tmp[0]); maxx = max(maxx, tmp[1]);
      miny = min(miny, tmp[2]); maxy = max(maxy, tmp[3]);
      minz = min(minz, tmp[4]); maxz = max(maxz, tmp[5]);
    }
  }
    
  ///////////////////////////////////////////////////////////
  //Check Fixed Bodys' Boudning Box
  if(!fixedBody.empty())
  {
    fixedBody.front()->FindBoundingBox();
    double* tmp = fixedBody.front()->GetBoundingBox();
    minx = tmp[0]; maxx = tmp[1];
    miny = tmp[2]; maxy = tmp[3];
    minz = tmp[4]; maxz = tmp[5];

    for(vector<shared_ptr<FixedBody> >::iterator I = fixedBody.begin()+1; I != fixedBody.end(); ++I)
    {
      (*I)->FindBoundingBox();
      tmp = (*I)->GetBoundingBox();
      minx = min(minx, tmp[0]); maxx = max(maxx, tmp[1]);
      miny = min(miny, tmp[2]); maxy = max(maxy, tmp[3]);
      minz = min(minz, tmp[4]); maxz = max(maxz, tmp[5]);
    }
  }

  ///////////////////////////////////////////////////////////
  //Pack
  boundingBox[0] = minx; boundingBox[1] = maxx;
  boundingBox[2] = miny; boundingBox[3] = maxy;
  boundingBox[4] = minz; boundingBox[5] = maxz;
	
  ///////////////////////////////////////////////////////////
  // Find maxAxisRange
  double rangex = maxx - minx;
  double rangey = maxy - miny;
  double rangez = maxz - minz;
  maxAxisRange = max(rangex, max(rangey,rangez));
}


//===================================================================
//  operator==
//===================================================================
bool MultiBody::operator==(const MultiBody& mb) const
{
  if(fixedBody.size() != mb.fixedBody.size())
    return false;
  for(size_t i=0; i < fixedBody.size(); ++i)
    if(*(fixedBody[i]) != *(mb.fixedBody[i]))
      return false;

  if(freeBody.size() != mb.freeBody.size())
    return false;
  for(size_t i=0; i < freeBody.size(); ++i)
    if(*(freeBody[i]) != *(mb.freeBody[i]))
      return false;

  return (bInternal == mb.bInternal) &&
         (CenterOfMass == mb.CenterOfMass) &&
         (boundingBox[0] == mb.boundingBox[0]) &&
         (boundingBox[1] == mb.boundingBox[1]) &&
         (boundingBox[2] == mb.boundingBox[2]) &&
         (boundingBox[3] == mb.boundingBox[3]) &&
         (boundingBox[4] == mb.boundingBox[4]) &&
         (boundingBox[5] == mb.boundingBox[5]) &&
         (maxAxisRange == mb.maxAxisRange);
}

#ifdef USE_SOLID
  void MultiBody::UpdateVertexBase(){
    for(vector<shared_ptr<FixedBody> >::iterator I = fixedBody.begin(); I != fixedBody.end(); ++I)
      (*I)->UpdateVertexBase();
    for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin(); I != freeBody.end(); ++I)
      (*I)->UpdateVertexBase();
  }
#endif

//==================================================================
//Polygonal Approximation
void MultiBody::PolygonalApproximation(vector<Vector3D>& result)
{
  result.clear();
  int i, nfree;

  nfree = GetFreeBodyCount();

  if(nfree > 0)
  {
    for(i=0; i<nfree-1; i++)
    {
      GMSPolyhedron first_bb = this->GetFreeBody(i)->GetWorldBoundingBox();
      vector<Vector3D> first_vertices = first_bb.vertexList;

      GMSPolyhedron second_bb = this->GetFreeBody(i)->GetForwardConnection(0).GetNextBody()->GetWorldBoundingBox();
      vector<Vector3D> second_vertices = second_bb.vertexList;

      vector<double> dis;
      vector<int> point;
      int j, k;
      //find the shortest distance for each vertex in one freebody to every vertex to the neighboring freebody and record the vertex pair
      for(k=0; k<8; k++)
      {
        dis.push_back((first_vertices[k]-second_vertices[0]).magnitude());
        point.push_back(0);
        for(j=1; j<8; j++)
        {
          if(((first_vertices[k]-second_vertices[j]).magnitude()) < dis[k])
          {
            dis.push_back((first_vertices[k]-second_vertices[j]).magnitude());
            point.push_back(j);
          }
        }
      }

      //find the 4 smallest distances
      for(k=0; k<4; k++)
      {
        int tmp = 0;
        for(j=1; j<8; j++)
 	{
          if(dis[tmp]<0 || dis[j]>dis[tmp])
            tmp = j;
        }
        dis[tmp] = -1;
      }

      double X, Y, Z;
      X=0;
      Y=0;
      Z=0;

      //average the x, y, z coordinates for 8 vertices
      for(j=0; j<8; j++)
      {
        if(dis[j]>=0)
        {
          X+=first_vertices[j].getX()+second_vertices[point[j]].getX();
          Y+=first_vertices[j].getY()+second_vertices[point[j]].getY();
          Z+=first_vertices[j].getZ()+second_vertices[point[j]].getZ();
        }
      }

      X /= 8.0;
      Y /= 8.0;
      Z /= 8.0;
      Vector3D res(X, Y, Z);
      result.push_back(res);
    }
  }
}

