// $Id$

/**
 * @file DistanceMetrics.h
 *
 * @author Daniel Vallejo
 * @date 8/21/1998
 */

////////////////////////////////////////////////////////////////////////////////////////////

#ifndef DistanceMetrics_h
#define DistanceMetrics_h

//////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers

#include "OBPRMDef.h"

/////////////////////////////////////////////////////////////////////////////////////////

class Cfg;
class MultiBody;
class Input;
class Environment;
class n_str_param;

class DistanceMetricMethod;
class EuclideanDistance;
class ScaledEuclideanDistance;
class MinkowskiDistance;
class ManhattanDistance;
class CenterOfMassDistance;

const int CS = 0;   ///< Type CS: Configuration space distance metric
const int WS = 1;   ///< Type WS: Workspace distance metric 


/**This is the main distance metric class.  It contains two vectors: all 
  *and selected.  all contains all of the different types of distance 
  *metric methods.  selected contains only those selected by the user.
  */
class DistanceMetric {
 public:
  DistanceMetric();
  ~DistanceMetric();

  static vector<DistanceMetricMethod*> GetDefault();

  int ReadCommandLine(n_str_param* DMstrings[MAX_DM], int numDMs);
  void PrintUsage(ostream& _os) const;
  void PrintValues(ostream& _os) const;
  void PrintDefaults(ostream& _os) const;

  /**Read information about DistanceMetricMethods selected from file.
    *@param _fname filename for data file.
    *@see ReadDMs(istream& _myistream)
    */
  void ReadDMs(const char* _fname);
  /**Read information about DistanceMetricMethods selected from input stream.
    *@see WriteDMs for data format
    *@note if this method could not be able to understand
    *input file format, then error message will be post to 
    *standard output.
    */
  void ReadDMs(istream& _myistream);
  /**Ouput information about selected DistanceMetricMethods to file.
    *@param _fname filename for data file.
    *@see WriteDMs(ostream& _myostream)
    */
  void WriteDMs(const char* _fname) const;
  /**Ouput information about selected DistanceMetricMethods to output stream.
    *@note format: DM_NAME (a string) DM_PARMS (double, int, etc) 
    *for each DistanceMetricMethod.
    *if scaledEuclidean is encountered, then sValue will be printed.
    *if minkowski is encountered, r1, r2, and r3 will be printed.
    */
  void WriteDMs(ostream& _myostream) const;

  /**Get Distance between two Configurations.
    *The distance is calculated by DistanceMetricMethod in selected vector. 
    *
    *@param _c1 Start Cfg
    *@param _c2 End Cfg
    *@return distance metrics between _c1 and _c2.
    *
    *@note there is the framework for multiple selected distance metrics, 
    *but now it only uses the first one in the selected vector.
    */
  virtual double Distance(Environment* env, const Cfg& _c1, const Cfg& _c2);
  virtual double Distance(Environment* env, const Cfg* _c1, const Cfg* _c2);

 protected:
  bool ParseCommandLine(int argc, char** argv);

  vector<DistanceMetricMethod*> all;
  vector<DistanceMetricMethod*> selected;
};


/**This is the interface for all distance metric methods(euclidean, 
  *scaledEuclidean, minkowski, manhattan, com, etc.).
  */
class DistanceMetricMethod {
 public:
  DistanceMetricMethod();
  ~DistanceMetricMethod();

  virtual char* GetName() const = 0;
  virtual void SetDefault() = 0;

  virtual bool operator==(const DistanceMetricMethod& dm) const;

  virtual void ParseCommandLine(int argc, char** argv) = 0;
  virtual void PrintUsage(ostream& _os) const = 0;
  virtual void PrintValues(ostream& _os) const = 0;
  virtual DistanceMetricMethod* CreateCopy() = 0;

  virtual double Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2) = 0;
 protected:
  int type; ///<WS or CS. Used to classify metrics.
};
ostream& operator<< (ostream& _os, const DistanceMetricMethod& dm);


/**This computes the euclidean distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class EuclideanDistance : public DistanceMetricMethod {
 public:
  EuclideanDistance();
  ~EuclideanDistance();

  virtual char* GetName() const;
  virtual void SetDefault();

  virtual void ParseCommandLine(int argc, char** argv);
  virtual void PrintUsage(ostream& _os) const;
  virtual void PrintValues(ostream& _os) const;
  virtual DistanceMetricMethod* CreateCopy();

  /**This method calculates 
    *sqrt((c11-c21)^2+(c12-c22)^2+(c13-c23)^2....+(c1n-c2n)^2).
    *
    *Here c1i and c2i are elements for each dimension and both of them
    *have n dimension. 
    *
    *@see Cfg::PositionMagnitude and OrientationMagnitude
    */
  virtual double Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2);

 protected:
  virtual double ScaledDistance(const Cfg& _c1, const Cfg& _c2, double sValue);
};


/**This computes the scaled euclidean distance between two cfgs.  This 
  *class is derived off of DistanceMetricMethod.
  */
class ScaledEuclideanDistance : public EuclideanDistance {
 public:
  ScaledEuclideanDistance();
  ScaledEuclideanDistance(double _sValue);
  ~ScaledEuclideanDistance();

  virtual char* GetName() const;
  virtual void SetDefault();

  virtual bool operator==(const ScaledEuclideanDistance& dm) const;

  virtual void ParseCommandLine(int argc, char** argv);
  virtual void PrintUsage(ostream& _os) const;
  virtual void PrintValues(ostream& _os) const;
  virtual DistanceMetricMethod* CreateCopy();

  /**This method calculates 
    *sqrt(s*(Position Magnitude)^ + (1-s)*(Orientation Magnitude)^2)
    *Position Magnitude is eulidean distance of position part of dimensions.
    *Position Magnitude is eulidean distance of orientation part of dimensions.
    *Usually first 3 dimensions are positions and rest of them are orientations.
    *
    *@see Cfg::PositionMagnitude and OrientationMagnitude
    */
  virtual double Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2);

  double GetS() const { return sValue; }

 protected:
  /**Scale for Euludean distance of position part. 
    *Should between [0,1].
    */
  double sValue; ///<For Euclidean distance of position part.
};


/**This computes the minkowski distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class MinkowskiDistance : public DistanceMetricMethod {
 public:
  MinkowskiDistance();
  ~MinkowskiDistance();

  virtual char* GetName() const;
  virtual void SetDefault();

  bool operator==(const MinkowskiDistance& dm) const;

  virtual void ParseCommandLine(int argc, char** argv);
  virtual void PrintUsage(ostream& _os) const;
  virtual void PrintValues(ostream& _os) const;
  virtual DistanceMetricMethod* CreateCopy();

  /**This method calculates 
    *pow( (c11-c21)^r1+(c12-c22)^r1+(c13-c23)^r1+(c14-c24)^r2....+(c1n-c2n)^r2 , r3)
    *
    *Here c1i and c2i are elements for each dimension and both of them
    *have n dimension. 
    *Usually first 3 dimensions are positions and rest of them are orientations.
    *position part using r1 as power factor, and orientation part using r2 as power factor.
    *usually, r1=r2 and r3=1/r1.
    */
  virtual double Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2);

  double GetR1() const { return r1; }
  double GetR2() const { return r2; }
  double GetR3() const { return r3; }

 protected:
  /**Power factors for Minkowski Distace.
    */
  double r1; ///<For position part.
  double r2; ///<For rotation part.
  double r3; ///<For calculating root.
};


/**This computes the manhattan distance between two cfgs.  This class is 
  *derived off of DistanceMetricMethod.
  */
class ManhattanDistance : public DistanceMetricMethod {
 public:
  ManhattanDistance();
  ~ManhattanDistance();

  virtual char* GetName() const;
  virtual void SetDefault();

  virtual void ParseCommandLine(int argc, char** argv);
  virtual void PrintUsage(ostream& _os) const;
  virtual void PrintValues(ostream& _os) const;
  virtual DistanceMetricMethod* CreateCopy();

  /**This method calculates 
    *( |c11-c21|+|c12-c22|+...+|c1n-c2n| ).
    *
    *Here |A| is absolute value of A.
    */
  virtual double Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2);
};


/**This computes the euclidean distance of only the position part between 
  *two cfgs.  This class is derived off of DistanceMetricMethod.
  */
class CenterOfMassDistance : public DistanceMetricMethod {
 public:
  CenterOfMassDistance();
  ~CenterOfMassDistance();

  virtual char* GetName() const;
  virtual void SetDefault();

  virtual void ParseCommandLine(int argc, char** argv);
  virtual void PrintUsage(ostream& _os) const;
  virtual void PrintValues(ostream& _os) const;
  virtual DistanceMetricMethod* CreateCopy();

  /**This method calculates
    *sqrt((c11-c21)^2+(c12-c22)^2+(c13-c23)^2).
    *This method only Euclidean Distance of position part and 
    *assumed that the first 3 dimension of Cfg are for position.
    */
  virtual double Distance(MultiBody* robot, const Cfg& _c1, const Cfg& _c2);
};

#endif

