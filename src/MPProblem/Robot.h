#ifndef ROBOT_H_
#define ROBOT_H_

#include <vector>
#include <string>
#include "boost/shared_ptr.hpp"
using boost::shared_ptr;

using namespace std;

class Connection;

struct Robot {
  enum Base {PLANAR, VOLUMETRIC, FIXED, JOINT}; //2D plane vs 3D
  enum BaseMovement {ROTATIONAL, TRANSLATIONAL}; //rotation+translation, just translation, no movement
  
  typedef shared_ptr<Connection> Joint;
  typedef vector<Joint> JointMap;
  typedef JointMap::iterator JointIT;

  Base m_base; //Base Type
  BaseMovement m_baseMovement; //can the base rotate? is the base fixed?
  JointMap m_joints; //Joints associated with robot
  int m_bodyIndex; //free body index for base

  Robot(Base _base, BaseMovement _baseMovement, 
      JointMap _joints, int _bodyIndex); 

  static Base GetBaseFromTag(const string _tag);      
  static BaseMovement GetMovementFromTag(const string _tag);  

  static string GetTagFromBase(const Base& _b);
  static string GetTagFromMovement(const BaseMovement& _bm);

#ifdef _PARALLEL
  public:
    void define_type(stapl::typer &_t)  
    {
      _t.member(m_base);
      _t.member(m_baseMovement);
      _t.member(m_joints);
      _t.member(m_bodyIndex);
    }
#endif
};

#endif
