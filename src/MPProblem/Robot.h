#ifndef ROBOT_H_
#define ROBOT_H_

#include <vector>
#include <string>

using namespace std;

struct Robot {
  enum Base {PLANAR, VOLUMETRIC, FIXED, JOINT}; //2D plane vs 3D
  enum BaseMovement {ROTATIONAL, TRANSLATIONAL}; //rotation+translation, just translation, no movement
  enum JointType {REVOLUTE, SPHERICAL, NONACTUATED}; //1dof vs 2dof rotational joints
  typedef vector<pair<pair<size_t, size_t>, JointType> > JointMap; //size_t is Joint
                                                          //index of next body, 
                                                          //joint type
  typedef JointMap::iterator JointIT;

  Base m_base; //Base Type
  BaseMovement m_baseMovement; //can the base rotate? is the base fixed?
  JointMap m_joints; //Joints associated with robot
  int m_bodyIndex; //free body index for base

  Robot(Base _base, BaseMovement _baseMovement, 
      JointMap _joints, int _bodyIndex); 

  static Base GetBaseFromTag(const string _tag);      
  static BaseMovement GetMovementFromTag(const string _tag);  
  static JointType GetJointTypeFromTag(const string _tag);

  static string GetTagFromBase(const Base& _b);
  static string GetTagFromMovement(const BaseMovement& _bm);
  static string GetTagFromJointType(const JointType& _jt);

  public:
#ifdef _PARALLEL
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
