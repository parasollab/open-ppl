#ifndef PPL_DISCRETE_EXTENDER_H_
#define PPL_DISCRETE_EXTENDER_H_

#include "ExtenderMethod.h"

#include <math.h>

////////////////////////////////////////////////////////////////////////////////
/// Discrete extender designed for use in dRRT.
///
/// TODO::Description
///
/// Currently only supports group cfgs.
///
/// @ingroup Extenders
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class DiscreteExtender : public ExtenderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType         CfgType;
    typedef typename MPTraits::GroupCfgType    GroupCfgType;
    typedef typename MPTraits::GroupWeightType GroupWeightType;
    typedef std::vector<size_t>                RobotFormation;

    ///@}
    ///@name Construction
    ///@{

    DiscreteExtender();

    DiscreteExtender(XMLNode& _node);

    ~DiscreteExtender();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    ///@}
    ///@name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) override;

    virtual bool Extend(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupCfgType& _new, GroupLPOutput<MPTraits>& _lp,
        const RobotFormation& _robotIndexes = RobotFormation()) override;

    virtual bool Extend(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupCfgType& _new, GroupLPOutput<MPTraits>& _lp, CDInfo& _cdInfo,
        const RobotFormation& _robotIndexes = RobotFormation()) override;

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    ///@}
    ///@name Internal State
    ///@{

    std::string m_lpLabel;

    ///@}
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
DiscreteExtender<MPTraits>::
DiscreteExtender() {
  this->SetName("DiscreteExtender");
}

template <typename MPTraits>
DiscreteExtender<MPTraits>::
DiscreteExtender(XMLNode& _node) : ExtenderMethod<MPTraits>(_node) {
  this->SetName("DiscreteExtender");

  m_lpLabel = _node.Read("lp", true, "", "Local planner to use to check "
                         "tensor product roadmap edges.");

}

template <typename MPTraits>
DiscreteExtender<MPTraits>::
~DiscreteExtender() {}

/*-------------------------- MPBaseObject Overrides --------------------------*/


/*-------------------------- Extender Overrides --------------------------*/

template <typename MPTraits>
bool
DiscreteExtender<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) {
  throw NotImplementedException(WHERE);
}

template <typename MPTraits>
bool
DiscreteExtender<MPTraits>::
Extend(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupCfgType& _new, GroupLPOutput<MPTraits>& _lp,
        const RobotFormation& _robotIndexes) {

  CDInfo cdInfo;
  return Extend(_start,_end,_new,_lp,cdInfo,_robotIndexes);
}

template <typename MPTraits>
bool
DiscreteExtender<MPTraits>::
Extend(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupCfgType& _new, GroupLPOutput<MPTraits>& _lp, CDInfo& _cdInfo,
        const RobotFormation& _robotIndexes) {

  
  auto computeAngle = [](Cfg& _cfg1, Cfg& _cfg2) {
    double dot = 0;
    for(size_t i = 0; i < _cfg1.DOF(); i++) {
      dot += _cfg1[i] * _cfg2[i];
    }
    double cos = dot / (_cfg1.Magnitude() * _cfg2.Magnitude());
    return abs(acos(cos));
  };

  // Look at tensor product neighbors of start
  auto robots = _start.GetRobots();
  for(size_t i = 0; i < robots.size(); i++) {
    auto robot = robots[i];
    auto rm = this->GetRoadmap(robot);

    auto vid = _start.GetVID(i);
    if(vid == MAX_INT)
      throw RunTimeException(WHERE) << "Expecting _start to be a tensor product vertex.";

    auto cfg1 = _start.GetRobotCfg(i);
    auto vec1 = _end.GetRobotCfg(i) - cfg1;

    // Initialize minimum neighbor as staying put
    Cfg empty(robot);
    double minAngle = computeAngle(vec1,empty);
    size_t newVID = vid;

    // Check each of the neighbors in the robot roadmap
    auto vit = rm->find_vertex(vid);
    for(auto eit = vit->begin(); eit != vit->end(); eit++) {

      // Grab neighboring vertex
      auto target = eit->target();
      auto cfg2 = rm->GetVertex(target);
      auto vec2 = cfg2 - cfg1;

      double angle = computeAngle(vec1,vec2);

      // Check if this is a new min angle
      if(angle >= minAngle) 
        continue;

      // If yes, save
      minAngle = angle;
      newVID = target;
    }
    
    _new.SetRobotCfg(i,newVID);
  }
  
  // Compute Local Plan between _start and _new

  auto lp = this->GetLocalPlanner(m_lpLabel);
  GroupCfgType col(this->GetGroupRoadmap());
  auto env = this->GetMPProblem()->GetEnvironment();

  auto isConnected = lp->IsConnected(_start,_new,col,&_lp,
                         env->GetPositionRes(),
                         env->GetOrientationRes(),
                         true, false, _robotIndexes);

  return isConnected;
}

/*----------------------------------------------------------------------------*/
#endif
