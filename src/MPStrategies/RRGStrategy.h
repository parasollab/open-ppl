#ifndef RRGSTRATEGY_H_
#define RRGSTRATEGY_H_

#include "BasicRRTStrategy.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class RRGStrategy : public BasicRRTStrategy {
  public:
    RRGStrategy(XMLNode& _node, MPProblem* _problem);
    virtual ~RRGStrategy() {}

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;
    virtual VID ExpandTree(CfgType& _dir);

  private:
    string m_nc;  // Contains the Node Connection Method string
};

#endif
