#ifndef RRGSTRATEGY_H_
#define RRGSTRATEGY_H_

#include "BasicRRTStrategy.h"

class RRGStrategy : public BasicRRTStrategy {
  public:
    RRGStrategy(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~RRGStrategy() {}

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os) const;
    virtual VID ExpandTree(CfgType& _dir);

  private:
    string m_nc;  // Contains the Node Connection Method string
};

#endif
