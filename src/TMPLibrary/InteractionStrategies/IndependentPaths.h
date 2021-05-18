#ifndef _PPL_INDEPENDENT_PATHS_H_
#define _PPL_INDEPENDENT_PATHS_H_

#include "InteractionStrategyMethod.h"

class IndependentPaths : public InteractionStrategyMethod {

  public:

    ///@name Construction
    ///@{

    IndependentPaths();

    IndependentPaths(XMLNode& _node);

    ~IndependentPaths();

    ///@}
    ///@name Interface
    ///@{

    ///@}

  private:
    string m_samplerLabel;

};
#endif
