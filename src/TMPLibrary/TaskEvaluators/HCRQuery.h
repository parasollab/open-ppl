#ifndef PPL_HCR_QUERY_H_
#define PPL_HCR_QUERY_H_

#include "TaskEvaluatorMethod.h"

class HCRQuery : public TaskEvaluatorMethod {

  public:
    ///@name Local Types
    ///@{
    ///@}
    ///@name Construction
    ///@{

    HCRQuery();

    HCRQuery(XMLNode& _node);

   ~ HCRQuery();

    ///@}

  protected: 
    ///@name Overrides
    ///@{

    virtual bool Run(Plan* _plan = nullptr) override;

    size_t temp{0};

    ///@}

  private:
    ///@name Helper Functions
    ///@{
    ///@}
    ///@name Internal State
    ///@{

    std::string m_sgLabel;

    ///@}

};

#endif
