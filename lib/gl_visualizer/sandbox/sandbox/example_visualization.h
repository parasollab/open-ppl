#ifndef EXAMPLE_SIMULATION_H_
#define EXAMPLE_SIMULATION_H_

#include "base_visualization.h"


class example_visualization : public base_visualization
{

  public:

    ///@name Construction
    ///@{

    example_visualization();
    virtual ~example_visualization() = default;

    ///@}
    ///@name Visualization Interface
    ///@{

    virtual void render() override;
    virtual void start() override;
    virtual void reset() override;

    ///@}

};

#endif
