#ifndef AVERAGE_ESTIMATOR_H
#define AVERAGE_ESTIMATOR_H

#include "StateEstimator.h"

class SensorInterface;

////////////////////////////////////////////////////////////////////////////////
/// Extension of a state estimator object, which will average all sensor
/// readings and update a robot's state according to this average. 
////////////////////////////////////////////////////////////////////////////////
class AverageEstimator : public StateEstimator {

  public:

    ///@name Construction
    ///@{

    AverageEstimator(Robot* const _robot);

    virtual ~AverageEstimator();

    ///@}
    ///@name StateEstimator Overrides
    ///@{
    
    /// Apply the most recent observations from a sensor to the current
    /// estimated state.
    /// @param _sensor The sensor to take observations from.
    virtual void ApplyObservations(SensorInterface* const _sensor) override;

    ///@}

  private:

    ///@name Estimation Helpers
    ///@{

    double ComputeRotation(SensorInterface* const _sensor);

    ///@}

};

#endif
