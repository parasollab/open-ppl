#ifndef PMPL_MP_STRATEGY_METHOD_H_
#define PMPL_MP_STRATEGY_METHOD_H_

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

// #include <cstddef>
// #include <iostream>
// #include <string>
// #include <vector>


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref MotionPlanningStrategies.
///
/// MPStrategyMethod has one main function, @c operator(), which
/// performs preprocessing, processing, and postprocessing functionalities of
/// the motion planning algorithm.
///
/// @usage
/// @code
/// MPStrategyPointer mps = this->GetMPStrategy(m_mpsLabel);
/// (*mps)(); //call as a function object
/// mps->operator()(); //call with pointer notation
/// @endcode
///
/// @todo Incorporate path constraints when generating the start and goal.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
class MPStrategyMethod : public MPBaseObject {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef size_t VID;

    ///@}
    ///@name Construction
    ///@{

    MPStrategyMethod() = default;

    MPStrategyMethod(XMLNode& _node);

    virtual ~MPStrategyMethod();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name Interface
    ///@{

    using MPBaseObject::Initialize;

    /// Execute the strategy by calling Initialize, Run, and Finalize.
    void operator()();
    ///@example MPStrategies_UseCase.cpp
    /// This is an example of how to use the MPStrategies methods

    /// Set output file writing to on or off (on by default). This is used to
    /// suppress generation of roadmap, path, and stat files.
    /// @param _enable True to enable, false to disable.
    void EnableOutputFiles(const bool _enable = true);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    virtual void Run();            ///< Call Iterate until EvaluateMap is true.
    virtual bool EvaluateMap();    ///< Check if we satisfied all map evaluators.
    virtual void Iterate() {}      ///< Execute one iteration of the strategy.
    virtual void Finalize();       ///< Clean-up and output results.

    virtual void ClearRoadmap();   ///< Pre-clear the roadmap(s) if requested.

    ///@}
    ///@name Start/Goal Generation
    ///@{

    /// Generate a 'start' node for the task and add it to the roadmap.
    /// @param _samplerLabel The label for the sampler to use if no query
    ///                      sampler was provided, or empty to require a query
    ///                      sampler.
    /// @return The VID of the generated configuration.
    /// @note This returns size_t so that it will work for both individual and
    ///       group strategies (all VID types are typedefs for size_t).
    virtual size_t GenerateStart(const std::string& _samplerLabel = "");

    /// Generate 'goal' node(s) for the task and add it(them) to the roadmap.
    /// @param _samplerLabel The label for the sampler to use if no query
    ///                      sampler was provided, or empty to require a query
    ///                      sampler.
    /// @return The VIDs of the generated configurations.
    /// @note This returns size_t so that it will work for both individual and
    ///       group strategies (all VID types are typedefs for size_t).
    virtual std::vector<size_t> GenerateGoals(
        const std::string& _samplerLabel = "");

    ///@}
    ///@name Internal State
    ///@{

    std::string m_querySampler;          ///< Sampler for generating start/goal.
    std::vector<std::string> m_meLabels; ///< The list of map evaluators to use.
    size_t m_iterations{0};              ///< The number of executed iterations.
    bool m_writeOutput{true};            ///< Write output at the end?
    bool m_clearMap{false};              ///< Clear the roadmap(s) before run?

    ///@}

    /// Needs access to Iterate for other methods.
    template <typename> friend class AOAnalyzer;

};

#endif
