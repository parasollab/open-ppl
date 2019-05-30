#ifndef PLACEMENT_METHOD_H_
#define PLACEMENT_METHOD_H_

//#include "Behaviors/Agents/Coordinator.h"
#include "MPLibrary/MPLibrary.h"
#include "MPLibrary/MPSolution.h"
#include "MPLibrary/MPTools/InteractionTemplate.h"
#include "MPLibrary/PMPL.h"
#include "Utilities/XMLNode.h"

class Coordinator;
class TMPStrategyMethod;

class PlacementMethod {

  public:

    //typedef typename MPSolutionType<MPTraits<Cfg, DefaultWeight<Cfg>>> MPSolution;

    ///@name Construction
    ///@{

		PlacementMethod() = default;

    PlacementMethod(MPProblem* _problem);

    //PlacementMethod(MPProblem* _problem, XMLNode& _node);

    ~PlacementMethod() = default;
		
		virtual std::unique_ptr<PlacementMethod> Clone();

    /// Create a dynamically-allocated placement method from an XML node.
    /// @param _p The problem which this method will be applied to.
    /// @param _node The XML node to parse.
    /// @return A plaement method of the type specified by _node.
    static std::unique_ptr<PlacementMethod> Factory(MPProblem* _p, XMLNode& _node);

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library,
                         TMPStrategyMethod* _tmpMethod);

    std::string GetLabel();

    ///@}


  protected:

    MPLibrary* m_library;

    MPProblem* m_problem;

    std::string m_label;

    bool m_debug{false};

};

#endif

