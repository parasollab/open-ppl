#ifndef IT_PLACEMENT_METHOD_H_
#define IT_PLACEMENT_METHOD_H_

#include "TMPLibrary/PoIPlacementMethods/PoIPlacementMethod.h"

#include "Utilities/XMLNode.h"

class TMPStrategyMethod;

class ITPlacementMethod : public PoIPlacementMethod {

  public:

    //typedef typename MPSolutionType<MPTraits<Cfg, DefaultWeight<Cfg>>> MPSolution;

    ///@name Construction
    ///@{

    ITPlacementMethod(XMLNode& _node);

    ITPlacementMethod() = default;

    ~ITPlacementMethod() = default;

    virtual std::unique_ptr<ITPlacementMethod> Clone();

    /// Create a dynamically-allocated placement method from an XML node.
    /// @param _p The problem which this method will be applied to.
    /// @param _node The XML node to parse.
    /// @return A plaement method of the type specified by _node.
    static std::unique_ptr<ITPlacementMethod> Factory(XMLNode& _node);

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution) override;

    ///@}

};

#endif

