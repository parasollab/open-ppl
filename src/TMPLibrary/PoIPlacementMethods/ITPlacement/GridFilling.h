#ifndef GRID_FILLING_H_
#define GRID_FILLING_H_

#include "TMPLibrary/PoIPlacementMethods/ITPlacement/ITPlacementMethod.h"

#include "Utilities/XMLNode.h"

class GridFilling : public ITPlacementMethod {

  public:

    ///@name Construction
    ///@{

    GridFilling(MPProblem* _problem);

    GridFilling(XMLNode& _node);

    ~GridFilling() = default;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution) override;

    ///@}
  protected:

    void CreateGrid(/*Space _spaceType*/);
    void TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg);

  private:

    Environment* m_environment;
    const Boundary* m_boundary;
    vector<Cfg> m_locations;
    int m_squaresX, m_squaresY; //in how many squares/parts we want the env. divided



};

#endif
