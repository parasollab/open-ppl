#ifndef PRINT_MAP_EVALUATION_H
#define PRINT_MAP_EVALUATION_H

#include "MapEvaluatorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Prints the roadmap to a file
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
class PrintMapEvaluation : public MapEvaluatorMethod {

  public:

    /// Constructs a PrintMapEvaluation object
    PrintMapEvaluation();

    /// Constructs a PrintMapEvaluation object with a base filename
    /// @param _basename The base name of the output file.
    PrintMapEvaluation(string _baseName);

    /// Constructs a PrintMapEvaluation object from an XML node
    /// @param _node The XML node to use.
    PrintMapEvaluation(XMLNode& _node);

    virtual ~PrintMapEvaluation();

    virtual void Print(ostream& _os) const;

    virtual bool operator()();

  protected:

    string m_baseName; ///< The basename of the file to write
};

#endif
