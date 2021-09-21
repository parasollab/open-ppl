#include "KDLParser.h"
#include "PMPLExceptions.h"

#include <iostream>
#include <kdl/tree.hpp>

KDL::Tree
ParseKDLFromFile(const std::string _filename) {
  KDL::Tree tree;
  if(!kdl_parser::treeFromFile(_filename,tree)) {
    throw RunTimeException(WHERE) << "Unable to parse kdl filename: "
                                  << _filename 
                                  << std::endl;
  }
  return tree;
}
